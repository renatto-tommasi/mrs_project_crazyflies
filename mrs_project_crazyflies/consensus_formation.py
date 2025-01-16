import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Twist, PoseStamped, Point
from nav_msgs.msg import Odometry



import numpy as np

class ConsensusFormationController(Node):
    def __init__(self):
        super().__init__('consensus_formation')

        self.get_logger().info(f"Consensus Formation Controller started correctly!")

        self.num_of_robots = 4 # Manual for now

        self.vel = {}               # Boid Velocities
        self.X = {}                 # Boid Locations
        self.formation = None
        self.A = None

        self.target = np.array([-1,1])

        self.dt = 0.1

        # PUBLISHERS
        self.vel_publishers = {f"cf_{i+1}": self.create_publisher(Twist, f"/cf_{i+1}/cmd_vel", 10) for i in range(self.num_of_robots)}
    

        # SUBSCRIBERS
        self.odom_subscribers = [self.create_subscription(Odometry, f"/cf_{i+1}/odom",self.store_odom, 10) for i in range(self.num_of_robots)]

        # # TIMERS
        self.delay = 5      # seconds
        self.delay_counter = 0
        self.timer = self.create_timer(self.dt, self.control_loop)

    def store_odom(self, msg:Odometry):
        """
        Description: This function forms a vector with the location of all the boids

        Output:
            X: nd.array (n,2) -> locations vector
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        frame = msg.child_frame_id
        self.X[frame] = np.array([x,y,z])

        if z < 0.5:
            self.get_logger().info(f"{frame} has not launched, attempting again")
            self.launch_drones()
            
        # self.get_logger().info(f"{frame} has x:{x}, y:{y}, z:{z}")

    def set_topology(self, topology):
        """
        Description: This function stablishes the topology of the communication between boids
        Input: 
            topology: string 

        Update:
            self.A: nd.array (n,n) -> adjacency matrix
        """
        if topology == 1:
            self.A = np.array([[0, 1, 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1],
                             [1, 0, 0, 0]])  # Ring topology
        elif topology == 2:
            self.A = np.array([[0, 1, 0, 1],
                             [1, 0, 1, 0],
                             [0, 1, 0, 1],
                             [1, 0, 1, 0]])  # Fully connected directed topology
        else:
            raise ValueError("Invalid choice. Please select 1 or 2.")


    def set_formation(self, formation):
        """
        Description: This function stablishes the formation to be obtained
        Input: 
            formation: string -> triangle, square, ... etc

        Output:
            xi: nd.array (n,2) -> positions vector
        """
        scale = 2
        if formation == 1:  # Triangle
            formation = np.array([[0, 0],
                                [1, 1],
                                [1, 2],
                                [2, 0]])
        elif formation == 2:  # Line
            formation = np.array([[0, 0],
                                [1, 0],
                                [2, 0],
                                [3, 0]])/scale
        elif formation == 3:  # Square
            formation = np.array([[0, 0],
                                [1, 0],
                                [1, 1],
                                [0, 1]])/scale
        else:
            raise ValueError("Invalid formation type. Please select 1, 2, or 3.")

        self.formation = formation # Center the formation

    def calculate_formation_vel(self, X, xi):
        """
        Description: This function calculate the velocities to achieve the formation for each robot
        Input: 
            X: nd.array (n,2) -> locations vector
            xi: nd.array (n,2) -> positions vector

        Output:
            V: nd.array (n,2) -> velocities vector
        """
        V = np.zeros_like(X[:, :2])

        degree_matrix = np.diag(np.sum(self.A, axis=1))
        laplacian_matrix = degree_matrix - self.A

        self.get_logger().error(f"L: {laplacian_matrix}, X: {X[:,:2]}, xi:{xi}")


        V = -np.dot(laplacian_matrix, (X[:,:2] - xi))*self.dt

        return V

    def to_world_frame(self, xi):
        
        origin = self.X["cf_1"][:2]  # Get the first 2 elements
        transform = np.zeros_like(xi)
        for i in range(self.num_of_robots):
            transform[i] = xi[i] + origin
        return transform

    def update_vel(self):
        for drone, velocity in self.vel.items():
            twist_vel = Twist()
            twist_vel.linear.x = velocity[0]
            twist_vel.linear.y = velocity[1]


            self.vel_publishers[drone].publish(twist_vel)

    def control_loop(self):
        """Main control loop for consensus-based formation control."""
        if self.delay_counter < self.delay/self.dt:
            self.delay_counter += 1
            return
        self.get_logger().error("Started Consensus Formation Protocol")
        # Current positions
        X = self._dictionary_to_matrix(self.X)
        # Desired positions
        xi = self.formation
        if X.size == 0:
            raise ValueError("Tf transforms are not being published. Restart the simulation!")
        if xi is None:
            self.get_logger().error("X or xi is None. Ensure proper initialization.")
            return
        # self.get_logger().info(f"Robots position: current={X}, desired={xi}")
        # Calculate velocities
        V = self.calculate_formation_vel(X, xi)
        # Send velocities
        self.vel = self._matrix_to_dictionary(V)

        self.get_logger().info(f"Robots Velocity: current={self.vel}")

        self.update_vel()

    
    def launch_drones(self):
        """
        Description: This function publishes the velocity to each boid
        Input: 
            V: nd.array (n,2) -> velocities vector

        """
        linear_z = float(0.5)


        # Create the Twist message
        twist_msg = Twist()
        twist_msg.linear.z = linear_z
        
        self.get_logger().info(f"Launching drones")

        for publisher in self.vel_publishers.values():
            publisher.publish(twist_msg)
        
    def _dictionary_to_matrix(self, dictionary):

        matrix = np.array([])  # Initialize an empty array
        for i in range(len(dictionary)):
            if i == 0:
                matrix = dictionary[f"cf_{i+1}"] 
            else:
                matrix = np.vstack((matrix, dictionary[f"cf_{i+1}"])) 

        return matrix

    def _matrix_to_dictionary(self, matrix):

        dictionary = {}
        for i in range(matrix.shape[0]):  # Iterate over the rows of the matrix
            dictionary[f"cf_{i+1}"] = matrix[i, :]  # Assign each row to a key
        return dictionary




def main(args=None):
    rclpy.init(args=args)
    topology = 1         # Define several topologies
    formation = 1
    # Instantiate the Consensus Controller
    controller = ConsensusFormationController()
    controller.launch_drones()
    controller.set_topology(topology)
    controller.set_formation(formation)

    # Keep the node alive until manually interrupted
    rclpy.spin(controller)

    # Shutdown
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
