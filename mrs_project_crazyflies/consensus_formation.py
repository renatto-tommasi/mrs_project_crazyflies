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
        self.goal = None

        self.leader = "cf_4"

        self.target = np.array([-1,1])

        self.dt = 0.1

        # PUBLISHERS
        self.vel_publishers = {f"cf_{i+1}": self.create_publisher(Twist, f"/cf_{i+1}/consensus_vel", 10) for i in range(self.num_of_robots)}
    

        # SUBSCRIBERS
        self.odom_subscribers = [self.create_subscription(Odometry, f"/cf_{i+1}/odom",self.store_odom, 10) for i in range(self.num_of_robots)]
        self.goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self.set_goal, 10)

        # # TIMERS
        self.delay = 2     # seconds
        self.delay_counter = 0
        self.timer = self.create_timer(self.dt, self.control_loop)

    def set_goal(self, goal: PoseStamped):
        x_goal_wf = goal.pose.position.x
        y_goal_wf = goal.pose.position.y
        self.goal = np.array([x_goal_wf, y_goal_wf])
        

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
        if self.num_of_robots == 4:
            if topology == 1:
                self.A = np.array([[0, 1, 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1],
                                [1, 0, 0, 0]])  # Ring topology
            elif topology == 2:
                self.A = np.array([[0, 1, 1, 1],
                                [1, 0, 1, 1],
                                [1, 1, 0, 1],
                                [1, 1, 1, 0]])  # Fully connected directed topology
            else:
                raise ValueError("Invalid choice. Please select 1 or 2.")
        elif self.num_of_robots == 3:
            if topology == 1:
                self.A = np.array([[0, 1, 0],
                                [0, 0, 1],
                                [1, 0, 0]])   # Ring topology
            elif topology == 2:
                self.A = np.array([[0, 1, 1],
                                [1, 0, 1],
                                [1, 1, 0]])  # Fully connected directed topology
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
        if self.num_of_robots == 4:
            if formation == "triangle":  # Triangle
                formation = np.array([[0, 0],
                                    [1, 1],
                                    [1, 2],
                                    [2, 0]])
            elif formation == "line":  # Line
                formation = np.array([[0, 0],
                                    [1, 0],
                                    [2, 0],
                                    [3, 0]])
            elif formation == "square":  # Square
                formation = np.array([[0, 1],
                                    [1, 1],
                                    [1, 0],
                                    [0, 0]])
            else:
                raise ValueError("Invalid formation type. Please select 1, 2, or 3.")
        elif self.num_of_robots == 3:
            if formation == "triangle":  # Triangle
                formation = np.array([[0, 0],
                                    [1, 2],
                                    [2, 0]])
            elif formation == "line_h":  # Line
                formation = np.array([[0, 0],
                                    [1, 0],
                                    [2, 0]])
            elif formation == "line_v":  # Square
                formation = np.array([[0, 1],
                                    [0, 1],
                                    [0, 2]])


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
    def get_leader(self):
        min_distance = float('inf')
        closest_key = None

        for key, value in self.X.items():
            
            distance = np.linalg.norm(value[:2] - self.goal)

            if distance < min_distance:
                min_distance = distance
                closest_key = key

        return closest_key
    
    def calculate_goal_vel(self):
        drone_0 = self.X[self.leader]
        vector = self.goal - drone_0[:2]
        mig_acc = vector / np.linalg.norm(vector)

        mig_acc = mig_acc * np.linalg.norm(vector)**2

        # Clip the acceleration magnitude to 1 m/s²
        max_acc = 1 # Maximum allowed acceleration in m/s²
        acc_norm = np.linalg.norm(mig_acc)
        
        if acc_norm > max_acc:
            mig_acc = (mig_acc / acc_norm) * max_acc

        self.get_logger().error(f"Mig: {mig_acc * self.dt}")

        return mig_acc * self.dt


    def update_vel(self):
        for drone, velocity in self.vel.items():
            twist_vel = Twist()
            twist_vel.linear.x = float(velocity[0])
            twist_vel.linear.y = float(velocity[1])

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

        if self.goal is not None:
            self.leader = self.get_leader()
            self.vel[self.leader] = self.calculate_goal_vel()

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
    # Get topology and formation from command line arguments
    node = Node("consensus_formation_controller")  # Create a temporary node for argument parsing
    topology = node.declare_parameter('topology', 1).value
    formation = node.declare_parameter('formation', 'square').value
    node.destroy_node()  # Destroy the temporary node
    # Instantiate the Consensus Controller
    controller = ConsensusFormationController()
    # controller.launch_drones()
    controller.set_topology(topology)
    controller.set_formation(formation)

    # Keep the node alive until manually interrupted
    rclpy.spin(controller)

    # Shutdown
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
