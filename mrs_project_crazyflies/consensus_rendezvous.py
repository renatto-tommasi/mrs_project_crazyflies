import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path

import numpy as np

class ConsensusRendezvousController(Node):
    def __init__(self):
        super().__init__('consensus_rendezvous')

        self.get_logger().info(f"Consensus Rendezvous Controller started correctly!")
       
        self.dt = 0.5

        self.vel = {}
        self.X = {} # Boid Locations

        self.A = None   # Adjacency Matrix

        self.num_of_robots = 3

        # PUBLISHERS
        self.vel_publishers = {f"cf_{i+1}": self.create_publisher(Twist, f"/cf_{i+1}/consensus_vel", 10) for i in range(self.num_of_robots)}

    

        # SUBSCRIBERS
        self.odom_subscribers = [self.create_subscription(Odometry, f"/cf_{i+1}/odom",self.store_odom, 10) for i in range(self.num_of_robots)]

        # # TIMERS
        self.delay = 5      # seconds
        self.delay_counter = 0
        self.timer = self.create_timer(self.dt, self.calculate_rendezvous_vel)

        self.robot_paths = {}
        self.path_publishers = {}

        # PLOT PATHS IN RVIZ

        # Create publishers for each robot's path
        self.robot_paths = {f"cf_{i+1}": Path() for i in range(self.num_of_robots)}
        self.path_publishers = {f"cf_{i+1}": self.create_publisher(Path, f"/cf_{i+1}/path", 10) for i in range(self.num_of_robots)}


    

    def update_vel(self):
        for drone, velocity in self.vel.items():
            twist_vel = Twist()
            twist_vel.linear.x = velocity[0]
            twist_vel.linear.y = velocity[1]


            self.vel_publishers[drone].publish(twist_vel)
        

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
            
        # PLOT PATH
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        # Append the pose to the robot's path
        self.robot_paths[frame].header = msg.header
        self.robot_paths[frame].poses.append(pose)

        # Publish the updated path
        self.path_publishers[frame].publish(self.robot_paths[frame])

    def set_num_of_robots(self, num_of_robots):
        self.num_of_robots == num_of_robots
        self.get_logger().info(f"Formation initialized with {self.num_of_robots} drones")


    

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
                self.get_logger().info(f"Topology set to: 1->2->3->4->1")
                
            elif topology == 2:
                self.A = np.array([[0, 1, 0, 1],
                                [1, 0, 1, 0],
                                [0, 1, 0, 1],
                                [1, 0, 1, 0]])  # Fully connected directed topology
                self.get_logger().info(f"Topology set to: 1<->2<->3<->4<->1")
                
                
            elif topology == 3:
                self.A = np.array([[0, 0, 0, 1],
                                [0, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, 0]])  
                self.get_logger().info(f"Topology set to: 1->4->3->2")
            elif topology == 4:
                self.A = np.array([[0, 0, 1, 0],
                                [1, 0, 0, 0],
                                [0, 1, 0, 1],
                                [0, 0, 0, 0]])  # II: 1->4->3->2
                self.get_logger().info(f"Topology set to: 1->3->2->1, 3->4")

            else:
                raise ValueError("Invalid choice. Please select 1, 2, 3 or 4.")
            
        elif self.num_of_robots == 3:
            if topology == 1:
                self.A = np.array([[0, 1, 0],
                                [0, 0, 1],
                                [1, 0, 0]])   # Ring topology
                self.get_logger().info(f"Topology set to: 1->2->3->1")
                
            elif topology == 2:
                self.A = np.array([[0, 1, 1],
                                [1, 0, 1],
                                [1, 1, 0]])  # Fully connected directed topology
                self.get_logger().info(f"Topology set to: 1<->2<->3<->1")
                
            else:
                raise ValueError("Invalid choice. Please select 1 or 2.")


    def calculate_rendezvous_vel(self):
        """
        Description: This function calculate the velocities to achieve the formation for each robot
        
            X: nd.array (n,2) -> locations vector
            self.A: nd.array (n,n) -> Adjacency matrix

        Update:
            self.V: nd.array (n,2) -> velocities vector
        """
        if self.delay_counter < self.delay/self.dt:
            self.delay_counter += 1
            return

        self.get_logger().info(f"Calculating Rendezvous Velocities")
        X = self._dictionary_to_matrix(self.X) # Changes the Dictionary into a Matrix compatible with the drones in order


        degree_matrix = np.diag(np.sum(self.A, axis=1))
        laplacian_matrix = degree_matrix - self.A

        self.get_logger().error(f"L: {laplacian_matrix}, X: {X[:,:2]}")


        V = -np.dot(laplacian_matrix, (X[:,:2]))*self.dt * 0.1


        

        self.vel = self._matrix_to_dictionary(V)
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
    num_of_robots = node.declare_parameter('num_of_robots', '3').value
    node.destroy_node()  # Destroy the temporary node
    
    # Instantiate the Consensus Controller
    controller = ConsensusRendezvousController()
    controller.set_num_of_robots(num_of_robots)
    controller.set_topology(topology)
    controller.launch_drones()
    

    # Keep the node alive until manually interrupted
    rclpy.spin(controller)

    # Shutdown
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
