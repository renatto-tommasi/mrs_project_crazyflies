import rclpy
from rclpy.node import Node

class ConsensusFormationController(Node):
    def __init__(self):
        super().__init__('reynolds_sim')

        self.get_logger().info(f"Consensus Formation Controller started correctly!")

        self.num_of_robots = 4 # Manual for now

    def set_formation(self, formation):
        """
        Description: This function stablishes the formation to be obtained
        Input: 
            formation: string -> triangle, square, ... etc

        Output:
            xi: nd.array (n,2) -> positions vector
        """
        #TODO: Build the position vector for each of the agents
        pass

    def get_locations(self):
        """
        Description: This function forms a vector with the location of all the boids

        Output:
            X: nd.array (n,2) -> locations vector
        """
        #TODO: Build the locations vector with all the agents from a subsriber to the position topic
        pass

    def calculate_formation_vel(self, X, xi):
        """
        Description: This function calculate the velocities to achieve the formation for each robot
        Input: 
            X: nd.array (n,2) -> locations vector
            xi: nd.array (n,2) -> positions vector

        Output:
            V: nd.array (n,2) -> velocities vector
        """
        #TODO: Calculate the velocities for each agent using the equation in the slides
        pass

    def send_velocities(self, V):
        """
        Description: This function publishes the velocity to each boid
        Input: 
            V: nd.array (n,2) -> velocities vector

        """
        #TODO: use a publisher to access all boid velocities
        pass



def main(args=None):
    rclpy.init(args=args)
    
    # Instantiate the Consensus Controller
    controller = ConsensusFormationController()

    # Keep the node alive until manually interrupted
    rclpy.spin(controller)

    # Shutdown
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
