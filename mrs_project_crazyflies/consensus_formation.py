import rclpy
from rclpy.node import Node

class ConsensusFormationController(Node):
    def __init__(self):
        super().__init__('reynolds_sim')

        self.get_logger().info(f"Consensus Formation Controller started correctly!")


def main(args=None):
    rclpy.init(args=args)
    


    # Instantiate the Consensus Controller
    controller = ConsensusFormationController()
    # print(f"Node {robot_id} / {num_of_robots} started correctly!")

    # Keep the node alive until manually interrupted
    rclpy.spin(controller)

    # Shutdown
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
