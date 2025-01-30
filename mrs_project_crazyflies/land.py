
import rclpy
import rclpy.duration
from rclpy.node import Node
from crazyflie_interfaces.srv import Land, NotifySetpointsStop

class Land(Node):
    def __init__(self):
        self.notify_clients = [self.create_client(NotifySetpointsStop,f"/cf_{i+1}/notify_setpoints_stop")for i in range(3)]
        self.land_clients = [self.create_client(Land,f"/cf_{i+1}/land")for i in range(3)]

    def land_drones(self):
        req = NotifySetpointsStop.Request()
        for client in self.notify_clients:
            client.call_async(req)

        req = Land.Request()
        for client in self.land_clients:
            req.height = 0
            req.duration = rclpy.duration.Duration(seconds=2.0).to_msg()
            client.call_async(req)




        

def main(args=None):
    rclpy.init(args=args)


    # Instantiate the Boid class
    land_node = Land()
    land_node.land_drones()
    # print(f"Node {robot_id} / {num_of_robots} started correctly!")

    # Keep the node alive until manually interrupted
    rclpy.spin(land_node)

    # Shutdown
    land_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 