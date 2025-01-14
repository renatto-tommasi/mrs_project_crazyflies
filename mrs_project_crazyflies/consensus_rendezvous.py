import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Twist, PoseStamped, Point
from nav_msgs.msg import Odometry

import numpy as np

class ConsensusRendezvousController(Node):
    def __init__(self):
        super().__init__('consensus_rendezvous')

        self.get_logger().info(f"Consensus Rendezvous Controller started correctly!")
        num_of_robots = 4
        timer_period = 3  # secondstimer_period = 0.5  # seconds
        self.takeoff = 0

        self.vel = None

        # PUBLISHERS
        self.vel_publishers = [self.create_publisher(Twist, f"/cf_{i+1}/cmd_vel", 10) for i in range(num_of_robots)]

        # SUBSCRIBERS
        self.odom_subscribers = [self.create_subscription(Odometry, f"/cf_{i+1}/odom",self.store_odom, 10) for i in range(num_of_robots)]

        # # TIMERS
        # self.timer = self.create_timer(timer_period, self.update_vel)




        self.num_of_robots = 4 # Manual for now

    def update_vel(self, drone_id, vel):
        pass
        

    def store_odom(self, msg:Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        frame = msg.child_frame_id
        self.get_logger().info(f"Drone {frame} is at x:{x}, y:{y}, z{z}")
    

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

        self.get_logger().info(f"Launching Drones")
        

        # Publish the same message to all publishers
        for publisher in self.vel_publishers:
            publisher.publish(twist_msg)



def main(args=None):
    rclpy.init(args=args)
    
    # Instantiate the Consensus Controller
    controller = ConsensusRendezvousController()
    controller.launch_drones()

    # Keep the node alive until manually interrupted
    rclpy.spin(controller)

    # Shutdown
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
