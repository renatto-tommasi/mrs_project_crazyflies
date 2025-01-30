#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose, Twist, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry, OccupancyGrid
from dataclasses import dataclass
import math
from mrs_project_crazyflies.svc import StateValidityChecker

from crazyflie_interfaces.srv import Land, NotifySetpointsStop
import signal
import time


@dataclass
class BoidState:
    pos: np.array
    vel: np.array
    dist: float

# Predefined colors for velocity types
BEHAVIOR_COLORS = [
    (1.0, 0.0, 0.0, 1.0),  # Red Cohesion
    (0.0, 1.0, 0.0, 1.0),  # Green Separation
    (0.0, 0.0, 1.0, 1.0),  # Blue Alignment
    (1.0, 1.0, 0.0, 1.0),  # Yellow Migration
    (1.0, 0.0, 1.0, 1.0),  # Magenta
    (0.0, 1.0, 1.0, 1.0),  # Cyan
]


class BoidController(Node):
    def __init__(self, k_all=0.2, k_sep=0.45, k_coh=0.7, k_mig=0.7, k_obs=2):
        super().__init__('reynolds_controller')

        # Declare parameters with default values
        self.declare_parameter('num_of_robots', 4)  # Default to 4 robots
        self.declare_parameter('robot_id', 0)  # Default to robot_id 0

        # Get parameters from the launch file
        self.num_of_robots = self.get_parameter('num_of_robots').value
        id = self.get_parameter('robot_id').value

        self.drone = f"cf_{id}"

        # Tuning parameters
        self.k_sep = k_sep
        self.k_all = k_all
        self.k_coh = k_coh
        self.k_mig = k_mig
        self.k_obs = k_obs
        self.vp = 2
        self.timeout = 1

        self.neighbors = {}
        self.p_wf = None  # Boid Position in World Frame x, y, theta
        self.orientation = 0.0
        self.vel = np.array([0.0, 0.0]).reshape(2, 1)
        self.migration_target = None
        self.max_speed = 0.5

        self.consensus_vel = None

        self.get_logger().info(f"Node {self.drone} started correctly!")
        self.state_checker = None


        # Publishers
        self.vel_pub = self.create_publisher(Twist, f"/{self.drone}/cmd_vel", 10)
        self.marker_pub = self.create_publisher(MarkerArray, f"/{self.drone}/velocity_marker", 10)
        

        # SUBSCRIBERS
        self.neighbors_subs = [self.create_subscription(Odometry, f"/cf_{i+1}/odom",self.get_odom_callback, 10) for i in range(self.num_of_robots)]
        self.goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self.migratory_urge, 10)
        self.consensus_sub = self.create_subscription(Twist, f"/{self.drone}/consensus_vel", self.consensus_control, 10)
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.save_map,
            10)  # QoS profile

        self.dt = 0.1

        # TIMERS
        self.delay = 5      # seconds
        self.delay_counter = 0
        self.timer = self.create_timer(self.dt, self.reynolds)
        self.chek_consensus = self.create_timer(0.5, self.check_topic_activity)  # Check every 1 second

        # Land Services
        self.notify_clients = [self.create_client(NotifySetpointsStop,f"/cf_{i+1}/notify_setpoints_stop")for i in range(self.num_of_robots)]
        self.land_clients = [self.create_client(Land,f"/cf_{i+1}/land")for i in range(self.num_of_robots)]
        # Create a signal handler for SIGINT (Ctrl+C)
        signal.signal(signal.SIGINT, self.signal_handler)
        self.land = False

    
    def save_map(self, map:OccupancyGrid):
        self.get_logger().info(f"Node {self.drone} GOT MAP!!!")

        self.state_checker = StateValidityChecker(map)

    def consensus_control (self, msg:Twist):
        self.vp = 0.3
        self.k_sep = 0.2
        self.k_all = 0
        self.k_coh = 0

        self.consensus_vel = np.array([msg.linear.x, msg.linear.y]).reshape((2,1))
        self.destroy_subscription(self.goal_sub)
        self.last_message_time = self.get_clock().now()


    def check_topic_activity(self):
        if self.consensus_vel is None:
            return
        
        time_diff = self.get_clock().now() - self.last_message_time
        if time_diff.nanoseconds / 1e9 > self.timeout:
            self.vp = 20
            self.k_sep = 0.45
            self.k_all = 0.2
            self.k_coh = 0.7
    
            self.goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self.migratory_urge, 10)
            self.consensus_vel = None
            


    def migratory_urge(self, goal: PoseStamped):
        x_goal_wf = goal.pose.position.x
        y_goal_wf = goal.pose.position.y
        self.migration_target = np.array([x_goal_wf, y_goal_wf]).reshape(2, 1)

    def get_odom_callback(self, msg:Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vl_x = msg.twist.twist.linear.x
        vl_y = msg.twist.twist.linear.y
        frame = msg.child_frame_id

        if msg.pose.pose.position.z < 0.1:
            self.launch_drone()

        if frame == self.drone:
            self.vel = np.array([vl_x, vl_y]).reshape(2, 1)
            self.orientation = self.get_orientation(self.vel)
            self.p_wf = np.array([x, y, self.orientation]).reshape(3, 1)
            return
        
        if self.p_wf is None:
            return
        
        vel = np.array([vl_x, vl_y]).reshape(2, 1)
        n_pos_rf = np.array([0.0, 0.0, 0.0]).reshape(3, 1)

        n_pos_wf = np.array([x, y]).reshape(2, 1)
        n_pos_rf[:2] = n_pos_wf - self.p_wf[:2]
        n_pos_rf[2] = np.arctan2(n_pos_rf[1], n_pos_rf[0]) - self.orientation
        n_dist = np.linalg.norm(n_pos_rf[:2])

        if self.is_visible(n_dist, n_pos_rf[2], self.vp):
                
                self.neighbors[frame] = BoidState(pos=n_pos_rf, vel=vel, dist=n_dist)
        elif frame in self.neighbors:
                del self.neighbors[frame]

        self.get_logger().info(f"Boid {self.drone} has {(self.neighbors)} Neighbors!")


    def is_visible(self, n_dist, angle, max_distance=1, field_of_view=np.pi):
        return n_dist <= max_distance and -field_of_view <= angle <= field_of_view

    def get_orientation(self, vel):
        if np.linalg.norm(vel) > 0:
            self.orientation = np.arctan2(vel[1], vel[0])[0]
        return self.orientation

    def get_neighbors_vel(self, n_odom: Odometry):
        return np.array([n_odom.twist.twist.linear.x, n_odom.twist.twist.linear.y]).reshape(2, 1)

    def reynolds(self):
        if self.delay_counter < self.delay/self.dt:
            self.delay_counter += 1
            return

        allignment_acc = self.get_allignment_acc() if len(self.neighbors) > 0 else np.zeros((2, 1)) 
        cohesion_acc = self.get_cohesion_acc() if len(self.neighbors) > 0 else np.zeros((2, 1)) 
        separation_acc = self.get_separation_acc() if len(self.neighbors) > 0 else np.zeros((2, 1)) 
        migration_acc = self.get_migration_acc()
        obstacle_acc = np.zeros((2, 1))

        vel = self.calculate_vel(allignment_acc, cohesion_acc, separation_acc, migration_acc, obstacle_acc)

        speed = np.linalg.norm(vel)

        if speed > self.max_speed:
            vel = vel / speed * self.max_speed

        self.publish_vel(vel)



    def get_allignment_acc(self):
        '''
        Matches the velocity of the neighbors
        '''
        allignment_vel = np.zeros((2,1))

        vel_all = np.zeros((2,1))
        
        if len(self.neighbors) > 0:
            for agent in self.neighbors.values():
                vel_all += agent.vel
            # Normalize by total weight if it's greater than zero
            allignment_vel = vel_all / len(self.neighbors)

        allignment_acc = allignment_vel - self.vel


        return allignment_acc   
    
    def get_cohesion_acc(self):
        coh_acc = np.zeros((2,1))
        center_of_mass = np.zeros((2, 1))  # Initialize center of mass to zero
        num_neighbors = len(self.neighbors)

        if num_neighbors > 0:
            # Calculate the center of mass
            for agent in self.neighbors.values():
                center_of_mass += agent.pos[:2]
            center_of_mass /= num_neighbors + 1  # Average position of neighbors

        coh_acc = center_of_mass 
        return coh_acc
    
    def get_separation_acc(self):
        sep_acc = np.zeros((2,1))
        num_neighbors = len(self.neighbors)

        if num_neighbors > 0:
            for agent in self.neighbors.values():
                direction_vector = -agent.pos[:2] / agent.dist

                sep_acc += (1/agent.dist**2) * direction_vector
            sep_acc /= len(self.neighbors)
        

        return sep_acc
    
    def get_migration_acc(self):

        if self.migration_target is None:
            return np.zeros((2,1))
        
        vector = self.migration_target - self.p_wf[:2]
        mig_acc = vector / np.linalg.norm(vector)

        

        return mig_acc * np.linalg.norm(vector)**2
    
    def get_obstacle_avoidance(self):
        if self.state_checker is None:
            # self.get_logger().error(f"State Checker is still none in {self.drone}")
            return np.zeros((2, 1))
        obs_acc = np.zeros((2, 1))             # Initialize acceleration vector
        obstacle_distance = 0.4                  # The maximum distance to check for obstacles
        resolution = self.state_checker.map_resolution
        map_data = self.state_checker.map
        map_dim = self.state_checker.map_dim
        map_obstacle_threshold = 5                  # Threshold for considering an obstacle
        grid_x, grid_y = self.state_checker.map_to_grid(self.p_wf[0], self.p_wf[0])
        radius = int(obstacle_distance / resolution) # Radius
        obstacle_count = 0
        # Iterate through the grid
        for iy in range(grid_y - radius, grid_y + radius + 1):
            for ix in range(grid_x - radius, grid_x + radius + 1):
                if 0 <= ix < map_dim[1] and 0 <= iy < map_dim[0]:                      # Check if the grid point is within the map bounds
                    if map_data[iy, ix] >= map_obstacle_threshold:  
                        direction_x = grid_x - ix                             
                        direction_y = grid_y - iy                           
                        distance = math.sqrt(direction_x**2 + direction_y**2)
                        if distance < 1e-3:
                            distance =  1e-3 # Compute direction from robot to the obstacle
                        obs_acc[0] += (direction_x / distance) / distance
                        obs_acc[1] += (direction_y / distance) / distance
                        obstacle_count += 1
        if obstacle_count > 0:
            obs_acc /= obstacle_count

        return obs_acc

    
    def calculate_vel(self, allignment_acc, cohesion_acc, separation_acc, migration_acc, obstacle_acc):

        if self.consensus_vel is not None:
            self.get_logger().info(f"Calculating Consensus Velocities")

            force_acc = 0.6 * separation_acc 
            agent_vel = force_acc*self.dt + self.consensus_vel*0.6


            return agent_vel
        
        self.get_logger().info(f"Calculating Reynolds Velocities")

        force_acc = self.k_coh * cohesion_acc + self.k_sep * separation_acc + self.k_all * allignment_acc + self.k_mig * migration_acc + self.k_obs * obstacle_acc

        agent_vel = force_acc * self.dt

        velocities = [self.k_coh * cohesion_acc,
                      self.k_sep * separation_acc,
                      self.k_all * allignment_acc,
                      self.k_mig * migration_acc,
                      self.k_obs * obstacle_acc,
                      agent_vel 

                      ]

        # self.publish_velocity_arrows(self.p_wf , velocities)


        return agent_vel
    
    def publish_vel(self, vel):
        if self.land:
            return

        vel_msg = Twist()
        vel_msg.linear.x = float(vel[0])
        vel_msg.linear.y = float(vel[1])

        self.vel_pub.publish(vel_msg)

    def publish_velocity_arrows(self, origin, velocities):
        """
        Publishes an RViz marker array representing velocities as arrows.

        Args:
            origin (tuple): The (x, y, z) coordinates of the arrows' base.
            velocities (list of numpy arrays): A list of velocity vectors (np.array([vx, vy])).
            velocity_types (list of str): The corresponding behavior types for each velocity.
            marker_pub (rospy.Publisher): The ROS publisher for the marker array.
        """
        marker_array = MarkerArray()

        for i, velocity in enumerate(velocities):
            # Create a unique marker for each velocity
            marker = Marker()
            marker.header.frame_id = f"{self.drone}/base_link"  # Replace with your frame ID
            # marker.header.stamp = rospy.Time.now()
            marker.ns = "velocity_arrows"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            # Define the start and end points of the arrow
            start_point = Point()
            start_point.x = 0.0
            start_point.y = 0.0
            start_point.z = 0.0
            end_point = Point(velocity[0], velocity[1], 0)
            marker.points.append(start_point)
            marker.points.append(end_point)

            # Set arrow properties
            marker.scale.x = 0.05  # Shaft diameter
            marker.scale.y = 0.1  # Head diameter
            marker.scale.z = 0.1  # Head length

            # Assign a predefined color based on the velocity type
            
            r, g, b, a = BEHAVIOR_COLORS[i]

            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = a

            # Add the marker to the array
            marker_array.markers.append(marker)

        # Publish the marker array
        self.marker_pub.publish(marker_array)

    def launch_drone(self):
        """
        Description: This function publishes the velocity to each boid
        Input: 
            V: nd.array (n,2) -> velocities vector

        """
        linear_z = float(0.5)


        # Create the Twist message
        twist_msg = Twist()
        twist_msg.linear.z = linear_z
        
        self.get_logger().info(f"Launching {self.drone}")

        
        self.vel_pub.publish(twist_msg)

    def land_drones(self):
        self.land = True
        time.sleep(1.0) 
        req = NotifySetpointsStop.Request()
        for client in self.notify_clients:
            client.call_async(req)

        req = Land.Request()
        for client in self.land_clients:
            req.height = 0.0
            req.duration = rclpy.duration.Duration(seconds=2.0).to_msg()
            client.call_async(req)
        
        time.sleep(2.0)   

    def signal_handler(self, signum, frame):
        # Get the logger and log a message
        self.get_logger().info('Received Ctrl+C, landing drones...')
        self.destroy_publisher(self.vel_pub)
        # Call the land_drones method
        self.land_drones()
        time.sleep(5.0) 
        # Continue with the shutdown process
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)


    # Instantiate the Boid class
    boid = BoidController()
    boid.launch_drone()

    # Keep the node alive until manually interrupted
    rclpy.spin(boid)

if __name__ == '__main__':
    main() 
