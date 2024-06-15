import rclpy  
from rclpy.node import Node  
from geometry_msgs.msg import Twist  
from sensor_msgs.msg import LaserScan  
import numpy as np  
from copy import deepcopy 

class FollowWalls(Node):  
    def __init__(self):  
        super().__init__('following_walls')  
        self.sub = self.create_subscription(LaserScan, "base_scan", self.lidar_cb, 10)  
        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  
        self.lidar = LaserScan() # Data from lidar will be stored here.  
        timer_period = 0.2  
        self.timer = self.create_timer(timer_period, self.timer_callback)  
        self.robot_vel = Twist() #Robot velocity  
        self.get_logger().info("Following Walls Node initialized!!!")  

    def timer_callback(self):  
        if self.lidar.ranges: #if we have data inside the lidar message  
            closest_range, closest_angle = self.get_closest_object()
            
            # Define the minimum obstacle distance
            min_obstacle_distance = 0.4  # in meters
            
            # Check if the closest object is too close
            if closest_range < min_obstacle_distance:
                self.robot_vel.linear.x = 0.0
                self.robot_vel.angular.z = 0.0
            else:
                if closest_range < 1.0:  # Within wall-following range
                    self.follow_wall(closest_angle)
                else:
                    self.robot_vel.linear.x = 0.2  # Move forward
                    self.robot_vel.angular.z = 0.0  # No rotation
            
            self.pub_cmd_vel.publish(self.robot_vel)  

    def lidar_cb(self, msg):  
        ## This function receives the ROS LaserScan message  
        self.lidar = msg 

    def get_closest_object(self): 
        # This function uses the self.lidar data and returns  
        # the range and angle to the closest object detected by the lidar. 
        closest_range = min(self.lidar.ranges) 
        closest_index = self.lidar.ranges.index(closest_range) 
        closest_angle = self.lidar.angle_min + closest_index * self.lidar.angle_increment 
        # wrap the angle to [-pi, pi] 
        closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle)) 
        return closest_range, closest_angle 

    def follow_wall(self, closest_angle): 
        # Adjust the robot's velocity to follow the wall counterclockwise
        ao_angle = self.get_ao_angle(closest_angle)
        
        # Simple proportional controller for wall following
        Kp = 0.55
        self.robot_vel.linear.x = 0.2  # Constant forward speed
        self.robot_vel.angular.z = Kp * ao_angle  # Angular speed to maintain distance from the wall

    def get_ao_angle(self, closest_angle): 
        # This function takes as input the angle to the closest object detected 
        # by the lidar and returns the angle of a vector that points away from it.  
        ao_angle = closest_angle + np.pi / 2  # Counterclockwise direction
        # wrap the angle to [-pi, pi] 
        ao_angle = np.arctan2(np.sin(ao_angle), np.cos(ao_angle)) 
        return ao_angle 

def main(args=None):  
    rclpy.init(args=args)  
    m_p = FollowWalls()  
    rclpy.spin(m_p)  
    m_p.destroy_node()  
    rclpy.shutdown()  

if __name__ == '__main__':  
    main()  
