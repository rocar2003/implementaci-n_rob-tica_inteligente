import rclpy  
from rclpy.node import Node  
from geometry_msgs.msg import Twist  
from sensor_msgs.msg import LaserScan  
import numpy as np  

class LaserScanSub(Node):  

    def __init__(self):  
        super().__init__('avoid_obstacles')  
        self.sub_lidar = self.create_subscription(LaserScan, "base_scan", self.lidar_cb, 10)
        self.sub_cmd_vel_aux = self.create_subscription(Twist, "cmd_vel_aux", self.vel_cb, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  
        self.lidar = LaserScan()  

        timer_period = 0.01  

        self.d_A0 = 1.5
        self.d_stop = 0.4
        self.v_A0 = 0.3
        self.k_A0 = 0.6

        self.timer = self.create_timer(timer_period, self.timer_callback)  
        self.robot_vel = Twist()  
        self.vel = Twist()  

        self.get_logger().info("Node initialized!!!")  

    def timer_callback(self):  
        if self.lidar.ranges:  
            closest_range, closest_angle = self.get_closest_object()  
            
            if closest_range > self.d_stop:
                if closest_range < self.d_A0:
                    theta_A0 = self.get_avoid_obstacles_angle(closest_angle)
                    v = self.v_A0
                    w = self.k_A0 * theta_A0
                else:
                    v = self.vel.linear.x
                    w = self.vel.angular.z
            else: 
                v = self.vel.linear.x
                w = self.vel.angular.z

            self.robot_vel.linear.x = v
            self.robot_vel.angular.z = w
            self.pub_cmd_vel.publish(self.robot_vel)  

    def lidar_cb(self, lidar_msg):  
        self.lidar = lidar_msg 

    def vel_cb(self, vel_msg):  
        self.vel = vel_msg 

    def get_closest_object(self):  
        ranges_len = len(self.lidar.ranges)
        if ranges_len == 0:
            return float('inf'), 0.0

        new_ranges = self.lidar.ranges[int(ranges_len/4):int(3*ranges_len/4)]
        new_angle_min = -np.pi/2.0

        range_closest = min(new_ranges)
        index = new_ranges.index(range_closest)
        
        angle_closest = new_angle_min + (index * self.lidar.angle_increment)
        angle_closest = np.arctan2(np.sin(angle_closest), np.cos(angle_closest))

        return range_closest, angle_closest  

    def get_avoid_obstacles_angle(self, closest_angle):
        theta_A0 = closest_angle + np.pi
        theta_A0 = np.arctan2(np.sin(theta_A0), np.cos(theta_A0))
        return theta_A0
    
def main(args=None):  
    rclpy.init(args=args)  
    node = LaserScanSub()  
    rclpy.spin(node)  
    node.destroy_node()  
    rclpy.shutdown()  

if __name__ == '__main__':  
    main()
