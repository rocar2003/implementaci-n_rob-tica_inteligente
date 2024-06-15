import rclpy  
from rclpy.node import Node  
from geometry_msgs.msg import Twist  
from sensor_msgs.msg import LaserScan  
import numpy as np  

class LaserScanSub(Node):  
    def __init__(self):  
        super().__init__('avoid_obstacles_1')  
        self.sub = self.create_subscription(LaserScan, "base_scan", self.lidar_cb, 10)  
        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  
        self.lidar = LaserScan() # Data from lidar will be stored here.  
        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.timer_callback)  
        self.d_AO = 1.5 #[m] distance to start the avoid obstacles behaviour
        self.d_stop = 0.4 #[m] distnace to stop
        self.v_AO = 0.3 #[m/s] linear speed for the avoid obstacles
        self.k_AO = 0.6 # angluar speed gain
        self.robot_vel = Twist() #Robot velocity  
        self.get_logger().info("Node initialized!!!")  

    def timer_callback(self):  
        if self.lidar.ranges: #if we have data inside the lidar message  
            closest_range, closest_angle = self.get_closest_object() 
            theta_AO = self.get_avoid_obstacles_angle(closest_angle) 
            if closest_range > self.d_stop:
                if closest_range < self.d_AO:
                    v = self.v_AO
                    w = self.k_AO * theta_AO

                else: #move to the front
                    v = self.v_AO
                    w = 0.0
            else: #stop
                v = 0.0
                w = 0.0

            # Fill the speed message and publish it 
            self.robot_vel.linear.x = v
            self.robot_vel.angular.z = w 
            self.pub_cmd_vel.publish(self.robot_vel)  

    def lidar_cb(self, lidar_msg):  
        ## This function receives the ROS LaserScan message  
        self.lidar =  lidar_msg 

    def get_closest_object(self): 
        # consider only ranges to the front 
        ranges_len = len(self.lidar.ranges)
        new_ranges = self.lidar.ranges[int(ranges_len/4):int(3*ranges_len/4)]
        new_angle_min = np.pi/2.0
        closest_range = min(new_ranges)
        closest_index = new_ranges.index(closest_range)
        closest_angle = new_angle_min + closest_index * self.lidar.angle_increment
        # crop the angle from -pi to pi
        closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle))

        return closest_range, closest_angle  
    
    def get_avoid_obstacles_angle(self, closest_angle):
        theta_AO = closest_angle + np.pi
        theta_AO = np.arctan2(np.sin(theta_AO), np.cos(theta_AO))

        return theta_AO

def main(args=None):  
    rclpy.init(args=args)  
    m_p=LaserScanSub()  
    rclpy.spin(m_p)  
    m_p.destroy_node()  
    rclpy.shutdown()  

if __name__ == '__main__':  
    main()  