import rclpy  
from rclpy.node import Node  
from geometry_msgs.msg import Twist  
from sensor_msgs.msg import LaserScan  
import numpy as np  
from copy import deepcopy 

  
class LaserScanSub(Node):  
    def __init__(self):  
        super().__init__('avoid_obstacles_2')  
        self.sub = self.create_subscription(LaserScan, "base_scan", self.lidar_cb, 10)  
        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  
        self.lidar = LaserScan() # Data from lidar will be stored here.  
        timer_period = 0.2  
        self.timer = self.create_timer(timer_period, self.timer_callback)  
        self.robot_vel = Twist() #Robot velocity  
        self.get_logger().info("Node initialized!!!")  

  

    def timer_callback(self):  
        if self.lidar.ranges: #if we have data inside the lidar message  
            croped_lidar =  self.crop_lidar(self.lidar) 
            #closest_range = min(croped_lidar.ranges)
            #closest_index = croped_lidar.ranges.index(closest_range)#
            #closeste_angle = self.get_angle(closest_index, croped_lidar.angle_min, croped_lidar.angle_increment)
            #(x, y) =  self.polar_to_cartesian(closest_range, closeste_angle)
            x = 0.0
            y = 0.0
            xT = 0.0
            yT = 0.0
            for i in range(0, len(croped_lidar.ranges)):
                r = croped_lidar.ranges[i]
                if np.isinf(r):
                    r = croped_lidar.range_max
                theta = self.get_angle(i, croped_lidar.angle_min, croped_lidar.angle_increment)
                (x ,y) = self.polar_to_cartesian(r, theta)
                xT = xT + x
                yT = yT + y
            print("xT: ", xT, "yT: ", yT)
            thetaT = np.arctan2(yT, xT)
            dT =  np.sqrt(xT*2 + yT*2)

            kv = 0.001
            kw = 0.3

            v =  kv*dT
            w = kw*thetaT

            #avoid hitting objetcs
            range_closest = min(croped_lidar.ranges)
            if range_closest < 0.4:
                v = 0.0
                

            self.robot_vel.linear.x = v
            self.robot_vel.angular.z = w     
            self.pub_cmd_vel.publish(self.robot_vel)  

  

    def lidar_cb(self, msg):  
        ## This function receives the ROS LaserScan message  
        self.lidar =  msg 


    def crop_lidar(self, lidar):  
        # Crop lidar readings behind the robot 
        new_lidar = LaserScan() 
        new_lidar = lidar 
        ranges_size = len(lidar.ranges) 
        # use just measurements from -pi/2 to pi/2 
        front_ranges =  lidar.ranges[int(ranges_size/4):int(3*ranges_size/4)] 
        new_lidar.angle_min= lidar.angle_min/2.0 
        new_lidar.angle_max = lidar.angle_max/2.0 
        # Get the closest object range from cropped data 
        new_lidar.ranges = deepcopy(front_ranges) 
        return new_lidar 

    
    def get_angle(self, idx, angle_min, angle_increment):   
        ## This function returns the angle for a given element of the object in the lidar's frame   
        angle= angle_min + idx * angle_increment   
        # Limit the angle to [-pi,pi]   
        angle = np.arctan2(np.sin(angle),np.cos(angle))   
        return angle   

  
    def polar_to_cartesian(self,r,theta):   
        ## This function converts polar coordinates to cartesian coordinates   
        x = r*np.cos(theta)
        y = r*np.sin(theta) 
        return (x,y)   

     
def main(args=None):  
    rclpy.init(args=args)  
    m_p=LaserScanSub()  
    rclpy.spin(m_p)  
    m_p.destroy_node()  
    rclpy.shutdown()  


if __name__ == '__main__':  
    main()