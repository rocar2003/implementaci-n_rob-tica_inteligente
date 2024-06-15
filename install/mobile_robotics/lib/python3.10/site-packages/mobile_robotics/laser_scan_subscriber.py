import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LaserScanSub(Node): 
    def __init__(self): 
        super().__init__('laser_scan_subscriber')

        self.sub = self.create_subscription(LaserScan, "base_scan", self.lidar_cb, 10) 
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.lidar = LaserScan() # Data from lidar will be stored here. 
        self.vel = Twist() 
        timer_period = 1.0 
        self.timer = self.create_timer(timer_period, self.timer_callback) 
        self.get_logger().info("Node initialized!!!") 

    def timer_callback(self): 
        #### ADD YOUR CODE ### 
        pass

    def lidar_cb(self, lidar_msg): 
        ## This function receives the ROS LaserScan message 
        self.lidar =  lidar_msg  

def main(args=None): 
    rclpy.init(args=args) 
    m_p=LaserScanSub() 
    rclpy.spin(m_p) 
    m_p.destroy_node() 
    rclpy.shutdown() 


if __name__ == '__main__': 
    main() 