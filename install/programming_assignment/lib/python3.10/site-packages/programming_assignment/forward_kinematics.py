from geometry_msgs.msg import TransformStamped 
import numpy as np 
import rclpy 
from rclpy.node import Node 
from tf2_ros import TransformBroadcaster 

def quaternion_from_euler(roll, pitch, yaw): 
    roll /= 2.0 
    pitch /= 2.0 
    yaw /= 2.0 
    ci = np.cos(roll) 
    si = np.sin(roll) 
    cj = np.cos(pitch) 
    sj = np.sin(pitch) 
    ck = np.cos(yaw) 
    sk = np.sin(yaw) 
    cc = ci*ck 
    cs = ci*sk 
    sc = si*ck 
    ss = si*sk 

    q = np.empty((4, )) 
    q[0] = cj*sc - sj*cs 
    q[1] = cj*ss + sj*cc 
    q[2] = cj*cs - sj*sc 
    q[3] = cj*cc + sj*ss 

    return q 

class FramePublisher(Node): 
    def __init__(self): 
        super().__init__('turtle_tf2_frame_publisher') 

        # Initialize the transform broadcaster 
        self.tf_broadcaster = TransformBroadcaster(self) 
        self.t = TransformStamped() #A ROS Transformation message 
        timer_period=0.1 
        self.timer = self.create_timer(timer_period, self.timer_callback) 
        self.get_logger().info("tf2_broadcaster node initialized!!!") 
        self.yaw=0.0 

    def timer_callback(self): 
        # Read message content and assign it to 
        # corresponding tf variables 
        self.t.header.stamp = self.get_clock().now().to_msg() 
        self.t.header.frame_id = "base_link" 
        self.t.child_frame_id = "link1" 

        #Translation 
        self.t.transform.translation.x = 1.0 
        self.t.transform.translation.y = 0.0 
        self.t.transform.translation.z = 0.0 

        #Orientation 
        roll= 0.0  
        pitch= 0.0 
        self.yaw += 0.1 #The yaw angle will increase 

        # The tf needs the orientation to be given as a quaternion 
        q=quaternion_from_euler(roll,pitch,self.yaw) 
        self.t.transform.rotation.x = q[0] 
        self.t.transform.rotation.y = q[1] 
        self.t.transform.rotation.z = q[2] 
        self.t.transform.rotation.w = q[3] 

        # Send the transformation 
        self.tf_broadcaster.sendTransform(self.t) 
 
def main(args=None): 
    rclpy.init(args=args) 
    f_p=FramePublisher() 
    rclpy.spin(f_p) 
    f_p.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main() 