""" This program publishes the radius and center of the detected ball   
    The radius will be zero if there is no detected object  
    published topics:  
        /processed_img [Image] 
    subscribed topics:  
        /robot/camera1/image_raw    [Image]  
"""  

import rclpy 
from rclpy.node import Node 
import cv2 
import numpy as np 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class CVExample(Node): 
    def __init__(self): 
        super().__init__('ros_color_tracker') 

        self.bridge = CvBridge() 

        self.sub = self.create_subscription(Image, 'robot/camera1/image_raw', self.camera_callback, 10) 
        self.pub = self.create_publisher(Image, 'processed_img', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.kv = 0.004
        self.kw = 0.002
        self.max_radius = 200 # pixels
        self.robot_vel = Twist()

        self.image_received_flag = False #This flag is to ensure we received at least one image  
        dt = 0.1 
        self.timer = self.create_timer(dt, self.timer_callback) 
        self.get_logger().info('ros_color_tracker Node started')
        

    def camera_callback(self, msg): # msg = ROS Image
        try:  
            # We select bgr8 because its the OpenCV encoding by default  
            self.cv_img= self.bridge.imgmsg_to_cv2(msg, "bgr8")  
            self.image_received_flag = True  

        except: 
            self.get_logger().info('Failed to get an image') 

    def timer_callback(self): 
        if self.image_received_flag:
            # Create a copy of the image 
            image=self.cv_img.copy() 

            size = image.shape
            xcenter = size[1]/2 # image Width / 2

            #Once we read the image we need to change the color space to HSV 
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            #Hsv limits are defined 
            #here is where you define the range of the color you´re looking for 
            #each value of the vector corresponds to the H,S & V values respectively 
            min_blue = np.array([105,20,20]) 
            max_blue = np.array([130,255,255]) 

            #This is the actual color detection  
            #Here we will create a mask that contains only the colors defined in your limits 
            #This mask has only one dimension, so its black and white } 
            mask_b = cv2.inRange(hsv, min_blue, max_blue) 

            #We use the mask with the original image to get the colored post-processed image 
            res_b = cv2.bitwise_and(image,image, mask= mask_b)

            # find contours in the mask and initialize the current (x, y) center of the ball  
            [cnts, hierarchy] = cv2.findContours(mask_b.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)  
            center = None  

            # only proceed if at least one contour was found  
            if len(cnts) > 0:  
                # find the largest contour in the mask, then use  
                # it to compute the minimum enclosing circle and  
                # centroid  
                c = max(cnts, key=cv2.contourArea)  
                ((x, y), radius) = cv2.minEnclosingCircle(c)  
                M = cv2.moments(c)  
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))  

                # only proceed if the radius meets a minimum size  
                if radius > 10: #10 pixels 
                    # Draw the circle and centroid on the cv_img. 
                    cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)  
                    cv2.circle(image, center, 5, (0, 0, 255), -1)  

                else: #If the detected object is too small 
                    radius = 0.0  #Just set the radius of the object to zero 

            else:  
                # All the values will be zero if there is no object  
                x = 0.0 
                y = 0.0 
                radius=0.0 

            print("x: ", x) 
            print("y: ", y) 
            print("radius: ", radius)

            d = xcenter - x
            r = self.max_radius - radius
            w = self.kw * d
            v = self.kv * r

            self.robot_vel.linear.x = v
            self.robot_vel.angular.z = w

            self.cmd_vel_pub.publish(self.robot_vel)
            self.pub.publish(self.bridge.cv2_to_imgmsg(image,'bgr8')) 

def main(args=None): 
    rclpy.init(args=args) 
    cv_e = CVExample() 
    rclpy.spin(cv_e) 
    cv_e.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main() 