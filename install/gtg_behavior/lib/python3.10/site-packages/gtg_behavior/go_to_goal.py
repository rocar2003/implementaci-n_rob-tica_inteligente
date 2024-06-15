import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Pose2D 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np 
from rclpy.qos import qos_profile_sensor_data

class Odometry(Node):  
    def __init__(self):  
        super().__init__('odometry_node') 
        self.pub_pose = self.create_publisher(Pose2D, 'pose', 10)  
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)  

        self.create_subscription(Float32, "VelocityEncR",  self.wr_cb, qos_profile_sensor_data)  
        self.create_subscription(Float32, "VelocityEncL",  self.wl_cb, qos_profile_sensor_data) 

        self.declare_parameters(
            namespace='',
            parameters=[
                ('p0.x', rclpy.Parameter.Type.DOUBLE),
                ('p0.y', rclpy.Parameter.Type.DOUBLE),
                ('p1.x', rclpy.Parameter.Type.DOUBLE),
                ('p1.y', rclpy.Parameter.Type.DOUBLE)
            ]
        )

        self.r = 0.05
        self.L = 0.19
        self.wl = 0.0
        self.wr = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.robot_pose = Pose2D() 
        self.vel = Twist()
        self.Kp_linear = 0.3 
        self.Kp_angular = 2.7

        self.cont = 0

        timer_period = 0.01 
        self.create_timer(timer_period, self.main_timer_cb) 
        self.get_logger().info("Node initialized!!") 
        self.first_time = True 
        self.state = "move"

    def main_timer_cb(self):
        if self.state == "stop":
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
            self.get_logger().info("Deteniendo el robot")
            self.pub_vel.publish(self.vel)
            return

        elif self.state == "move":
            self.get_logger().info("Moviendo el robot")
            if self.first_time: 
                self.previous_time = self.get_clock().now().nanoseconds 
                self.first_time = False 
            else: 
                self.current_time = self.get_clock().now().nanoseconds 
                self.dt = float(self.current_time - self.previous_time) / (10.0**9) #Se calcula el intervalo de tiempo 
                self.previous_time = self.current_time 

                [self.v, self.w] = self.get_robot_vel(self.wr, self.wl) #Se calculan las velocidades
                #Se actualiza la posición del robot 
                self.x = self.x + self.v * np.cos(self.theta) * self.dt 
                self.y = self.y + self.v * np.sin(self.theta) * self.dt
                self.theta = self.theta + self.w * self.dt #Se actualiza la orientación del robot

                #Se actualiza la pose del robot
                self.robot_pose.x = self.x 
                self.robot_pose.y = self.y 
                self.robot_pose.theta = self.theta 

                if self.cont < 2:
                    self.xG = self.get_parameter('p' + str(self.cont) + '.x').get_parameter_value().double_value
                    self.yG = self.get_parameter('p' + str(self.cont) + '.y').get_parameter_value().double_value
                else:
                    self.state = "stop"

                # Se establece la tolerancia de 0.01 al objetivo
                if np.isclose(self.x, self.xG, atol=0.01) and np.isclose(self.y, self.yG, atol=0.1):
                    self.cont += 1

                ed = np.sqrt((self.xG - self.x)**2 + (self.yG - self.y)**2) #Se calcula la distancia al objetivo
                thetaG = np.arctan2((self.yG - self.y), (self.xG - self.x)) #Se calcula el ángulo objetivo
                e_theta = thetaG - self.theta #Se calcula el error angular
                e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) #Se ajusta e_theta para estar en rango -pi pi

                if self.state == "move":
                    V = self.Kp_linear * ed #Se calcula la velocidad lineal
                    w = self.Kp_angular * e_theta #Se calcula la velocidad angular 

                    #Se establece la velocidad angular y lineal
                    self.vel.angular.z = w
                    self.vel.linear.x = V 

                    self.pub_pose.publish(self.robot_pose) 
                    self.pub_vel.publish(self.vel) 

    def wl_cb(self, wl):  
        self.wl = wl.data  

    def wr_cb(self, wr):  
        self.wr = wr.data 

    def get_robot_vel(self, wr, wl): 
        v = self.r * (wr + wl) / 2.0 
        w = (self.r / self.L) * (wr - wl) 
        return [v, w] 
    
def main(args=None): 
    rclpy.init(args=args) 
    node = Odometry() 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main()
