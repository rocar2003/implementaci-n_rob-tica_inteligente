import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class follower(Node): 
    def __init__(self): 
        super().__init__('follower')

        # SUSCIPTORES
        self.sub = self.create_subscription(LaserScan, "base_scan", self.lidar_cb, 10) 
        # PUBLICADORES
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.timer_period = 1.0 
        self.timer = self.create_timer(self.timer_period, self.timer_callback) 
        self.get_logger().info("Node initialized!!!") 

        # VARIABLES
        self.lidar = LaserScan() 
        self.vel = Twist() 
        self.turning_d = 1.0  # Distancia para empezar a girar
        self.stop_d = 0.5     # Distancia para detenerse completamente
        self.k_w = 0.5        # Ganancia proporcional para la velocidad angular

    def timer_callback(self): 
        # Si no hay datos del LIDAR, detiene el robot
        if len(self.lidar.ranges) == 0:
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
            self.pub.publish(self.vel)
            return

        # Filtra los rangos inválidos (<0 o infinito)
        valid_ranges = [r for r in self.lidar.ranges if np.isfinite(r) and r > 0]

        # Si no hay rangos válidos, detiene el robot
        if not valid_ranges:
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
        else:
            # Encuentra la distancia mínima y el índice correspondiente
            d_closest = min(valid_ranges)
            angle_closest = self.lidar.ranges.index(d_closest)

            # Calcula el ángulo hacia el objeto más cercano
            angle_increment = self.lidar.angle_increment
            angle_to_closest = angle_increment * angle_closest - (self.lidar.angle_max - self.lidar.angle_min) / 2.0

            if d_closest < self.stop_d:
                # Si el objeto está dentro de la distancia de parada, detiene el robot
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0
            elif d_closest < self.turning_d:
                # Si el objeto está dentro de la distancia de giro, gira hacia el objeto
                self.vel.linear.x = 0.0
                self.vel.angular.z = self.k_w * angle_to_closest
            else:
                # Si el objeto está más lejos que la distancia de giro, avanza hacia el objeto
                self.vel.linear.x = 0.5
                self.vel.angular.z = self.k_w * angle_to_closest

        self.pub.publish(self.vel) 

    def lidar_cb(self, lidar_msg): 
        self.lidar = lidar_msg  

def main(args=None): 
    rclpy.init(args=args) 
    m_p = follower() 
    rclpy.spin(m_p) 
    m_p.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__': 
    main()
