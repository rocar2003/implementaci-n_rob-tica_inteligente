import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os
import datetime

class CVExample(Node):
    def __init__(self):
        super().__init__('save_images')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)
        self.pub = self.create_publisher(Image, 'processed_img', 10)
        self.image_received_flag = False  # Esta bandera es para asegurar que recibimos al menos una imagen
        dt = 0.5
        self.timer = self.create_timer(dt, self.timer_callback)
        self.get_logger().info('save_images.py Node started')
        print("I will save images every ", dt, "seconds")

    def camera_callback(self, msg):
        try:
            # Seleccionamos bgr8 porque es la codificación por defecto de OpenCV
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received_flag = True
        except Exception as e:
            self.get_logger().info(f'Failed to get an image: {e}')

    def timer_callback(self):
        try:
            if self.image_received_flag:
                # Crear una copia de la imagen
                copy = self.cv_img.copy()
                # Asegurarse de que el directorio existe
                save_path = os.path.expanduser('~/Pictures/dataset/red')
                os.makedirs(save_path, exist_ok=True)
                # Crear un nombre de archivo único basado en la fecha y hora actuales
                timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S%f')
                filename = os.path.join(save_path, f'image_{timestamp}.jpg')
                cv2.imwrite(filename, copy)
                self.get_logger().info('Image saved successfully')
                print("Saving image")
        except Exception as e:
            self.get_logger().info(f'Error: I could not save the file due to {e}')
            print(f"Error: I could not save the file due to {e}")

def main(args=None):
    rclpy.init(args=args)
    cv_e = CVExample()
    rclpy.spin(cv_e)
    cv_e.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
