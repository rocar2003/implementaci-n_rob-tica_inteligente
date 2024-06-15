import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

class My_Publisher(Node):
    def __init__(self):
        super().__init__('signal_generator_node')
        ###### INICIALIZAR PUBLICADORES
        self.publisher = self.create_publisher(Float32, 'signal', 10)
        self.publisher2 = self.create_publisher(Float32, 'time', 10)
        ###### CREAR TIMER DE PUBLICADORES
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        ###### MENSAJE PARA INICIAR
        self.get_logger().info('Signal node succesfully initialized!!!')
        ###### VARIABLES QUE VAYAS A USAR
        self.msg_float = Float32()
        self.aux = Float32()
        self.increment = 0.1

    def timer_callback(self):
        self.msg_float.data = np.sin(self.aux.data)
        self.aux.data = self.aux.data + self.increment
        ###### COMANDOS PARA PUBLICAR
        self.publisher.publish(self.msg_float)
        self.publisher2.publish(self.aux)
###### Protocolo
def main(args=None):
    rclpy.init(args=args)
    m_p = My_Publisher()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()