import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')

        # **Publicador del setpoint**
        self.publisher_ = self.create_publisher(Float32, '/set_point', 10)

        # **Parámetros de la señal**
        self.signal_type = "step"
        self.set_point = -10 # Valor de referencia

        self.amplitude = 15.0  # Amplitud de la señal representa radianes por segundos
        self.frequency = 0.05  # Frecuencia en Hz (solo para seno y cuadrada)

        # **Timer para publicar cada 0.1s**
        self.timer = self.create_timer(0.1, self.publish_signal)
        self.time = 0.0  # Contador de tiempo
        self.get_logger().info(f'Generando señal {self.signal_type} con amplitud {self.amplitude} y frecuencia {self.frequency} Hz')

    def publish_signal(self):
        msg = Float32()

        # **Generación de señales**
        if self.signal_type == 'step':
            msg.data = float(self.set_point)
        elif self.signal_type == 'sine':
            msg.data = self.amplitude * np.sin(2 * np.pi * self.frequency * self.time)
        elif self.signal_type == 'square':
            msg.data = float(self.set_point) if np.sin(2 * np.pi * self.frequency * self.time) >= 0 else -float(self.set_point)

        # **Publicar mensaje**
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Setpoint: {msg.data}')
        
        self.time += 0.1  # Avanzar el tiempo

def main(args=None):
    rclpy.init(args=args)
    node = InputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
