import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')

        # Setpoint Publisher
        self.publisher_ = self.create_publisher(Float32, '/set_point', 10)

        # Node Parameters
        
        
        # Declare signal generator parameters
        self.declare_parameter('type', 'step')
        self.declare_parameter('set_point', -10.0)
        self.declare_parameter('amplitude', 15.0)
        self.declare_parameter('frequency', 0.05)

        # Retrieve parameters
        self.signal_type = self.get_parameter('type').get_parameter_value().string_value
        self.set_point = self.get_parameter('set_point').get_parameter_value().double_value
        self.amplitude = self.get_parameter('amplitude').get_parameter_value().double_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value

        # Timer to update setpoint
        self.timer = self.create_timer(0.1, self.publish_signal)
       
        self.time = 0.0  # Time Counter
        self.get_logger().info(f'Generating a: {self.signal_type} with amplitude: {self.amplitude} and frequency: {self.frequency} Hz')


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
