import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class PIController(Node):
    def __init__(self):
        super().__init__('control_node')

        # Publicador y suscriptor
        self.speed  = 0.0
        self.set_point = 0.0
        self.pwm_publisher = self.create_publisher(Float32, '/pwm', 10)
        self.speed_sub = self.create_subscription(Float32, '/motor_output', self.motor_feedback_callback, 10)
        self.set_pointsub = self.create_subscription(Float32, '/set_point', self.set_point_callback, 10)

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.publish_control)

        self.get_logger().info("Controlador PI inicializado.")

        # Motor parameters:
        self.J = 0.0001
        self.b = 0.0001
        self.L = 0.0005
        self.R = 5
        self.Ke = 0.02
        self. Kt = 0.02
        self.Tc = 0

        # Controller parameters
        self.Kp = 0.06
        self.Ki = 0.049
        self.Kd = 0.0002
        
        # Controller variables
        self.Integral = 0.0
        self.Derivative = 0.0
        self.prev_error = 0.0

    def motor_feedback_callback(self, msg):
        self.speed = msg.data
        #self.get_logger().info(f"Velocidad recibida: {msg.data}")
    
    def set_point_callback(self, msg):
        self.set_point = msg.data

    
    def publish_control(self):
        self.Integral = max(min(self.Integral, 10), -10)  # Límite de la integral
        error_speed = self.set_point - self.speed
        self.Integral += error_speed * self.timer_period
        
        # Filtro de la señal
        alpha = 0.8  # Factor de suavizado (0.0 - 1.0, mayor -> menos filtrado)
        self.Derivative = alpha * self.Derivative + (1 - alpha) * ((error_speed - self.prev_error) / self.timer_period)

        self.Derivative = (error_speed - self.prev_error) / self.timer_period
        self.prev_error = error_speed

        controlOutput = (self.Kp * error_speed + self.Ki * self.Integral + self.Kd * self.Derivative) 
       
        self.get_logger().info(f"controlOutput: {controlOutput}")
        def map_value(x, in_min, in_max, out_min, out_max):
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        
        if controlOutput > 1.0:
            controlOutput = 1.0
        elif controlOutput < -1.0:
            controlOutput = -1.0
        
        #controlOutput = map_value(controlOutput, -13.0, 13.0, -1.0, 1.0)
        self.get_logger().info(f"controlOutput: {controlOutput}")
        msg = Float32()
        msg.data = float(controlOutput)
        self.pwm_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PIController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Apagando controlador...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
