import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class PIController(Node):
    def __init__(self):
        super().__init__('control_node')

        # Motor State Variables
        self.speed  = 0.0
        self.set_point = 0.0

        # Publishers and Subscribers
        self.pwm_publisher = self.create_publisher(Float32, '/pwm', 10)
        self.speed_sub = self.create_subscription(Float32, '/motor_output', self.motor_feedback_callback, 10)
        self.set_pointsub = self.create_subscription(Float32, '/set_point', self.set_point_callback, 10)

        # Timers
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.publish_control)

        self.get_logger().info("PID Controller Initialized.")

        # Declare motor parameters
        self.declare_parameter('J', 0.0001)
        self.declare_parameter('b', 0.0001)
        self.declare_parameter('L', 0.0005)
        self.declare_parameter('R', 5.0)
        self.declare_parameter('Ke', 0.02)
        self.declare_parameter('Kt', 0.02)
        self.declare_parameter('Tc', 0.0)

        # Declare controller parameters
        self.declare_parameter('Kp', 0.06)
        self.declare_parameter('Ki', 0.049)
        self.declare_parameter('Kd', 0.0002)

        # Get parameter values
        self.J = self.get_parameter('J').get_parameter_value().double_value
        self.b = self.get_parameter('b').get_parameter_value().double_value
        self.L = self.get_parameter('L').get_parameter_value().double_value
        self.R = self.get_parameter('R').get_parameter_value().double_value
        self.Ke = self.get_parameter('Ke').get_parameter_value().double_value
        self.Kt = self.get_parameter('Kt').get_parameter_value().double_value
        self.Tc = self.get_parameter('Tc').get_parameter_value().double_value

        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.Ki = self.get_parameter('Ki').get_parameter_value().double_value
        self.Kd = self.get_parameter('Kd').get_parameter_value().double_value
        
        # Controller variables
        self.Integral = 0.0
        self.Derivative = 0.0
        self.prev_error = 0.0

    def motor_feedback_callback(self, msg):
        self.speed = msg.data
    
    def set_point_callback(self, msg):
        self.set_point = msg.data

    
    def publish_control(self):
        self.Integral = max(min(self.Integral, 10), -10)  # Bound the integral from -10 to 10 to avoid overstimulation
        error_speed = self.set_point - self.speed
        self.Integral += error_speed * self.timer_period
        
        # Filter signal to smooth derivative term
        alpha = 0.8  # Smoothing factor (0.0 - 1.0, higher -> less filtering)
        self.Derivative = alpha * self.Derivative + (1 - alpha) * ((error_speed - self.prev_error) / self.timer_period)

        self.Derivative = (error_speed - self.prev_error) / self.timer_period
        self.prev_error = error_speed

        # Controller should be tuned to reflect speed in terms of PWM value
        controlOutput = (self.Kp * error_speed + self.Ki * self.Integral + self.Kd * self.Derivative) 
        
        # Constrain the value to a standard PWM signal
        if controlOutput > 1.0:
            controlOutput = 1.0
        elif controlOutput < -1.0:
            controlOutput = -1.0
        
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
