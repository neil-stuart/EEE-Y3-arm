import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

class ControlsGenerator(Node):
    def __init__(self):
        super().__init__('controls_generator')

        # current_time = self.get_clock().now()
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'xys',
            self.calculate_controls,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Float32MultiArray, 'control', 10)
        self.i = 0

    def calculate_controls(self, msg):
        self.i += 1
        # Use the xyz position to determine direction arm needs to move 
        # Publish to control topic
        xys = msg.data

        # Define parameters


        controls = Float32MultiArray()
        controls.data = [0.0,0.0,0.0,0.0,0.0,0.0] #Lets say 5 motors, giving angles between 0 and mazx angle
        self.publisher_.publish(controls)
        
        if self.i == 60:
            self.get_logger().info(f'{str(controls.data)}')
            self.i = 0

def main(args=None):
    rclpy.init(args=args)

    controls_generator = ControlsGenerator()

    rclpy.spin(controls_generator)

    controls_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()