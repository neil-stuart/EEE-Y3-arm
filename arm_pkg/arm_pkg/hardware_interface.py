import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

class HardwareInterface(Node):
    def __init__(self):
        super().__init__('cotnrols_generator')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'control',
            self.actuate_motors,
            10)
        self.subscription  # prevent unused variable warning

    def actuate_motors(self, msg):
        # Use the xyz position to determine direction arm needs to move 
        pass

def main(args=None):
    rclpy.init(args=args)

    hardware_interface = HardwareInterface()

    rclpy.spin(hardware_interface)

    hardware_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()