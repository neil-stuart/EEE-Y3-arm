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

    def actuate_motors(self, msg):
        pass

    def find_nsteps_foreach_motor(self, angles):
        pass

def main(args=None):
    rclpy.init(args=args)

    hardware_interface = HardwareInterface()

    rclpy.spin(hardware_interface)

    hardware_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()