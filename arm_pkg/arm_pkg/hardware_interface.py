import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from g11_moveo import BCN3D_Moveo, Moveo_IK

class HardwareInterface(Node):
    def __init__(self):
        super().__init__('controls_generator')
        self.arm = BCN3D_Moveo("")
        self.ik = Moveo_IK()
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'control',
            self.update,
            10)

    def update(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)

    hardware_interface = HardwareInterface()

    rclpy.spin(hardware_interface)

    hardware_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()