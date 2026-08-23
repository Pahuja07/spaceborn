import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

class JammerController(Node):

    def __init__(self):

        super().__init__('jammer_controller')

        self.publisher_ = self.create_publisher(
            Int32,
           '/jammer_mode',
           10
        )
        self.mode = 0
        self.timer = self.create_timer(
           5.0,
           self.next_mode
        )

    def next_mode(self):

        msg = Int32()

        msg.data = self.mode

        self.publisher_.publish(msg)

        self.get_logger().info(
            f"Mission Mode : {self.mode}"
        )

        self.mode += 1

        if self.mode > 5:
            self.mode = 0    
        
def main(args=None):

    rclpy.init(args=args)

    node = JammerController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
