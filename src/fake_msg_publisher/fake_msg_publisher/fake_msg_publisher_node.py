from ast import arg
import rclpy
from rclpy.node import Node
from robot_interface.msg import CarHP as CarHPMsg
from robot_interface.msg import CarPos as CarPosMsg
from robot_interface.msg import GameInfo as GameInfoMsg
from robot_interface.msg import Serial as SerialMsg
class FakePub(Node):

    def __init__(self):
        super().__init__('FakeMsgPub')
        self.publisher_CarHP = self.create_publisher(CarHPMsg, '/FakeCarHP', 10)
        self.publisher_CarPosMsg = self.create_publisher(CarPosMsg,'/FakePosMsg', 10)
        self.publisher_GameInfoMsg = self.create_publisher(GameInfoMsg,'/FakeGameInfoMsg',10)
        self.publisher_SerialMsg = self.create_publisher(SerialMsg,'/FakeSerialMsg',10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = FakePub()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

