#!/usr/bin/env python
from ast import arg
import rclpy
from rclpy.node import Node
from robot_interface.msg import CarHP as CarHPMsg
from robot_interface.msg import CarPos as CarPosMsg
from robot_interface.msg import GameInfo as GameInfoMsg
from robot_interface.msg import Serial as SerialMsg

class Lier(Node):

    def __init__(self):
        super().__init__('fake_msg_publisher')
        self.publisher_CarHP = self.create_publisher(
            CarHPMsg, '/car_hp', 10)
        self.publisher_CarPosMsg = self.create_publisher(
            CarPosMsg, '/car_pos', 10)
        self.publisher_GameInfoMsg = self.create_publisher(
            GameInfoMsg, '/game_info', 10)
        self.publisher_SerialMsg = self.create_publisher(
            SerialMsg, '/serial_msg', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.make_fake()
        self.publisher_CarHP.publish(self.carHP_msg)
        self.publisher_CarPosMsg.publish(self.carPos_msg)
        self.publisher_GameInfoMsg.publish(self.gameInfo_msg)
        self.publisher_SerialMsg.publish(self.serial_msg)

    def make_fake(self):
        self.carHP_msg = CarHPMsg()
        self.carPos_msg = CarPosMsg()
        self.gameInfo_msg = GameInfoMsg()
        self.serial_msg = SerialMsg()

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Lier()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
