#!/usr/bin/env python
from ast import arg
import rclpy
from rclpy.node import Node
from global_interface.msg import ObjHP as ObjHPMsg
from global_interface.msg import CarPos as CarPosMsg
from global_interface.msg import GameInfo as GameInfoMsg
from global_interface.msg import Serial as SerialMsg
from global_interface.msg import Point2f
from global_interface.msg import ModeSet
import random


class Lier(Node):

    def __init__(self):
        super().__init__('fake_msg_publisher')
        self.publisher_CarHP = self.create_publisher(
            ObjHPMsg, '/obj_hp', 10)
        self.publisher_CarPosMsg = self.create_publisher(
            CarPosMsg, '/car_pos', 10)
        self.publisher_GameInfoMsg = self.create_publisher(
            GameInfoMsg, '/game_info', 10)
        self.publisher_SerialMsg = self.create_publisher(
            SerialMsg, '/serial_msg', 10)
        self.publisher_ModeSet = self.create_publisher(
            ModeSet, '/mode_set', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.carPos_msg = CarPosMsg()
        self.w = 12.
        self.h = 8.
        for i in range(-1, 11):
            temp_pos2 = Point2f()
            temp_pos2.x = 6.
            temp_pos2.y = 4.
            self.carPos_msg.pos[i] = temp_pos2

    def timer_callback(self):
        self.make_fake()
        temp_pos = Point2f()
        # temp_pos.x = 4.6
        # temp_pos.y = 6.4
        temp_pos.x = 3.82
        temp_pos.y = 7.0
        modeSet_msg = ModeSet()
        modeSet_msg.mode = 1
        modeSet_msg.x = 3.
        modeSet_msg.y = 4.
        self.carPos_msg.pos[5] = temp_pos
        self.publisher_CarHP.publish(self.objHP_msg)
        self.publisher_CarPosMsg.publish(self.carPos_msg)
        self.publisher_GameInfoMsg.publish(self.gameInfo_msg)
        self.publisher_SerialMsg.publish(self.serial_msg)
        self.publisher_ModeSet.publish(modeSet_msg)
        self.get_logger().info("Publish Fake Msgs")

    def make_fake(self):
        self.objHP_msg = ObjHPMsg()
        self.objHP_msg.header.stamp = self.get_clock().now().to_msg()
        self.objHP_msg.hp[5] = 500
        self.objHP_msg.hp[6] = 600
        self.objHP_msg.hp[7] = 600
        for i in range(-1, 11):
            temp_pos2 = Point2f()
            aim_x = random.uniform(self.carPos_msg.pos[i].x - 0.1, self.carPos_msg.pos[i].x + 0.1)
            aim_y = random.uniform(self.carPos_msg.pos[i].y - 0.1, self.carPos_msg.pos[i].y + 0.1)
            # aim_x = 0.
            # aim_y = 0.
            if aim_x < 0.:
                aim_x = 0.
            if aim_x > self.w:
                aim_x = self.w
            if aim_y < 0.:
                aim_y = 0.
            if aim_y > self.h:
                aim_y = self.h
            temp_pos2.x = aim_x
            temp_pos2.y = aim_y
            self.carPos_msg.pos[i] = temp_pos2
        self.carPos_msg.header.stamp = self.get_clock().now().to_msg()
        self.gameInfo_msg = GameInfoMsg()
        self.gameInfo_msg.header.stamp = self.get_clock().now().to_msg()
        self.gameInfo_msg.game_stage = 4
        self.serial_msg = SerialMsg()
        self.serial_msg.header.stamp = self.get_clock().now().to_msg()


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Lier()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
