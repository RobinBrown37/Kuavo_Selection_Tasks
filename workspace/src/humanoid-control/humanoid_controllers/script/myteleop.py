#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
import threading
import cppyy

class KeyboardController:
    def __init__(self):
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist_msg = Twist()
        self.rate = rospy.Rate(150)  # 设置循环的频率为150Hz

    def on_press(self, key):
        try:
            if key.char == 'z':
                # 退出程序
                rospy.signal_shutdown("Quit")
                print("Exiting program...")
            else:
                # 根据按键设置线性速度和角速度
                if key.char == 'w':
                    self.twist_msg.linear.x = 0.4
                elif key.char == 's':
                    self.twist_msg.linear.x = -0.4
                else:
                    self.twist_msg.linear.x = 0.0

                if key.char == 'a':
                    self.twist_msg.linear.y = 0.2
                elif key.char == 'd':
                    self.twist_msg.linear.y = -0.2
                else:
                    self.twist_msg.linear.y = 0.0

                if key.char == 'q':
                    self.twist_msg.angular.z = 0.4
                elif key.char == 'e':
                    self.twist_msg.angular.z = -0.4
                else:
                    self.twist_msg.angular.z = 0.0
        except AttributeError:
            pass

    def on_release(self, key):
        # 松开按键时设置线性速度和角速度为0，并发布消息
        self.twist_msg.linear.x = 0.0
        self.twist_msg.linear.y = 0.0
        self.twist_msg.angular.z = 0.0

def ros_publish():
    while not rospy.is_shutdown():
        controller.publisher.publish(controller.twist_msg)
        controller.rate.sleep()

if __name__ == '__main__':
    # try:
    #     cppyy.load_library("/home/butterfly/Kuavo/Tryouts/workspace/devel/lib/libhumanoid_dummy.so")
    #     print("Library loaded successfully.")
    # except Exception as e:
    #     print("Failed to load library:", e)

    # print(dir(cppyy.gbl))

    # # 导入类
    # GaitKeyboardPublisher = cppyy.gbl.ocs2.humanoid.GaitKeyboardPublisher

    # # 创建类的对象
    # gaitkeyboard_pub = GaitKeyboardPublisher()

    # 初始化ROS节点
    rospy.init_node("keyboard_control")

    controller = KeyboardController()

    # 打印初始提示信息
    print("Keyboard Control Node Started")
    print("Use 'w' to move forward, 's' to move backward, 'a' to move left, 'd' to move right")
    print("Use 'q' to turn left, 'e' to turn right")
    print("Press 'z' to exit the program")
    print("Input: ")

    # 创建一个独立的线程来监听键盘按键
    thread = threading.Thread(target=ros_publish)
    thread.start()

    listener = keyboard.Listener(on_press=controller.on_press, on_release=controller.on_release)
    listener.start()

    while not rospy.is_shutdown():
        pass

    listener.stop()
    listener.join()

    # 退出程序
    thread.join()
    print("Program exited successfully")