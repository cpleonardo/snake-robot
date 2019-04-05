#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from snake_msgs.msg import Module
import json


class JointIncPublisher():

    def __init__(self):
        self.module = Module()
        self.module.speed = 0
        self.module.yaw = 0.0
        self.module.pitch = 0.0
        self.module.selected = 1
        self.MAX_SPEED = 255
        self.YAW_MULTIPLAYER = 1
        self.PITCH_MULTIPLAYER = 1
        self.YAW_AXIS = 3
        self.PITCH_AXIS = 4
        self.SPEED_AXIS = 1

    def joyCallback(self, data):
        # right_trigger_value = data.axes[5]
        right_trigger_value = data.axes[self.SPEED_AXIS]
        left_button = data.buttons[4]
        rigth_button = data.buttons[5]
        self.module.yaw = data.axes[self.YAW_AXIS] * self.YAW_MULTIPLAYER
        self.module.pitch = data.axes[self.PITCH_AXIS] * self.PITCH_MULTIPLAYER
        if rigth_button or left_button:
            if rigth_button and self.module.selected != 7:
                self.module.selected += 1
            if left_button and self.module.selected != 1:
                self.module.selected -= 1
        # if right_trigger_value != 1.0:
        #     self.module.speed = int(
        #         (1 - right_trigger_value) / 2 * self.MAX_SPEED)
        self.module.speed = right_trigger_value * self.MAX_SPEED
        # print(json.dumps({
        #     "speed": self.module.speed,
        #     "pitch": self.module.pitch,
        #     "yaw": self.module.yaw,
        #     "active_module": self.module.selected
        # }, indent=4))

    def start_node(self):
        rospy.Subscriber("joy", Joy, self.joyCallback)
        module_publisher = rospy.Publisher('snake_joint_inc',
                                           Module,
                                           queue_size=10)
        rospy.init_node('snake_joint_inc', anonymous=True)
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            # rospy.loginfo(hello_str)
            module_publisher.publish(self.module)
            rate.sleep()

if __name__ == '__main__':
    try:
        joint_inc_pub = JointIncPublisher()
        joint_inc_pub.start_node()
    except rospy.ROSInterruptException:
        pass
