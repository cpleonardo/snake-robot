#! /usr/bin/env python
import rospy
from std_msgs.msg import Float64, Int16
import json
from snake_msgs.msg import Module, ArrayModule


class SnakeJointStatePublisher():

    def __init__(self):
        self.joints_states = ArrayModule()
        self.MIN_PITCH = 0
        self.MAX_PITCH = 90
        self.MIN_YAW = -90
        self.MAX_YAW = 90

    def moduleCallback(self, module):
        for data in self.joints_states.ArrayModule:
            data.speed = module.speed
        pitch = self.joints_states.ArrayModule[module.selected - 1].pitch
        pitch += module.pitch
        if pitch > self.MAX_PITCH:
            pitch = self.MAX_PITCH
        elif pitch < self.MIN_PITCH:
            pitch = self.MIN_PITCH
        self.joints_states.ArrayModule[module.selected - 1].pitch = pitch
        yaw = self.joints_states.ArrayModule[module.selected - 1].yaw
        yaw += module.yaw
        if yaw > self.MAX_YAW:
            yaw = self.MAX_YAW
        elif yaw < self.MIN_YAW:
            yaw = self.MIN_YAW
        self.joints_states.ArrayModule[module.selected - 1].yaw = yaw
        print(json.dumps({
            "speed": self.joints_states.ArrayModule[module.selected - 1].speed,
            "pitch": self.joints_states.ArrayModule[module.selected - 1].pitch,
            "yaw": self.joints_states.ArrayModule[module.selected - 1].yaw,
            "active_module": module.selected,
        }, indent=4))

    def start_node(self):
        rospy.Subscriber("snake_joint_inc", Module, self.moduleCallback)
        joint_state_pub = rospy.Publisher('snake_joints_states',
                                          ArrayModule,
                                          queue_size=10)
        rospy.init_node('snake_joint_state_publisher', anonymous=True)
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            joint_state_pub.publish(self.joints_states)
            rate.sleep()

if __name__ == '__main__':
    try:
        joint_state = SnakeJointStatePublisher()
        joint_state.start_node()
    except rospy.ROSInterruptException:
        pass
