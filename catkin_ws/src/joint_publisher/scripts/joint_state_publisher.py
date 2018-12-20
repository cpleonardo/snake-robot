#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Joy

def joyCallback(data):
    print(data)

# Intializes everything
def startNode():
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, joyCallback)
    # starts the node
    rospy.init_node('joint_state_publisher')
    rospy.spin()

if __name__ == '__main__':
    startNode()
