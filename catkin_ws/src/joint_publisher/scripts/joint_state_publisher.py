#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Int16
import json

speed = 0.0
yaw = 0.0
pitch = 0.0
active_module = 1


def joyCallback(data):
    global speed
    global active_module
    right_trigger_value = data.axes[5]
    left_button = data.buttons[4]
    rigth_button = data.buttons[5]
    if rigth_button or left_button:
        if rigth_button and active_module != 7:
            active_module += 1
        if left_button and active_module != 1:
            active_module -= 1
    if right_trigger_value != 1.0:
        speed = (1 - right_trigger_value) / 2 * 255
    print(json.dumps({
        "speed": speed,
        "pitch": pitch,
        "yaw": yaw,
        "active_module": active_module
    }, indent=4))


def jointStatePublisher():
    rospy.Subscriber("joy", Joy, joyCallback)
    speed_pub = rospy.Publisher('speed', Float64, queue_size=10)
    pitch_pub = rospy.Publisher('pitch', Float64, queue_size=10)
    yaw_pub = rospy.Publisher('yaw', Float64, queue_size=10)
    module_pub = rospy.Publisher('active_module', Int16, queue_size=10)
    rospy.init_node('joint_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        # rospy.loginfo(hello_str)
        speed_pub.publish(speed)
        pitch_pub.publish(pitch)
        yaw_pub.publish(yaw)
        rate.sleep()

if __name__ == '__main__':
    try:
        jointStatePublisher()
    except rospy.ROSInterruptException:
        pass
