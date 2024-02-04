#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray, Float64MultiArray


# publishers
thrust_control = rospy.Publisher("/control/thrusters", Float64MultiArray, queue_size=1)
thrust_control_data = Float64MultiArray()


def on_thrust(x):
    thrust_control_data.data = [a / 100 for a in x.data]
    thrust_control.publish(thrust_control_data)


# subscribers
thrust_response = rospy.Subscriber("/rose_tvmc/thrust", Float32MultiArray, on_thrust)


if __name__ == "__main__":
    rospy.init_node("sim_translation_layer")
    rospy.loginfo("Started Simulator Translation Layer")
    rospy.spin()

