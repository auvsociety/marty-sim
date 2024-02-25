#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Float64


# publishers
depth_pub = rospy.Publisher("/depth_data", Float64, queue_size=1)
depth_data = Float64()

thrust_control = rospy.Publisher("/control/thrusters", Float64MultiArray, queue_size=1)
thrust_control_data = Float64MultiArray()


def on_thrust(x):
    thrust_control_data.data = [a / 100 for a in x.data]
    thrust_control.publish(thrust_control_data)


def on_depth(x):
    depth_data.data = x.fluid_pressure * 10 / (9.8 * 997)
    depth_pub.publish(depth_data)


# subscribers
thrust_response = rospy.Subscriber("/rose_tvmc/thrust", Float32MultiArray, on_thrust)
depth_response = rospy.Subscriber("/emulation/pressure", FluidPressure, on_depth)


if __name__ == "__main__":
    rospy.init_node("sim_translation_layer")
    rospy.loginfo("Started Simulator Translation Layer")
    rospy.spin()

