#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray

rospy.init_node('forward')
pub = rospy.Publisher('/control/thrusters', Float64MultiArray, queue_size=1)
cmd = Float64MultiArray()
cmd.data = [0.01, 0, 0, 0, 0, 0, 0] 

rate = rospy.Rate(5)  
while not rospy.is_shutdown():
    pub.publish(cmd)
    rate.sleep()
