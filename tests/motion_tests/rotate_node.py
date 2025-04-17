#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped

rospy.init_node("simple_angular_publisher")
pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

rate = rospy.Rate(10)  # 10 Hz
msg = TwistStamped()
msg.twist.angular.x = 0.5

while not rospy.is_shutdown():
    pub.publish(msg)
    rate.sleep()
