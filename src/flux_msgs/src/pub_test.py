#!/usr/bin/env python3
from flux_msgs.msg import VelocityArray, Velocity
import rospy

rospy.init_node("just_something")

just_pub = rospy.Publisher("blah", VelocityArray, queue_size=5)

wiw = Velocity()
wiw.vx = 10.1
wiw.vy = 10.2

while(not rospy.is_shutdown()):
    wow = VelocityArray()
    wow.vels.append(wiw)
    rospy.Rate(5).sleep()
    just_pub.publish(wow)
