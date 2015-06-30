#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

pub = rospy.Publisher('update_filter_cmd', Bool, queue_size=1)
rospy.init_node('update_filter')
r = rospy.Rate(0.1) # 0.1hz
while not rospy.is_shutdown():
    #when enter key is hit, send reset message to wallfilter
    raw_input("Press Enter to reset filter. Press Ctrl-C to quit.")
    pub.publish(True)
    r.sleep()


