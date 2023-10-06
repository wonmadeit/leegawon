#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32


rospy.init_node("wego_node")
# 노드 역할 설정
pub = rospy.Subscriber("/counter", Int32, queue_size=1)
num = 0
while True:
    num += 1
    pub.publish(num)
        
