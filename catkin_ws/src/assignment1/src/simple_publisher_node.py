#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def talker():
    pub = rospy.Publisher('ChaeyunJeong', Int32, queue_size=10)
    rospy.init_node('simple_publisher_node')
    rate = rospy.Rate(30) # 10hz
    while not rospy.is_shutdown():
        id_num = 20200597
        pub.publish(id_num)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass