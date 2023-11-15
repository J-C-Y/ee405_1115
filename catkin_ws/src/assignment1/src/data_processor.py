#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def callback(data):
    only_num=data.data-20200000
    pub = rospy.Publisher('Processed_ChaeyunJeong', Int32, queue_size=10)
    pub.publish(only_num)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('data_processor')

    rospy.Subscriber('ChaeyunJeong', Int32, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()