#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import json


def callback(data):
    for part in data.data:

        for point in part:

            rospy.loginfo(rospy.get_caller_id() + "eyes: %s", str(point))
            # print(point)


def listener():
    rospy.init_node('demo_listener', anonymous=True)
    rospy.Subscriber("pose_data", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
