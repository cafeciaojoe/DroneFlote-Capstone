#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import json


def callback(data):
    points = json.loads(data.data)
    # points = point.replace("u\'", "\'").replace("[", "").replace("]", "")
    readings = points[0]
    for point in readings:
        rospy.loginfo(rospy.get_caller_id() + "nose: %s", readings[point])


def listener():
    rospy.init_node('demo_listener', anonymous=True)
    rospy.Subscriber("pose_data", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
