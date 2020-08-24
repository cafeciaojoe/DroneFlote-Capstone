#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import json

# Mapping of parts to array as per posenet keypoints.
PART_MAP = {
    0: "nose",
    1: "leftEye",
    2: "rightEye",
    3: "leftEar",
    4: "rightEar",
    5: "leftShoulder",
    6: "rightShoulder",
    7: "leftElbow",
    8: "rightElbow",
    9: "leftWrist",
    10: "rightWrist",
    11: "leftHip",
    12: "rightHip",
    13: "leftKnee",
    14: "rightKnee",
    15: "leftAnkle",
    16: "rightAnkle"
}


def convert_to_dictionary(data):
    pose_dict = {}
    for i in range(0, len(data)):
        pose_dict[PART_MAP[i]] = {
            # "part": data[i]["part"],
            "position": (data[i]["position"]["x"], data[i]["position"]["y"]),
            "score": data[i]["score"]
        }
    return pose_dict


def callback(data):
    points_data = json.loads(data.data)
    keypoints = convert_to_dictionary(points_data)
    rospy.loginfo(rospy.get_caller_id() + "nose: %s", keypoints)


def listener():
    rospy.init_node('demo_listener', anonymous=True)
    rospy.Subscriber("pose_data", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
