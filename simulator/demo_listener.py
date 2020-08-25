#!/usr/bin/env python
import math

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


class ListenerNode:

    ANGLE_THRESHOLD = 15
    previous_angle = None

    @staticmethod
    def convert_to_dictionary(data):
        pose_dict = {}
        for i in range(0, len(data)):
            pose_dict[PART_MAP[i]] = {
                # "part": data[i]["part"],
                "position": (data[i]["position"]["x"], data[i]["position"]["y"]),
                "score": data[i]["score"]
            }
        return pose_dict

    def callback(self, data):
        points_data = json.loads(data.data)
        keypoints = ListenerNode.convert_to_dictionary(points_data)
        angle_horizontal = ListenerNode.get_angle(keypoints["rightShoulder"], keypoints["rightWrist"])
        current_angle = True if angle_horizontal > self.ANGLE_THRESHOLD else False
        if self.previous_angle is None:
            self.previous_angle = not current_angle
        if self.previous_angle != current_angle:
            rospy.loginfo(rospy.get_caller_id() + " Over %s degrees?: %s", self.ANGLE_THRESHOLD, current_angle)
        self.previous_angle = current_angle

    def listener(self):
        rospy.init_node('demo_listener', anonymous=True)
        rospy.Subscriber("pose_data", String, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    @staticmethod
    def get_angle(base_point, outer_point):
        x = outer_point[0] - base_point[0]
        y = outer_point[1] - base_point[1]
        return math.degrees(math.tan(x / y))


if __name__ == '__main__':
    node = ListenerNode()
    node.listener()
