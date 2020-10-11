#!/usr/bin/env python
import math

import rospy
from std_msgs.msg import String, Header
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist
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

pub = rospy.Publisher('crazyflie2/command/trajectory', MultiDOFJointTrajectory, queue_size=20)


class PoseParserNode:
    ANGLE_THRESHOLD = 15
    previous_angle = None
    POSITION_BASE = "rightShoulder"
    POSITION_OUTER = "rightWrist"
    MINIMUM_CONFIDENCE = 0.5
    high = None

    def __init__(self):
        rospy.init_node('parser_node', anonymous=True)

    @staticmethod
    def convert_to_dictionary(data):
        pose_dict = {}
        for i in range(0, len(data)):
            pose_dict[PART_MAP[i]] = {
                # "part": data[i]["part"],
                "position": (float(data[i]["position"]["x"]), float(data[i]["position"]["y"])),
                "score": float(data[i]["score"])
            }
        return pose_dict

    def callback(self, data):
        points_data = json.loads(data.data)
        keypoints = PoseParserNode.convert_to_dictionary(points_data)
        self.simulation_pose_demo(keypoints)

    def simulation_pose_demo(self, keypoints):
        if keypoints["nose"]["score"] > self.MINIMUM_CONFIDENCE and \
                keypoints["rightKnee"]["score"] > self.MINIMUM_CONFIDENCE and \
                keypoints["leftKnee"]["score"] > self.MINIMUM_CONFIDENCE and \
                keypoints["rightWrist"]["score"] > self.MINIMUM_CONFIDENCE:
            midpoint_y = ((keypoints["leftKnee"]["position"][1] + keypoints["rightKnee"]["position"][1]) / 2) - \
                         keypoints["nose"]["position"][1]
            # Y axis is inverted, assign bool accordingly, Lower is larger, Higher is smaller
            above = False if keypoints["rightWrist"]["position"][1] > midpoint_y else True
            # rospy.loginfo(
            #     "High = %s, midpoint = %s, wrist_y = %s" % (above, midpoint_y, keypoints["rightWrist"]["position"][1]))
            if self.high is None:
                self.high = not above
            if self.high != above:
                rospy.loginfo("Switch hover mode")
                rospy.loginfo("High = %s, midpoint = %s, wrist_y = %s" % (
                    above, midpoint_y, keypoints["rightWrist"]["position"][1]))
                if above:
                    self.publisher(0, 0, 3)
                else:
                    self.publisher(0, 0, 1)
            self.high = above
        else:
            pass
            # rospy.loginfo("nose = %s, %s, knee1 = %s, %s, knee2 = %s, %s, wrist = %s, %s" % (keypoints["nose"]["score"],
            #                                                                                  keypoints["nose"][
            #                                                                                      "position"],
            #                                                                                  keypoints["rightKnee"][
            #                                                                                      "score"],
            #                                                                                  keypoints["rightKnee"][
            #                                                                                      "position"],
            #                                                                                  keypoints["leftKnee"][
            #                                                                                      "score"],
            #                                                                                  keypoints["leftKnee"][
            #                                                                                      "position"],
            #                                                                                  keypoints["rightWrist"][
            #                                                                                      "score"],
            #                                                                                  keypoints["rightWrist"][
            #                                                                                      "position"]))

    def positional_demo(self, keypoints):
        if keypoints[self.POSITION_BASE]["score"] > self.MINIMUM_CONFIDENCE and \
                keypoints[self.POSITION_OUTER]["score"] > self.MINIMUM_CONFIDENCE:

            angle_horizontal = PoseParserNode.get_angle(keypoints[self.POSITION_BASE]["position"],
                                                        keypoints[self.POSITION_OUTER]["position"])
            current_angle = True if angle_horizontal > self.ANGLE_THRESHOLD else False
            if self.previous_angle is None:
                self.previous_angle = not current_angle
            if self.previous_angle != current_angle:
                rospy.loginfo(rospy.get_caller_id() + " Over %s degrees?: %s | Confidence = %s:%s | Angle = %s",
                              self.ANGLE_THRESHOLD, current_angle, keypoints[self.POSITION_BASE]["score"],
                              keypoints[self.POSITION_OUTER]["score"], angle_horizontal)
            self.previous_angle = current_angle

    def listener(self):
        rospy.Subscriber("pose_data", String, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def publisher(self, x, y, z):
        trajectory = MultiDOFJointTrajectory()
        trajectory.header = Header()
        trajectory.header.stamp = rospy.Time()
        trajectory.header.frame_id = ''
        trajectory.joint_names = ["base_link"]
        point = MultiDOFJointTrajectoryPoint([self.create_point(x, y, z)], [self.create_velocity(0, 0, 0)],
                                             [self.create_acceleration(0, 0, 0)], rospy.Time(3))
        trajectory.points.append(point)
        pub.publish(trajectory)

    def create_point(self, x, y, z):
        transformation = Transform()
        transformation.translation.x = x
        transformation.translation.y = y
        transformation.translation.z = z
        transformation.rotation.x = 0
        transformation.rotation.y = 0
        transformation.rotation.z = 0
        transformation.rotation.w = 0
        return transformation

    def create_velocity(self, x, y, z):
        velocity = Twist()
        velocity.linear.x = x
        velocity.linear.y = y
        velocity.linear.z = z
        velocity.angular.x = 0
        velocity.angular.y = 0
        velocity.angular.z = 0
        return velocity

    def create_acceleration(self, x, y, z):
        acceleration = Twist()
        acceleration.linear.x = x
        acceleration.linear.y = y
        acceleration.linear.z = z
        acceleration.angular.x = 0
        acceleration.angular.y = 0
        acceleration.angular.z = 0
        return acceleration

    def test_publish(self):
        while True:
            rospy.sleep(10)
            self.publisher(0, 0, 1)
            rospy.sleep(10)
            self.publisher(0, 0, 1)

    @staticmethod
    def get_angle(base_point, outer_point):
        if outer_point[0] > base_point[0]:
            x = outer_point[0] - base_point[0]
        else:
            x = base_point[0] - outer_point[0]
        if outer_point[1] > base_point[1]:
            y = outer_point[1] - base_point[1]
        else:
            y = base_point[1] - outer_point[1]

        return math.degrees(math.atan(y / x))


if __name__ == '__main__':
    node = PoseParserNode()
    node.listener()
    # node.test_publish()
