#!/usr/bin/env python
import math

import rospy
from std_msgs.msg import String, Header
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist
import json
import numpy as np


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
    """
    ROSpy Node for parsing data from posenet.
    """
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
        """
        Takes data recorded from posenet and converts it into a python dictionary with sensible keys.

        Args:
            data(list): A list sent from posenet.

        Returns:
            dict: A python dictionary containing pose data for each keypoint including (x, y)
                locations and confidence score.
        """
        pose_dict = {}
        for i in range(0, len(data)):
            # Use posenet part mapping to label positions from list.
            pose_dict[PART_MAP[i]] = {
                "position": (float(data[i]["position"]["x"]), float(data[i]["position"]["y"])),
                "score": float(data[i]["score"])
            }
        return pose_dict

    def callback(self, data):
        """
        Callback function for ROS pub/sub model.
        Converts message to dictionary and runs currently selected metric.

        Args:
            data: Data received from ROS subscription.

        """
        points_data = json.loads(data.data)
        keypoints = PoseParserNode.convert_to_dictionary(points_data)
        self.simulation_pose_demo(keypoints)

    def simulation_pose_demo(self, keypoints):
        """
        Midpoint metric for simulation demo. Monitors right wrist in relation to middle point between nose and knees.
        Logs locations of key-points to console and forwards message to simulator.

        Args:
            keypoints(dict): A dictionary of parsed posenet key-points.

        """
        # Check the confidence in all points required is above our threshold.

        if keypoints["nose"]["score"] > self.MINIMUM_CONFIDENCE and \
                keypoints["rightKnee"]["score"] > self.MINIMUM_CONFIDENCE and \
                keypoints["leftKnee"]["score"] > self.MINIMUM_CONFIDENCE and \
                keypoints["rightWrist"]["score"] > self.MINIMUM_CONFIDENCE:
            midpoint_y = ((keypoints["leftKnee"]["position"][1] + keypoints["rightKnee"]["position"][1]) / 2) - \
                         keypoints["nose"]["position"][1]
            # Y axis is inverted, assign bool accordingly, Lower is larger, Higher is smaller
            above = False if keypoints["rightWrist"]["position"][1] > midpoint_y else True
            rospy.loginfo(
                "High = %s, midpoint = %s, wrist_y = %s" % (above, midpoint_y, keypoints["rightWrist"]["position"][1]))
            if self.high is None:
                self.high = not above
            if self.high != above:
                rospy.loginfo("Switch hover mode")
                rospy.loginfo("High = %s, midpoint = %s, wrist_y = %s" % (
                    above, midpoint_y, keypoints["rightWrist"]["position"][1]))
                # Statically defined heights for drone locations for demo purposes.
                if above:
                    self.publisher(0, 0, 3)
                else:
                    self.publisher(0, 0, 1)
            self.high = above
        else:
            # Log keypoint data if confidence did not meet threshold.
            rospy.loginfo("nose = %s, %s, knee1 = %s, %s, knee2 = %s, %s, wrist = %s, %s" % (keypoints["nose"]["score"],
                                                                                             keypoints["nose"][
                                                                                                 "position"],
                                                                                             keypoints["rightKnee"][
                                                                                                 "score"],
                                                                                             keypoints["rightKnee"][
                                                                                                 "position"],
                                                                                             keypoints["leftKnee"][
                                                                                                 "score"],
                                                                                             keypoints["leftKnee"][
                                                                                                 "position"],
                                                                                             keypoints["rightWrist"][
                                                                                                 "score"],
                                                                                             keypoints["rightWrist"][
                                                                                                 "position"]))

    def positional_demo(self, keypoints):
        """
        Measures horizontal angle between two points and logs when the angle passes between a threshold angle.

        Args:
            keypoints(dict): Parsed posenet dictionary of key-points.

        """
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
        """
        Starts the node listening on subscribed topics until shut down.
        """
        rospy.Subscriber("pose_data", String, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def publisher(self, x, y, z):
        """
        Generates and publishes a MultiDOFJointTrajectory message from inputs to publisher topic for simulator.

        Args:
            x:
            y:
            z:
        """
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
        """
        Creates and returns a Transform object for messaging given x, y, z values.

        Returns:
            Transform: Transform from inputs.
        """
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
        """
        Creates and returns a Velocity object for messaging given x, y, z values.

        Returns:
            Twist: Velocity from inputs.
        """
        velocity = Twist()
        velocity.linear.x = x
        velocity.linear.y = y
        velocity.linear.z = z
        velocity.angular.x = 0
        velocity.angular.y = 0
        velocity.angular.z = 0
        return velocity

    def create_acceleration(self, x, y, z):
        """
        Creates and returns an Acceleration object for messaging given x, y, z values.

        Returns:
            Twist: Acceleration from inputs.
        """
        acceleration = Twist()
        acceleration.linear.x = x
        acceleration.linear.y = y
        acceleration.linear.z = z
        acceleration.angular.x = 0
        acceleration.angular.y = 0
        acceleration.angular.z = 0
        return acceleration

    def test_publish(self):
        """
        Test function for sending dummy messages over publisher topic.

        """
        while True:
            rospy.sleep(10)
            self.publisher(0, 0, 1)
            rospy.sleep(10)
            self.publisher(0, 0, 1)

    @staticmethod
    def get_angle(base_point, outer_point):
        """
        Helper method for calculating angle between two points.

        Args:
            base_point(tuple[float, float]): First point
            outer_point(tuple[float, float]): Second point

        Returns:

        """
        if outer_point[0] > base_point[0]:
            x = outer_point[0] - base_point[0]
        else:
            x = base_point[0] - outer_point[0]
        if outer_point[1] > base_point[1]:
            y = outer_point[1] - base_point[1]
        else:
            y = base_point[1] - outer_point[1]

        return math.degrees(math.atan(y / x))


class PoseMetrics:

    DEFAULT_MIDPOINT_1 = "leftWrist"
    DEFAULT_MIDPOINT_2 = "rightWrist"
    DEFAULT_HISTORY_LENGTH = 10

    def __init__(self, history_length=DEFAULT_HISTORY_LENGTH):
        self.history_length = history_length
        self.history = [{}]
        for point_name in PART_MAP:
            self.history[point_name] = []


    @staticmethod
    def midpoint(keypoints, point_1_name=DEFAULT_MIDPOINT_1, point_2_name=DEFAULT_MIDPOINT_2):
        """
        Finds the middle point between 2 x,y locations. Default points are left and right wrists.

        Args:
            keypoints(dict): A dictionary of all pose keypoints.
            point_1_name(str): Name of keypoint 1 to use in metric.
            point_2_name(str): Name of keypoint 2 to use in metric.

        Returns:
            tuple[float, float, float]: The midpoint x,y given two points and a score of between 0-1 based on proximity
                to left and right hand respectively.
        """
        point_1 = keypoints[point_1_name]["position"]
        point_2 = keypoints[point_2_name]["position"]
        x_diff = abs(point_1[0] - point_2[0])
        y_diff = abs(point_1[1] - point_2[1])
        midpoint_x = min(point_1[0], point_2[0]) - (x_diff / 2)
        midpoint_y = min(point_1[1], point_2[1]) - (y_diff / 2)
        proximity_x = (midpoint_x - min(point_1[0], point_2[0])) / x_diff
        proximity_y = (midpoint_y - min(point_1[1], point_2[1])) / x_diff
        return midpoint_x, midpoint_y, (proximity_x + proximity_y) / 2

    @staticmethod
    def centroid(keypoints, point_list=None):
        """
        Returns the mean x,y coordinates as a midpoint from a list of specified point names.
        Defaults to entire part map.

        Args:
            keypoints(dict): A dictionary of all pose keypoints.
            point_list(list[str]): A list of keypoint position names present in the part map.

        Returns:

        """
        if point_list is None:
            point_list = PART_MAP.values()
        x_list = []
        y_list = []
        for point in point_list:
            if point in PART_MAP.values():
                x_list.append(keypoints[point][0])
                y_list.append(keypoints[point][1])
        return np.mean(x_list), np.mean(y_list)

    def speed_of_points(self, keypoints, point_list=None):
        if point_list is None:
            point_list = PART_MAP.values()
        for point in keypoints:
            if point in PART_MAP.values():


    metric_list = {
        "offset_midpoints": midpoint,
        "centroid": centroid,
        "speed_of_hands": speed_of_hands
    }




if __name__ == '__main__':
    # Startup for node.
    node = PoseParserNode()
    node.listener()
    # node.test_publish()
