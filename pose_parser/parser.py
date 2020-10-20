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
}  # A 'timestamp' field is added to dictionary when parsed.

pub = rospy.Publisher('crazyflie2/command/trajectory', MultiDOFJointTrajectory, queue_size=20)


class PoseParserNode:
    """
    ROSpy Node for parsing data from posenet. Adds timestamp to notate when message was received.
    """
    ANGLE_THRESHOLD = 15
    previous_angle = None
    POSITION_BASE = "rightShoulder"
    POSITION_OUTER = "rightWrist"
    MINIMUM_CONFIDENCE = 0.5
    high = None

    def __init__(self):
        self.metrics = PoseMetrics()
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
        pose_dict["timestamp"] = rospy.Time.now()
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

            angle_horizontal = PoseMetrics.get_angle(keypoints[self.POSITION_BASE]["position"],
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

    def publisher(self, x, y, z, x_2=0, y_2=0, z_2=0, w=0):
        """
        Generates and publishes a MultiDOFJointTrajectory message from inputs to publisher topic for simulator.

        Args:
            x:
            y:
            z:
            x_2:
            y_2:
            z_2:
            w:
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

    def create_point(self, x, y, z, x_2=0, y_2=0, z_2=0, w=0):
        """
        Creates and returns a Transform object for messaging given x, y, z values.

        Returns:
            Transform: Transform from inputs.
        """
        transformation = Transform()
        transformation.translation.x = x
        transformation.translation.y = y
        transformation.translation.z = z
        transformation.rotation.x = x_2
        transformation.rotation.y = y_2
        transformation.rotation.z = z_2
        transformation.rotation.w = w
        return transformation

    def create_velocity(self, x, y, z, x_2=0, y_2=0, z_2=0):
        """
        Creates and returns a Velocity object for messaging given x, y, z values.

        Returns:
            Twist: Velocity from inputs.
        """
        velocity = Twist()
        velocity.linear.x = x
        velocity.linear.y = y
        velocity.linear.z = z
        velocity.angular.x = x_2
        velocity.angular.y = y_2
        velocity.angular.z = z_2
        return velocity

    def create_acceleration(self, x, y, z, x_2=0, y_2=0, z_2=0):
        """
        Creates and returns an Acceleration object for messaging given x, y, z values.

        Returns:
            Twist: Acceleration from inputs.
        """
        acceleration = Twist()
        acceleration.linear.x = x
        acceleration.linear.y = y
        acceleration.linear.z = z
        acceleration.angular.x = x_2
        acceleration.angular.y = y_2
        acceleration.angular.z = z_2
        return acceleration

    def test_publish(self):
        """
        Test function for sending dummy messages over publisher topic.

        """
        x = 5
        y = 0
        z = 0
        x_2 = 0
        y_2 = 0
        z_2 = 0
        w = 0
        # Rotate through each setting to test drone response once every 10 sec.
        while True:
            rospy.sleep(10)
            self.publisher(x, y, z, x_2, y_2, z_2, w)
            x_tmp = x
            y_tmp = y
            z_tmp = z
            x_2_tmp = x_2
            y_2_tmp = y_2
            z_2_tmp = z_2
            w_tmp = w

            x = w_tmp
            y = x_tmp
            z = y_tmp
            x_2 = z_tmp
            y_2 = x_2_tmp
            z_2 = y_2_tmp
            w = z_2_tmp


class PoseMetrics:
    """
    Container class for pose metrics.
    """
    DEFAULT_FOCUS_POINT_1 = "leftWrist"
    DEFAULT_FOCUS_POINT_2 = "rightWrist"
    DEFAULT_HISTORY_LENGTH = 20

    def __init__(self, history_length=DEFAULT_HISTORY_LENGTH):
        self.history_length = history_length
        self.history = [{}]
        for point_name in PART_MAP:
            self.history[point_name] = []

    def register_keypoints(self, keypoints):
        """
        Takes a dictionary of parsed pose data and adds it to the history list with timestamp.

        Args:
            keypoints(dict): Latest set of pose data as parsed dictionary.

        """
        data = {}
        for point in keypoints:
            if point in PART_MAP.values():
                data[point] = keypoints[point]
        self.history.insert(0, data)
        # Prune list if it gets too long
        if len(self.history) > self.history_length:
            self.history.pop()

    @staticmethod
    def midpoint(keypoints, point_1_name=DEFAULT_FOCUS_POINT_1, point_2_name=DEFAULT_FOCUS_POINT_2):
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
    def centroid(keypoints, point_list=(DEFAULT_FOCUS_POINT_1, DEFAULT_FOCUS_POINT_2)):
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

    def avg_speed_of_points(self, point_list=None):
        """
        Computes the average speed of one or more keypoints using recorded history.

        Args:
            point_list(list[str]): A list of keypoint names to measure.

        Returns:
            dict[str, float]: Dictionary of average speed of each point requested.
        """
        speed_dict = {}
        if point_list is None:
            point_list = PART_MAP.values()
        for point in point_list:
            if point in PART_MAP.values():
                speed_dict[point] = self.average_speed_of_point(point)
        return speed_dict

    @staticmethod
    def get_angle(base_point, outer_point):
        """
        Helper method for calculating angle between two points.

        Args:
            base_point(tuple[float, float]): First point
            outer_point(tuple[float, float]): Second point

        Returns:
            float: Angle given two points in degrees.
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

    @staticmethod
    def absolute_speed(point_name, keypoints_a, keypoints_b):
        """
        Quickly calculates the absolute velocity between two sets of x,y co-ordinates with given timestamps.
        Args:
            point_name(str): Name of the point to measure.
            keypoints_a(dict): Dictionary of keypoints for point A.
            keypoints_b(dict): Dictionary of keypoints for point B.

        Returns:
            float: Velocity for movement between two points irrespective of direction.
        """
        abs_speed = 0.0
        if point_name in PART_MAP.values():
            abs_speed = np.sqrt((abs(keypoints_a[point_name][0] - keypoints_b[point_name][0]) ** 2) +
                                (abs(keypoints_a[point_name][0] - keypoints_b[point_name][0]) ** 2)) / \
                        abs(keypoints_b["timestamp"].to_sec()) - keypoints_a["timestamp"].to_sec()
        return abs_speed

    def average_speed_of_point(self, point_name):
        """
        Calculate the overage speed for a point based on history of keypoints.

        Args:
            point_name(str): Name of keypoint to measure.

        Returns:
            float: Average speed of a point over our history irrespetive of direction.
        """
        avg_speed = 0.0
        previous_keypoints = None
        if len(self.history) >= 2 and point_name is not None and point_name in PART_MAP.values():
            for keypoints in self.history:
                if previous_keypoints is not None:
                    avg_speed += PoseMetrics.absolute_speed(point_name, keypoints, previous_keypoints)
                previous_keypoints = keypoints
            avg_speed /= len(self.history)
        return avg_speed

    # Helper dictionary for selecting functions for metrics.
    metric_list = {
        "offset_midpoints": midpoint,
        "centroid": centroid,
        "average_speed_of_points": avg_speed_of_points
    }


if __name__ == '__main__':
    # Startup for node.
    node = PoseParserNode()
    node.listener()
    # node.test_publish()
