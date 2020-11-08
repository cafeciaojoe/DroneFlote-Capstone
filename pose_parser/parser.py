#!/usr/bin/env python
import math

import rospy
import eigenpy
from std_msgs.msg import String, Header
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, Vector3
import mav_msgs
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
    # Options for default metric are: TODO
    DEFAULT_METRIC = "demo_metric"
    MINIMUM_CONFIDENCE = 0.5

    def __init__(self):
        self.metrics = PoseMetrics()
        self.metric_functions = PoseMetrics.metric_list
        rospy.init_node('parser_node', anonymous=True)

    def convert_to_dictionary(self, data):
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
        self.metrics.register_keypoints(pose_dict)
        return pose_dict

    def callback(self, data):
        """
        Callback function for ROS pub/sub model.
        Converts message to dictionary and runs currently selected metric.

        Args:
            data: Data received from ROS subscription.

        """
        rospy.loginfo("Callback Called")
        points_data = json.loads(data.data)
        keypoints = self.convert_to_dictionary(points_data)
        # self.simulation_pose_demo(keypoints) TODO
        # self.test_metrics(keypoints)
        trajectory_points = self.metric_functions[self.DEFAULT_METRIC](keypoints)
        self.publisher(trajectory_points)

    def listener(self):
        """
        Starts the node listening on subscribed topics until shut down.
        """
        rospy.Subscriber("pose_data", String, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def publisher(self, trajectory_parameters):
        """
        Generates and publishes a MultiDOFJointTrajectory message from inputs to publisher topic for simulator.

        Args:
            trajectory_parameters(dict): A dictionary containing all data fields required to build a Trajectory message.
        """
        trajectory = MultiDOFJointTrajectory()
        trajectory.header = Header()
        trajectory.header.stamp = rospy.Time()
        trajectory.header.frame_id = ''
        trajectory.joint_names = ["base_link"]
        point = MultiDOFJointTrajectoryPoint([self.create_point(trajectory_parameters["x"],
                                                                trajectory_parameters["y"],
                                                                trajectory_parameters["z"],
                                                                trajectory_parameters["default_rotation_x"],
                                                                trajectory_parameters["rotation_y"],
                                                                trajectory_parameters["rotation_z"],
                                                                trajectory_parameters["rotation_w"])],
                                             [self.create_velocity(trajectory_parameters["velocity_x"],
                                                                   trajectory_parameters["velocity_y"],
                                                                   trajectory_parameters["velocity_z"],
                                                                   trajectory_parameters["velocity_angular_x"],
                                                                   trajectory_parameters["velocity_angular_y"],
                                                                   trajectory_parameters["velocity_angular_z"])],
                                             [self.create_acceleration(trajectory_parameters["acceleration_linear_x"],
                                                                       trajectory_parameters["acceleration_linear_y"],
                                                                       trajectory_parameters["acceleration_linear_z"],
                                                                       trajectory_parameters["acceleration_angular_x"],
                                                                       trajectory_parameters["acceleration_angular_y"],
                                                                       trajectory_parameters[
                                                                           "acceleration_angular_z"])],
                                             rospy.Time(1))
        trajectory.points.append(point)
        pub.publish(trajectory)

    def create_point(self, x, y, z, x_2=0, y_2=0, z_2=0, w=1):
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

    def create_velocity(self, x, y, z, x_2=1, y_2=1, z_2=1):
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

    def create_acceleration(self, x, y, z, x_2=1, y_2=1, z_2=1):
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

    def test_metrics(self, keypoints):
        self.metrics.midpoint(keypoints)
        self.metrics.centroid(keypoints)
        self.metrics.avg_speed_of_points()


class PoseMetrics:
    """
    Container class for pose metrics.
    """
    # Default Focus for the average_speed metric
    DEFAULT_FOCUS_POINT_1 = "leftWrist"
    DEFAULT_FOCUS_POINT_2 = "rightWrist"

    # Default focus for angle threshold demo.
    ANGLE_THRESHOLD = 15
    previous_angle = None
    POSITION_BASE = "rightShoulder"
    POSITION_OUTER = "rightWrist"

    # Length of list to calculate averages from history.
    DEFAULT_HISTORY_LENGTH = 50

    # Confidence score required to validate a keypoint set.
    MINIMUM_CONFIDENCE = 0.5

    # Used for demo_metric.
    high = None

    # Default values for metric return properties.
    default_rotation_x = 0
    default_rotation_y = 0
    default_rotation_z = 0
    default_rotation_w = 1
    default_velocity_x = 1
    default_velocity_y = 1
    default_velocity_z = 1
    default_velocity_angular_x = 1
    default_velocity_angular_y = 1
    default_velocity_angular_z = 1
    default_acceleration_linear_x = 1
    default_acceleration_linear_y = 1
    default_acceleration_linear_z = 1
    default_acceleration_angular_x = 1
    default_acceleration_angular_y = 1
    default_acceleration_angular_z = 1

    def __init__(self, history_length=DEFAULT_HISTORY_LENGTH):
        self.history_length = history_length
        self.history = [{}]
        self.centroid_history = []

    def register_keypoints(self, keypoints):
        """
        Takes a dictionary of parsed pose data and adds it to the history list with timestamp.
        Ideally, should only be called once, immediately after keypoints are parsed.

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
        if len(self.centroid_history) > self.history_length:
            self.centroid_history.pop()

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
        rospy.loginfo("Offset Mid\nPoint 1: %s @ %s, Point 2: %s @ %s, Mid-Point: %s, Proximity Score: %s" %
                      (point_1_name, str(point_1), point_2_name, str(point_2), str((midpoint_x, midpoint_y)),
                       str((proximity_x + proximity_y) / 2)))
        return midpoint_x, midpoint_y, (proximity_x + proximity_y) / 2

    @staticmethod
    def centroid(keypoints, point_list1=(DEFAULT_FOCUS_POINT_1, DEFAULT_FOCUS_POINT_2), point_list2=None):
        """
        Returns the mean x,y coordinates as a midpoint from a list of specified point names.
        Defaults to entire part map.

        Args:
            keypoints(dict): A dictionary of all pose keypoints.
            point_list1(list[str]): A list of keypoint position names present in the part map.

        Returns:

        """
        if point_list1 is None:
            point_list1 = PART_MAP.values()
        x_list = []
        y_list = []
        for point in point_list1:
            if point in PART_MAP.values():
                x_list.append(keypoints[point][0])
                y_list.append(keypoints[point][1])
        midpoint = (np.mean(x_list), np.mean(y_list))
        rospy.loginfo("Centroid\nMidpoint: %s" % str(midpoint))
        return midpoint

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
        rospy.loginfo("Average Speeds\n%s" % str(speed_dict))
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

    def positional_demo(self, keypoints):
        """
        Measures horizontal angle between two points and logs when the angle passes between a threshold angle.
        Logs results to console as True/False based on user interaction.

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

    def simulation_pose_demo(self, keypoints):
        """
        Midpoint metric for simulation demo. Monitors right wrist in relation to middle point between nose and knees.
        Logs locations of key-points to console and forwards message to simulator.

        Args:
            keypoints(dict): A dictionary of parsed posenet key-points.

        """
        ret_dict = None
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
                    ret_dict = self.create_return_dictionary(x=0, y=0, z=3)
                else:
                    ret_dict = self.create_return_dictionary(x=0, y=0, z=1)
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
        return ret_dict

    def create_return_dictionary(self, x=None, y=None, z=None,
                                 rotation_x=default_rotation_x, rotation_y=default_rotation_y,
                                 rotation_z=default_rotation_z, rotation_w=default_rotation_w,
                                 velocity_x=default_velocity_x, velocity_y=default_velocity_y,
                                 velocity_z=default_velocity_z,
                                 velocity_angular_x=default_velocity_angular_x,
                                 velocity_angular_y=default_velocity_angular_y,
                                 velocity_angular_z=default_velocity_angular_z,
                                 acceleration_linear_x=default_acceleration_linear_x,
                                 acceleration_linear_y=default_acceleration_linear_y,
                                 acceleration_linear_z=default_acceleration_linear_z,
                                 acceleration_angular_x=default_acceleration_angular_x,
                                 acceleration_angular_y=default_acceleration_angular_y,
                                 acceleration_angular_z=default_acceleration_angular_z,
                                 proximity_value=None):
        """
        Creates a generically formatted dictionary based on any given values, filling unspecified parameters with
        predefined default values.

        Args:
            x: x Location Co-ordinates.
            y: y Location Co-ordinates.
            z: z Location Co-ordinates.
            rotation_x: x rotation value.
            rotation_y: y rotation value.
            rotation_z: z rotation value.
            rotation_w: w rotation value.
            velocity_x: x linear velocity.
            velocity_y: y linear velocity.
            velocity_z: z linear velocity.
            velocity_angular_x: x angular velocity.
            velocity_angular_y: y angular velocity.
            velocity_angular_z: z angular velocity.
            acceleration_linear_x: x linear acceleration.
            acceleration_linear_y: y linear acceleration.
            acceleration_linear_z: z linear acceleration.
            acceleration_angular_x: x angular acceleration.
            acceleration_angular_y: y angular acceleration.
            acceleration_angular_z: z angular acceleration.
            proximity_value: Value between 0-1 corresponding to drones proximity to a left or right keypoint.

        Returns:
            dict: The dictionary of all these values, populated with defaults for unspecified values.
        """
        dict_of_points = {
            "x": x,
            "y": y,
            "z": z,
            "rotation_x": rotation_x,
            "rotation_y": rotation_y,
            "rotation_z": rotation_z,
            "rotation_w": rotation_w,
            "velocity_x": velocity_x,
            "velocity_y": velocity_y,
            "velocity_z": velocity_z,
            "velocity_angular_x": velocity_angular_x,
            "velocity_angular_y": velocity_angular_y,
            "velocity_angular_z": velocity_angular_z,
            "acceleration_linear_x": acceleration_linear_x,
            "acceleration_linear_y": acceleration_linear_y,
            "acceleration_linear_z": acceleration_linear_z,
            "acceleration_angular_x": acceleration_angular_x,
            "acceleration_angular_y": acceleration_angular_y,
            "acceleration_angular_z": acceleration_angular_z,
            "proximity": proximity_value
        }
        return dict_of_points

    # Helper dictionary for selecting functions for metrics.
    metric_list = {
        "positional_demo": positional_demo,
        "demo_metric": simulation_pose_demo,
        "offset_midpoints": midpoint,
        "centroid": centroid,
        "average_speed_of_points": avg_speed_of_points
    }


if __name__ == '__main__':
    # Startup for node.
    node = PoseParserNode()
    node.listener()
    # node.test_publish()
