#! /usr/bin/python
import time

from flask import Flask, render_template, json, request
from flask_cors import CORS, cross_origin
import logging
import rospy
from std_msgs.msg import String
"""
Runs a simple Flask server for communication between Posenet and ROS
"""

app = Flask("__main__")
CORS(app)
app.config['CORS_HEADERS'] = 'Content-Type'
pub = rospy.Publisher('pose_data', String, queue_size=20)
rospy.init_node('posenet', anonymous=True)


@app.route("/", methods=['GET', 'POST', 'OPTIONS'])
@cross_origin()
def index():
    """
    Contact point for Posenet to send data to.
    Publishes the data to a publish topic for Pose Parser Node.

    """
    if rospy.is_shutdown() is False:
        time.sleep(0.1)
        data = request.get_json()
        if type(data) is list:
            data = data[0]
        # rospy.loginfo(json.dumps(data["keypoints"]))
        pub.publish(json.dumps(data["keypoints"]))
        return "", 201
    else:
        return "", 410


@app.route("/pose", methods=['GET', 'POST', 'OPTIONS'])
def coco():
    """
    Initial test functionality from posenet.

    """
    print(request.get_json(force=True))

# Start server.
try:
    app.run(host="0.0.0.0", debug=False)
except rospy.ROSInterruptException:
    pass
