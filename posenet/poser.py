#! /usr/bin/python
from flask import Flask, render_template, json, request
from flask_cors import CORS, cross_origin
import logging
import rospy
from std_msgs.msg import String


app = Flask("__main__")
CORS(app)
app.config['CORS_HEADERS'] = 'Content-Type'
pub = rospy.Publisher('pose_data', String, queue_size=20)
rospy.init_node('posenet', anonymous=True)


@app.route("/", methods=['GET', 'POST', 'OPTIONS'])
@cross_origin()
def index():
    if rospy.is_shutdown() is False:
        data = request.get_json()
        if type(data) is list:
            data = data[0]
        rospy.loginfo(json.dumps(point["keypoints"]))
        pub.publish(json.dumps(data["keypoints"]))
        return "", 201
    else:
        return "", 410


@app.route("/pose", methods=['GET', 'POST', 'OPTIONS'])
def coco():
    print(request.get_json(force=True))


try:
    app.run(host="0.0.0.0", debug=True)
except rospy.ROSInterruptException:
    pass
