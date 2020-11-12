#! /usr/bin/python
import time

from flask import Flask, render_template, json, request, send_from_directory
from flask_cors import CORS, cross_origin
from parser import PoseParserNode
import logging
"""
Runs a simple Flask server for communication between Posenet and ROS
"""

app = Flask(__name__)
CORS(app)
app.config['CORS_HEADERS'] = 'Content-Type'
pose_parser = PoseParserNode.instance()


@app.route('/<path:path>')
def send_js(path):
    return send_from_directory('templates', path)

@app.route("/", methods=['GET', 'POST', 'OPTIONS'])
@cross_origin()
def index():
    """
    Contact point for Posenet to send data to.
    Publishes the data to a publish topic for Pose Parser Node.

    """

    # time.sleep(0.1)
    return app.send_static_file("camera.html")

# @app.route("/extra", methods=['GET', 'POST', 'OPTIONS'])
# @cross_origin()
# def send_js():
#     return app.send_static_file("camera.b3ee27ff.js")

@app.route("/backend", methods=['GET', 'POST', 'OPTIONS'])
def coco():
    """
    Initial test functionality from posenet.

    """
    data = list(request.get_json())

    if type(data) is list:
        data = data[0]
        pose_parser.callback(data)
    return "", 200


# Start server.
app.run(host="0.0.0.0", debug=False)

