#! /usr/bin/python
from flask import Flask, render_template, json, request
from flask_cors import CORS, cross_origin
import rospy
import logging

app = Flask("__main__")
CORS(app)
app.config['CORS_HEADERS'] = 'Content-Type'


@app.route("/", methods=['GET', 'POST', 'OPTIONS'])
@cross_origin()
def index():
    data = request.get_json()
    for point in data:
        app.logger.info(str(point["keypoints"]))
    return "", 201


@app.route("/pose", methods=['GET', 'POST', 'OPTIONS'])
def coco():
    print(request.get_json(force=True))


app.run(host="0.0.0.0", debug=True)
