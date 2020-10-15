#!/bin/bash
# Starts an apache server and runs the Node.
service apache2 restart
source /pose/devel/setup.bash
rosrun publisher poser.py