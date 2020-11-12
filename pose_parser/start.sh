#!/bin/bash
# Starts an apache server and runs the Node.
service apache2 restart
source /parser/devel/setup.bash
cfclient &
rosrun parser parser.py
