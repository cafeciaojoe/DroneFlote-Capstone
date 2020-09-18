#!/bin/bash
service apache2 restart
source /parser/devel/setup.bash
rosrun parser parser.py
