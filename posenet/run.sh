#!/bin/bash
service apache2 restart
source /pose/devel/setup.bash
rosrun publisher poser.py