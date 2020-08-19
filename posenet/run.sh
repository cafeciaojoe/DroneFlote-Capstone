#!/bin/bash
service apache2 restart
source /opt/ros/melodic/setup.bash
python ./poser.py