#!/bin/bash
source /simulator/devel/setup.bash
gzclient &
gazebo roslaunch rotors_gazebo crazyflie2_hovering_example.launch
rosrun demo_listener demo_listener.py