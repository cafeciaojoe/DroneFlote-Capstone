#!/bin/bash
source /simulator/devel/setup.bash
#gzclient &
roslaunch rotors_gazebo crazyflie2_hovering_example.launch --wait &
rosrun demo_listener demo_listener.py