#! /bin/bash
# This script safely launches ros nodes with buffer time to allow param server population
xterm -e roslaunch kuka_arm target_description.launch &
sleep 5 &&
xterm -e roslaunch kuka_arm cafe.launch &
sleep 5 &&
xterm -e roslaunch kuka_arm spawn_target.launch &
sleep 10 &&
#x-terminal-emulator -e roslaunch kuka_arm inverse_kinematics.launch
roslaunch kuka_arm inverse_kinematics.launch
