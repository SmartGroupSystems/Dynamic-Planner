#! /usr/bin/env bash
set -e

RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch so3_quadrotor_simulator simulator_env.launch" gnome-terminal --title="Simulator_env" --tab &
sleep 2;

RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch so3_quadrotor_simulator simulator_uav.launch" gnome-terminal --title="Simulator_uav" --tab &
sleep 2;

RUN_AFTER_BASHRC="source devel/setup.bash;rosbag play -l dyn_map.bag" gnome-terminal --title="sim_map" --tab &
sleep 2;

RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch mapping mapping_sta_dyn.launch" gnome-terminal --title="Mapping" --tab &
sleep 2;

RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch grid_path_searcher astar_dyn.launch" gnome-terminal --title="Astar" --tab & 
sleep 2 ;

RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch bspline_race planning_with_sta_dyn.launch" gnome-terminal --title="Bspline" --tab &
wait
exit 0
