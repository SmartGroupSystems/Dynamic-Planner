roslaunch so3_quadrotor_simulator simulator_env.launch & sleep 2;
roslaunch so3_quadrotor_simulator simulator_uav.launch & sleep 2;
rosrun map_generator click_map_dynamic

