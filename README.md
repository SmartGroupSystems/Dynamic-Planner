# Dynamic-Planner
A planner designed for quadrotors in dynamic complex environments. If you have any questions, please ask in the "issues" section or send email to ```ruochengli@bit.edu.cn```, thanks!

Video link：https://www.youtube.com/watch?v=TOEeoFO4OxY

Paper:

```
@article{li2024autonomous,
  title={Autonomous Navigation of Quadrotors in Dynamic Complex Environments},
  author={Li, Ruocheng and Xin, Bin},
  journal={IEEE Transactions on Industrial Electronics},
  year={2024},
  publisher={IEEE}
}

```

# Known Issues:

(1) __Compile Error__: If the compilation indicates that ```<bspline_race/BsplineTraj.h>``` or ```<map_generator/dynamic_obs.h>``` are missing, please try recompiling a few times.

-----
I have already add these two files in the folder,,,,
please place them under the ```/devel/include/map_generator``` and ```/devel/include/bspline_race``` directory and try testing again.

(2) __Eigen Error__: If the Opt window reports an error related to Eigen matrices, it's likely due to the map being allocated too small, leading to a memory overflow error. You can resolve this by modifying the ```map_size``` parameter in ```mapping_sta_dyn.launch```, ```astar_dyn.launch```, and ```planning_with_sta_dyn.launch```. Note that in ```planning_with_sta_dyn```, the parameters ```start_x``` and ```start_y``` need to be adjusted. These values represent the center of the grid at the bottom-left corner of the map, so they must be decimals. For example, if the map size is ```80.0```, then ```start_x = -80/2 + 0.05 = -39.95``` and ```start_y = 80/2 - 0.05 = 39.95```. Since the code has been maintained for a long time, some bugs make it inconvenient to modify, so we'll keep it as is for now... I plan to refactor the entire project in future development.

(3) __A-star Error__: If sometimes sending a target doesn't elicit a response, it could be because the A* algorithm has stopped working. This happens when there are no point clouds in the local map, causing the A* algorithm to halt. Restarting the project can resolve this issue.

# Description
This repository mainly contains the following modules: 

(1)```bspline_race```: This module is the trajectory optimization module, primarily responsible for backend optimization, where the ```NLopt``` optimization library is used.

(2)```grid_path_searcher```: The path search module utilizes the A-star algorithm.

(3)```mapping```: A lightweight mapping module.

(4)```uav_simulator```: A lightweight quadrotor simulator.

We referred to a significant amount of code from [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) while writing our own code, and we express our deepest gratitude to the authors for their contributions.

# Compile
__Tested environment__: Ubuntu 20.04 + ROS Noetic

1.Install dependencies

The entire project depends on ```NLOPT```. First, please refer to [NLOPT](https://nlopt.readthedocs.io/en/latest/NLopt_Installation/) to install the NLOPT library.

Also, you need to have mavros.

2.Download and compile the repo

```
mkdir -p dynamic_planner_ws/src
cd dynamic_planner_ws/src
git clone https://github.com/SmartGroupSystems/Dynamic-Planner.git
cd ..
catkin_make
```

If the compilation indicates that ```<bspline_race/BsplineTraj.h>``` or ```<map_generator/dynamic_obs.h>``` are missing, please try recompiling a few times.

3.Configure the command-line tools

Open a new window and enter the following commands in the root directory:

```
sudo gedit .bashrc
```

Then, add the following commands at the bottom of the opened file window:

```
eval "$RUN_AFTER_BASHRC"
```


# Building Simulation Maps

__Click-map__: You can use the two built-in components of rviz to click on the map to generate static and dynamic obstacles, respectively, like this:

```
cd dynamic_planner_ws
source devel/setup.bash
./create_dyn_map.sh
```

Then in a new window, use the following command to save the published map as a rosbag file.

```
source devel/setup.bash
./save_dyn_map.sh
```
It is recommended to save a rosbag for at least ```30``` seconds to ensure a more coherent map. After running the script for more than 30 seconds, use ```Ctrl+C``` to pause the script, and you will obtain a file named ```dyn_map.bag```. This file will be used in the subsequent planning stage.

Please ensure that you have granted executable permissions to these two sh files before executing them.

![click_map](gif/click_map.gif)

# Test Dynamic_Planner

```
cd dynamic_planner_ws
source devel/setup.bash
./dynamic_planner.sh
```

![Example](https://github.com/SmartGroupSystems/Dynamic-Planner/blob/main/gif/tutieshi_640x360_18s.gif)

The core parts of dynamic obstacle avoidance refer to ```gvo.cpp``` and ```bspline_race.cpp```. Here, if a finite state machine is added to handle some extreme cases, the algorithm's performance will improve. 

If the planner indicates ```VO is complex... the result is not reliable,``` it means the collision avoidance was not successful. This issue may arise if obstacles are too dense or if the initial position is too close to the obstacles. I will work on improving and upgrading this in future developments. If you are interested, you are also welcome to give it a try!