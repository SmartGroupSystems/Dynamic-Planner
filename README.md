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

(1) __Compile Error__:

(2) __Eigen Error__:

(3) __A-star Error__:

# Description
This repository mainly contains the following modules: 

(1)```bspline_race```: This module is the trajectory optimization module, primarily responsible for backend optimization, where the ```NLopt``` optimization library is used.

(2)```grid_path_searcher```: The path search module utilizes the A-star algorithm.

(3)```mapping```: A lightweight mapping module.

(4)```uav_simulator```: A lightweight quadrotor simulator.

We referred to a significant amount of code from [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) while writing our own code, and we express our deepest gratitude to the authors for their contributions.

# Compile
__Tested environment__: Ubuntu 20.04 + ROS Noetic


# Building Simulation Maps

__Click-map__: You can use the two built-in components of rviz to click on the map to generate static and dynamic obstacles, respectively, like this:


# Test Dynamic_Planner

![Example](https://github.com/SmartGroupSystems/Dynamic-Planner/blob/main/gif/tutieshi_640x360_18s.gif)


# Real World Experiment

