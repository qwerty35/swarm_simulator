# swarm_simulator

This package presents an efficient multi-agent trajectory planning algorithm which generates safe trajectories in obstacle-dense environments. 
Our algorithm combines the advantages of both grid-based and optimization-based approaches, and generates safe, dynamically feasible trajectory without suffering from an errorneous optimization setup such as imposing infeasible collision constraints.
The details can be found at the following link.

- **Authors:** Jungwon Park, Junha Kim, Inkyu Jang and H. Jin Kim from [LARR](http://larr.snu.ac.kr/), Seoul National Univ.
- **Paper:** Efficient Multi-Agent Trajectory Planning with Feasibility Guarantee using Relative Bernstein Polynomial [PDF Link](https://arxiv.org/abs/1909.10219)
- **Video:** [Youtube Link](https://www.youtube.com/watch?v=0koj-AlIbbI&list=PLdzwkGI22JhXa63sRb8zPTK3ZNFRmkdIu&index=2&t=0s)

## 0. Dependencies
Following sources are used to implement this package.
- [CPLEX](https://www.ibm.com/products/ilog-cplex-optimization-studio/resources)
- [octomap](https://github.com/OctoMap/octomap)
- [libMultiRobotPlanning](https://github.com/whoenig/libMultiRobotPlanning)
- [matplotlib-cpp](https://github.com/lava/matplotlib-cpp)

## 1. Install
(1) Install ROS Kinetic (for Ubuntu 16.04) or Melodic (for Ubuntu 18.04).

(2) Install CPLEX and fix CMAKELIST depending on intallation location. For instance:
```
set(CPLEX_PREFIX_DIR /opt/ibm/ILOG/CPLEX_Studio129)
```

(3) At terminal:
```
sudo apt-get install python-matplotlib python-numpy python2.7-dev
cd ~/catkin_ws/src
git clone https://github.com/qwerty35/swarm_simulator.git
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## 2. Demo
```
roslaunch swarm_planner plan_rbp_random_forest.launch
```

## 3. Simulation Configuration
You can configure the simulation setting at the launch file, plan_rbp_random_forest.launch.
runsim: true - You can see the planning result at the rviz.

log: true - You can see more detail message, and QPmodel and planning results will be saved in "swarm_planner/log"

mission: You can deploy the mission by editing the json file in "swarm_planner/missions" directory.

replay: false - It runs the simulation at the random forest. true - It runs the simulation at the map specified at 'replay_map' tag.
Map files are located in "swarm_planner/worlds", and should be octomap bt files.

plan_time_scale: true - Execute time scale to match dynamic limits specified at mission file.

plan_time_step: You can execute time scale with this tag manually unless plan_time_step is true 

plan_sequential: true - Execute seqeuntial planning. You can change the batch size at 'plan_batch_size' tag. 

plan_batch_iter: positive number - You can see the intermediate result of seqential planning. 0 - it shows initial trajectory. -1  - it means maximum batch iteration.


## 3. Notes
(1) You may turn off 'runsim', 'log' arguments to check the correct computation time.
(2) This code is not fully tested with various maps. Grid generation algorithm may not work with other maps.
