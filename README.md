## Kinodynamic Motion Planning in Dynamic Environment for robot soccer
![gif](demo_short2.gif)  
click [here](https://youtu.be/jYTKOSrrcoY) to view full video  

### You'll need :
- ROS
- gcc >= 7 (for C++17)
- Cython

### Cloning  
do recursive cloning :   
```
git clone --recursive https://alifahrri@bitbucket.org/alifahrri/robosoccer_motion_planning_ws.git
```

### Nodes
* the core of the motion planning algorithm `robosoccer_motion_planning/src/robosoccer_motion_planning_node`
* trajectory tracking node for robot soccer `robosoccer_trajectory_tracking/scripts/trajectory_cytracker.py` 
* control robots with waypoints `robosoccer_teleop/scripts/robosoccer_teleop_node.py`
* example of trajectory generator as server node `robosoccer_motion_planning/src/trajectory_generator_node`
* example of trajectory generator client node `robosoccer_motion_planning/scripts/trajectory_demo.py`
   