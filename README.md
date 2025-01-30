# Multi-Robot Systems: Swarm Control using Consensus Protocols and Reynold's Rules

This project implements consensus protocols for multi-robot systems in ROS2, building upon previous work using Reynolds' rules for collision avoidance. It utilizes the Crazyflies simulation in the CrazySim Gazebo environment.

## How to Run

**Prerequisites:**

* ROS2 installed and configured
* CrazySim ROS2 workspace cloned and built (`colcon build --packages-select mrs_project_crazyflies --merge-install`)
* Workspace sourced (`source install/setup.sh`)

**Running the Nodes:**

**Reynolds:**

```bash
ros2 launch mrs_project_crazyflies reynolds.launch.py num_of_robots:=3
```

**Consensus Rendezvous:**
```bash
ros2 run mrs_project_crazyflies consensus_rendezvous --ros-args -p num_of_robots:=3 -p topology:=1 
```
**Consensus Formation:**
```bash
ros2 launch mrs_project_crazyflies reynolds.launch.py num_of_robots:=3
ros2 run mrs_project_crazyflies consensus_formation --ros-args -p num_of_robots:=3 -p topology:=1 -p formation:="triangle"
```
