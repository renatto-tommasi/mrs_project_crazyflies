Clone into to your CrazySim ros2_ws/src
Reynolds:
ros2 launch mrs_project_crazyflies reynolds.launch.py 

Rendezvous:
ros2 run mrs_project_crazyflies consensus_rendezvous --ros-args -p topology:=1

Formation
ros2 launch mrs_project_crazyflies reynolds.launch.py 
ros2 run mrs_project_crazyflies consensus_formation --ros-args -p topology:=1 -p formation:="square"
