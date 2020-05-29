#rosrun r200_publisher r200_publisher &
#roslaunch myoctomap_server octomap_tracking_server.launch &

source devel_isolated/robot/setup.bash 
rosrun robot robot_node &

./scripts/cam.sh &

source devel_isolated/myoctomap_server/setup.bash
rosrun myoctomap_server myoctomap_tracking_server_node o.bt &

source devel_isolated/humanoid_planner_2d/setup.bash
rosrun humanoid_planner_2d sbpl_2d_planner_node & 

source  devel_isolated/setup.bash
rviz &

source devel_isolated/SLAM/setup.bash
rosrun SLAM RGBD utils/ORBvoc.bin utils/rgbd.yaml true false
