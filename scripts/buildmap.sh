rosrun r200_publisher r200_publisher &

roslaunch myoctomap_server octomap_tracking_server.launch &

#rosrun myoctomap_server myoctomap_tracking_server_node o4.bt &

#rosrun humanoid_planner_2d sbpl_2d_planner_node &

#roslaunch footstep_planner planner_complete.launch &

rosrun SLAM RGBD utils/ORBvoc.bin utils/rgbd.yaml true false




