roscore

./vrep.sh

source devel_isolated/bodyhub/setup.bash
roslaunch bodyhub bodyhub.launch sim:=true

source devel_isolated/bodyhub/setup.bash
rosservice call /MediumSize/BodyHub/StateJump "masterID: 6
> stateReq: ’setStatus’"



source devel_isolated/robot/setup.bash
rosrun robot robot_node

source devel_isolated/img_publisher/setup.bash
rosrun img_publisher img_publisher

source devel_isolated/SLAM/setup.bash
rosrun SLAM RGBD utils/ORBvoc.bin utils/rgbd.yaml true true

#launch ocotmap_server
source devel_isolated/octomap_server/setup.bash
roslaunch octomap_server static-octomap_tracking_server.launch

source devel_isolated/humanoid_planner_2d/setup.bash
rosrun humanoid_planner_2d sbpl_2d_planner_node

source  devel_isolated/setup.bash
rviz




