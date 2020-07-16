roscore

sudo /sbin/ifconfig enp0s25 down
sudo /sbin/ifconfig enp0s25 hw ether 08:ed:b9:c6:d2:b9
sudo /sbin/ifconfig enp0s25 up

./vrep.sh

source devel_isolated/setup.bash
roslaunch bodyhub bodyhub.launch sim:=true

source devel_isolated/bodyhub/setup.bash
rosservice call /MediumSize/BodyHub/StateJump 6 setStatus

source devel_isolated/bodyhub/setup.bash
rosservice call /MediumSize/BodyHub/StateJump 6 walking

source devel_isolated/gait_command/setup.bash
rosrun gait_command gait_command_node 

source devel_isolated/path_controler/setup.bash
rosrun path_controler path_controler

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




