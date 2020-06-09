#!/bin/bash

rosservice call MediumSize/BodyHub/StateJump 1 setStatus    # 动作调试节点获取控制 masterID = 1
sleep 2



echo "MediumSize/BodyHub/DirectMethod/SyncWrite"
rosservice call MediumSize/BodyHub/DirectMethod/SyncWrite "Goal_Position" "[1,2,3,19,20]" 5 "[45,45,45,45,45]"
sleep 2

echo "MediumSize/BodyHub/DirectMethod/InstRead"
rosservice call MediumSize/BodyHub/DirectMethod/InstRead Present_Temperature 19
sleep 2

echo "MediumSize/BodyHub/DirectMethod/InstWrite"
rosservice call MediumSize/BodyHub/DirectMethod/InstWrite Goal_Position 19 90
sleep 2

echo "MediumSize/BodyHub/DirectMethod/SetServoLockState"
rosservice call MediumSize/BodyHub/DirectMethod/SetServoLockState 20 0           
sleep 2

echo "MediumSize/BodyHub/DirectMethod/SetServoLockStateAll"
rosservice call MediumSize/BodyHub/DirectMethod/SetServoLockStateAll "[1,2,3,4,5,6,19]" 7 "[1,1,1,1,1,1,0]"
sleep 2

echo "MediumSize/BodyHub/DirectMethod/GetServoLockStateAll"
rosservice call MediumSize/BodyHub/DirectMethod/GetServoLockStateAll "[1,2,3,4,5,6,19,20]" 8
sleep 2

echo "MediumSize/BodyHub/DirectMethod/SetServoTarPosition"
rosservice call MediumSize/BodyHub/DirectMethod/SetServoTarPosition 1 90
sleep 2

echo "MediumSize/BodyHub/DirectMethod/SetServoTarPositionAll"
rosservice call MediumSize/BodyHub/DirectMethod/SetServoTarPositionAll "[1,2,3,4,5,6,19,20]" 8 "[45,45,45,45,45,45,45,45]"
sleep 2

echo "MediumSize/BodyHub/DirectMethod/GetServoPositionAll"
rosservice call MediumSize/BodyHub/DirectMethod/GetServoPositionAll "[1,2,3,4,5,6,19,20]" 8
sleep 2


echo "准备终止 rosmaster"
sleep 1
killall -9 rosmaster



