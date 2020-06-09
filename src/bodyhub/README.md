# bodyhub Package

## 概述
bodyhub节点是上位机其他节点与下位机通信的中间节点，机器人的所有控制指令的发送和机器人数据的获取都通过此节点实施。  
bodyhub实现了一个状态机来管理下位机，用bodyhub状态机管理下位机可确保如舵机这类执行器设备，在同一时刻只受一个上层节点控制，避免上层节点之间相互干扰导致控制异常。  
bodyhub节点有两种运行模式，分别是普通模式和仿真模式。
## 如何运行
#### 普通模式运行
在终端运行以下命令，回车后输入用户的密码确认，修改USB设备权限
```
sudo chmod 666 /dev/ttyUSB0
```
在终端运行以下命令，设置串口参数
```
setserial /dev/ttyUSB0 low_latency
```
在终端运行以下命令，以普通模式启动节点
```
roslaunch bodyhub bodyhub.launch
```
#### 仿真模式运行
仿真模式下的仿真场景是在vrep里面运行的。  
打开终端输入以下命令打开ros master
```
roscore
```
打开vrep仿真软件，并用软件打开`*/bodyhub/vrep/Roban.ttt`场景文件。  
在新终端执行以下命令，以仿真模式启动节点
```
roslaunch bodyhub bodyhub.launch sim:=true
```
## 使用状态机
使用id为6的节点占用状态机，向/MediumSize/BodyHub/StateJump服务发送如下请求
```
rosservice call /MediumSize/BodyHub/StateJump "masterID: 6 
stateReq: 'setStatus'"
```
若返回以下内容，则占用成功
```
stateRes: 22
```
若返回其他内容，则占用失败。  
释放对状态机的占用，/MediumSize/BodyHub/StateJump服务发送如下请求
```
rosservice call /MediumSize/BodyHub/StateJump "masterID: 6 
stateReq: 'reset'"
```
若返回以下内容，则释放成功
```
stateRes: 21
```
若返回其他内容，则释放失败。
## 订阅的话题
- /MediumSize/BodyHub/MotoPosition (bodyhub::JointControlPoint)  
用于所有关节的控制。消息定义
  ```
  float64[] positions
  float64[] velocities
  float64[] accelerations
  float64[] effort
  duration  time_from_start
  uint16    mainControlID
  ```
  mainControlID占用节点的控制id。
- /MediumSize/BodyHub/HeadPosition (bodyhub::JointControlPoint)  
用于头部关节的控制。
- /BodyHub/SensorControl (bodyhub::SensorControl)  
用于外接执行器的控制。消息定义
  ```
  string SensorName
  uint16 SetAddr
  uint8[] ParamList
  ```
  SensorName执行器名称，SetAddr执行器参数地址，ParamList执行器参数列表。
- /gaitCommand ([std_msgs/Float64MultiArray][std_msgs/Float64MultiArray])  
行走模式下用于机器人行走控制的脚步。
- /simulationStepDone ([std_msgs/Bool][std_msgs/Bool])  
表示仿真的一帧执行完成。
- /simulationState ([std_msgs/Int32][std_msgs/Int32])  
仿真的状态。
- /sim/joint/angle ([std_msgs/Float64MultiArray][std_msgs/Float64MultiArray])  
仿真机器人的关节角度。
- /sim/joint/velocity ([std_msgs/Float64MultiArray][std_msgs/Float64MultiArray])  
仿真机器人的关节速度。
- /sim/force/leftFoot ([std_msgs/Float64MultiArray][std_msgs/Float64MultiArray])  
仿真机器人的左脚力传感器的数值。
- /sim/force/rightFoot ([std_msgs/Float64MultiArray][std_msgs/Float64MultiArray])  
仿真机器人的右脚力传感器的数值。
## 发布的话题
- /MediumSize/BodyHub/Status ([std_msgs/UInt16][std_msgs/UInt16])  
节点状态机的状态。状态定义
  ```
  enum StateStyle {
    init = 20,
    preReady,
    ready,
    running,
    pause,
    stoping,
    error,
    directOperate,
    walking
  };
  ```
- /MediumSize/BodyHub/ServoPositions (bodyhub::ServoPositionAngle)  
机器人的关节角度。
- /jointPosTarget ([std_msgs/Float64MultiArray][std_msgs/Float64MultiArray])  
机器人关节的期望角度。
- /jointPosMeasure ([std_msgs/Float64MultiArray][std_msgs/Float64MultiArray])  
机器人关节的实际角度。
- /jointVelTarget ([std_msgs/Float64MultiArray][std_msgs/Float64MultiArray])  
机器人关节的期望速度。
- /jointVelMeasure ([std_msgs/Float64MultiArray][std_msgs/Float64MultiArray])  
机器人关节的实际速度。
- MediumSize/BodyHub/SensorRaw (bodyhub::SensorRawData)  
外接传感器的原始数据。消息定义
  ```
  uint8[]   sensorReadID
  uint16[]  sensorStartAddress
  uint16[]  sensorReadLength
  int32[]   sensorData
  uint8     sensorCount
  uint8     dataLength
  ```
  sensorReadID外接传感器id列表，sensorStartAddress外接传感器起始地址列表，sensorReadLength外接传感器数据长度列表，sensorData外接传感器原始数据列表，sensorCount外接传感器个数，dataLength数据总长。
- /requestGaitCommand ([std_msgs/Bool][std_msgs/Bool])  
行走模式下，机器人中脚步指令低于一定值时，请求新的控制脚步。
- /startSimulation ([std_msgs/Bool][std_msgs/Bool])  
开始仿真。
- /stopSimulation ([std_msgs/Bool][std_msgs/Bool])  
停止仿真。
- /pauseSimulation ([std_msgs/Bool][std_msgs/Bool])  
暂停仿真。
- /enableSyncMode ([std_msgs/Bool][std_msgs/Bool])  
使能同步仿真模式。
- /triggerNextStep ([std_msgs/Bool][std_msgs/Bool])  
触发仿真运行一帧。
- /sim/joint/command ([std_msgs/Float64MultiArray][std_msgs/Float64MultiArray])  
仿真机器人的关节角度指令。
## 提供的服务
- /MediumSize/BodyHub/StateJump (bodyhub::SrvState)  
节点状态机跳转。服务定义
  ```
  uint8 masterID
  string stateReq
  ---
  int16 stateRes
  ```
  masterID占用节点的id，stateReq状态跳转的控制字符串，stateRes操作返回的状态数值。
- /MediumSize/BodyHub/GetStatus (bodyhub::SrvString)  
返回节点状态机的状态；返回关节指令队列的长度。服务定义
  ```
  string str
  ---
  string data
  uint32 poseQueueSize
  ```
  str请求字符串，可任意设置，不能为空，data状态的字符串，poseQueueSize关节指令度列的长度。
- /MediumSize/BodyHub/GetMasterID (bodyhub::SrvTLSstring)  
返回控制节点的ID。服务定义
  ```
  string str
  ---
  uint8 data
  ```
  str请求的字符串，可任意设置，data返回的id值。
- /MediumSize/BodyHub/GetJointAngle (bodyhub::SrvServoAllRead)  
返回机器人关节的当前角度。
- /MediumSize/BodyHub/DirectMethod/InstReadVal (bodyhub::SrvInstRead)  
读Dynamixel设备寄存器。
- /MediumSize/BodyHub/DirectMethod/InstWriteVal (bodyhub::SrvInstWrite)  
写Dynamixel设备寄存器。
- /MediumSize/BodyHub/DirectMethod/SyncWriteVal (bodyhub::SrvSyncWrite)  
同步写Dynamixel寄存器。
- /MediumSize/BodyHub/DirectMethod/SetServoTarPositionVal (bodyhub::SrvServoWrite)  
设置单个舵机目标位置的值。
- /MediumSize/BodyHub/DirectMethod/SetServoTarPositionValAll (bodyhub::SrvServoAllWrite)  
设置全部舵机目标位置的值。
- /MediumSize/BodyHub/DirectMethod/GetServoPositionValAll (bodyhub::SrvServoAllRead)  
获取全部舵机位置的值。
- /MediumSize/BodyHub/DirectMethod/InstRead (bodyhub::SrvInstRead)  
读Dynamixel设备寄存器。
- /MediumSize/BodyHub/DirectMethod/InstWrite (bodyhub::SrvInstWrite)  
写Dynamixel设备寄存器。
- /MediumSize/BodyHub/DirectMethod/SyncWrite (bodyhub::SrvSyncWrite)  
同步写Dynamixel寄存器。
- /MediumSize/BodyHub/DirectMethod/SetServoTarPosition (bodyhub::SrvServoWrite)  
设置单个舵机目标位置的角度。 
- /MediumSize/BodyHub/DirectMethod/SetServoTarPositionAll (bodyhub::SrvServoAllWrite)  
设置全部舵机目标位置的角度。
- /MediumSize/BodyHub/DirectMethod/GetServoPositionAll (bodyhub::SrvServoAllRead)  
获取全部舵机位置的角度。
- /MediumSize/BodyHub/DirectMethod/SetServoLockState (bodyhub::SrvServoWrite)  
设置单个舵机的扭矩开关。
- /MediumSize/BodyHub/DirectMethod/SetServoLockStateAll (bodyhub::SrvServoAllWrite)  
设置全部舵机的扭矩开关。
- /MediumSize/BodyHub/DirectMethod/GetServoLockStateAll (bodyhub::SrvServoAllRead)  
获取全部舵机的扭矩开关状态。
- MediumSize/BodyHub/RegistSensor (bodyhub::SrvInstWrite)  
注册外接传感器。服务定义
  ```
  string itemName
  uint8 dxlID
  float64 setData
  ---
  bool complete
  ```
  itemName注册的传感器的名称，dxlID要获取的传感器数据的寄存器地址，setData要获取的传感器数据的地址长度，complete注册成功与否的结果。
- MediumSize/BodyHub/DeleteSensor (bodyhub::SrvTLSstring)  
删除注册的外接传感器。服务定义
  ```
  string str
  ---
  uint8 data
  ```
  str要删除的传感器名称，data删除的结果。
## 调用的服务
无
## 参数
- poseOffsetPath (std::string, default:"")  
机器人零点文件路径。
- poseInitPath (std::string, default:"")  
机器人初始姿势文件路径。
- sensorNameIDPath (std::string, default:"")  
机器人外接传感器的信息文件路径。
- simenable (bool, default:false)  
仿真模式的控制变量。
## 附录
[std_msgs/Bool]: http://docs.ros.org/api/std_msgs/html/msg/Bool.html
[std_msgs/Int32]: http://docs.ros.org/api/std_msgs/html/msg/Int32.html
[std_msgs/UInt16]: http://docs.ros.org/api/std_msgs/html/msg/UInt16.html
[std_msgs/Float64]: http://docs.ros.org/api/std_msgs/html/msg/Float64.html
[std_msgs/Float64MultiArray]: http://docs.ros.org/api/std_msgs/html/msg/Float64MultiArray.html