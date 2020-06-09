### BodyHub节点简介

bodyhub节点是上位机其他节点与下位机通信的中间节点，机器人的所有控制指令和数据获取都通过此节点实现。  BodyHub实现了一个状态机，用BodyHub统一管理下位机可确保如舵机这类设备在同一时刻只有一个上层节点控制，避免节点之间的干扰导致控制异常，详细内容可参考相关文档。

#### 针对机器人控制

- 下发舵机控制数据
- 读取舵机位置数据

#### 针对传感器

- 提供获取传感器数据注册服务
- 读取注册的传感器数据
- 处理执行器控制请求并下发到控制板

---

### 运行BodyHub节点

#### 准备工作

- 上位机已安装ROS，可正常运行
- 本地ROS工作空间内已有`bodyhub`、`dynamixelsdk`、`dynamixel-workbench`和`dynamixel_workbench_msgs`功能包
- 机器人下位机串口连接到上位机USB，并Linux系统识别到，记下`ttyUSBx`的x值（一般为0）
- 打开`bodyhub/src/BodyHubNode.cpp`文件，在主函数找到`STATEinit`函数，在`STATEinit`函数中找到`const char *portName = "/dev/ttyUSB0";`代码，修改`"ttyUSB0"`为上一步记下的值
- 编译ROS工作空间
- 下位机至少连接一个传感器或执行器

#### 运行节点
以下内容假设ttyUSBx号为ttyUSB0
- 打开终端运行命令`roscore`，打开ros master
- 打开新终端运行命令`sudo chmod 666 /dev/ttyUSB0`，回车后输入自己的密码确认，修改USB设备权限
- 继续在终端运行命令`setserial /dev/ttyUSB0 low_latency`，设置串口参数
- 继续在终端运行命令`roslaunch bodyhub bodyhub.launch `[^脚注1]，等待节点运行即可 


[^脚注1]: `bodyhub.launch `为BodyHub节点启动文件，此文件加载了节点所需的参数并启动节点的运行，需使用`roslaunch`命令运行，不能直接用`rosrun`运行节点，否则会导致节点所需的参数无法加载。

