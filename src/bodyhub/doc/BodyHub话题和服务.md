BodyHub节点实现了我们功能所需的一些话题和服务，下面将简要介绍。
### 话题
##### 发布话题
- `MediumSize/BodyHub/Status`
 发布节点状态消息。此话题在初始化的时候创建，话题句柄`StatusPub`，在修改BodyHub节点状态时，话题会发布一个包含节点新状态的消息。消息格式：`std_msgs::UInt16`。状态定义在`bodyhubStatus.h`文件，实例如下：[^脚注1]
    ```
    enum StateStyle {
        init = 20,
        preReady,
        ready,
        running,
        pause,
        stoping,
        error,
        directOperate
    };
    ```

- `MediumSize/BodyHub/ServoPositions`
 发布机器人关节舵机的当前角度。此话题在初始化的时候创建，话题句柄`ServoPositionPub`，当节点收到控制舵机的消息或部分改变状态的消息，改变节舵机的角度，会将新的角度数组通过此话题发布。消息定义在`ServoPositionAngle.msg`文件，格式如下：
    ```
    float64[] angle
    ```

- `MediumSize/BodyHub/SensorRaw`
 发布传感器原始数据。此话题在初始化的时候创建，话题句柄`SensorRawDataPub`，当传感器列表中有传感器注册时，此话题会发布节点获取的注册传感器的原始数据。消息定义在`SensorRawData.msg`文件，格式如下：
    ```
    uint8[]   sensorReadID
    uint8     sensorCount
    uint16[]  sensorStartAddress
    uint16[]  sensorReadLength
    int32[]   sensorData
    uint8     dataLength
    ```
##### 订阅话题
- `MediumSize/BodyHub/MotoPosition`
 接收关节控制消息。此话题在`QueueThread`进程中订阅，话题句柄`MotoPositionSub`，在`QueueThread`进程中接收消息，当节点接收到此话题消息时，会获取消息附带的关节舵机的控制数据，并将数据放到关节舵机的控制数据队列。消息定义在`JointControlPoint.msg`文件，格式如下：
    ```
    float64[] positions
    float64[] velocities
    float64[] accelerations
    float64[] effort
    duration  time_from_start
    uint16    mainControlID
    ```

- `MediumSize/BodyHub/HeadPosition`
接收头部舵机控制消息。此话题在`QueueThread`进程中订阅，话题句柄`HeadPositionSub`，在`QueueThread`进程中接收消息，当节点接收到此话题消息时，会获取消息附带的头部舵机的控制数据，并将数据放到头部舵机的控制数据队列。消息定义在`JointControlPoint.msg`文件，格式如下：
    ```
    float64[] positions
    float64[] velocities
    float64[] accelerations
    float64[] effort
    duration  time_from_start
    uint16    mainControlID
    ```
- `/SensorControl`
接收执行器控制消息。此话题在初始化时订阅，话题句柄`SensorControlSub`，在主函数中接收消息，当节点接收到此话题消息时，会获取消息附带的执行器的控制数据，并将数据放到执行器的控制数据队列。消息定义在`SensorControl.msg`文件，格式如下：
    ```
    string SensorName
    uint16 SetAddr
    uint8[] Paramete
    ```
### 服务
##### 服务列表
- `MediumSize/BodyHub/StateJump`
 将节点设置为占用状态或释放节点，服务类型定义在`SrvState.srv`文件，格式如下：
    ```
    uint8 masterID
    string stateReq
    ---
    int16 stateRes
    ```
- `MediumSize/BodyHub/GetStatus`
 获取节点状态字符，服务类型定义在`SrvString.srv`文件，格式如下：
    ```
    string str
    ---
    string data
    ```
- `MediumSize/BodyHub/GetMasterID`
 获取节点占用id，服务类型定义在`SrvTLSstring.srv`文件，格式如下：
    ```
    string str
    ---
    uint8 data
    ```
- `MediumSize/BodyHub/GetJointAngle`
 获取关节舵机角度数据，服务类型定义在`SrvServoAllRead.srv`文件，格式如下：
    ```
    uint8[] idArray
    uint8 idCnt
    ---
    float64[] getData
    ```
- `MediumSize/BodyHub/RegistSensor`
 注册需要节点获取并发布数据的传感器，服务类型定义在`SrvInstWrite.srv`文件，格式如下：
    ```
    string itemName
    uint8 dxlID
    float64 setData
    ---
    bool complete
    ```
- `MediumSize/BodyHub/DeleteSensor`
 删除已经注册需要获取和发布数据的传感器，服务类型定义在`SrvTLSstring.srv`文件，格式如下：
    ```
    string str
    ---
    uint8 data
    ```

以下列表为`Dynamixel`协议功能的实现，请读者先熟悉`Dynamixel`的[Protocol 1.0](http://emanual.robotis.com/docs/en/dxl/protocol1/)协议
- `MediumSize/BodyHub/DirectMethod/InstRead`
实现`Protocol 1.0`协议的`Read`功能，服务类型定义在`SrvInstRead.srv`文件，格式如下：
    ```
    string itemName
    uint8 dxlID
    ---
    float64 getData
    ```
- `MediumSize/BodyHub/DirectMethod/InstWrite`
实现`Protocol 1.0`协议的`Wrte`功能，服务类型定义在`SrvInstWrite.srv`文件，格式如下：
    ```
    string itemName
    uint8 dxlID
    float64 setData
    ---
    bool complete
    ```
- `MediumSize/BodyHub/DirectMethod/SyncWrite`
实现`Protocol 1.0`协议的`SyncWrite`功能，服务类型定义在`SrvSyncWrite.srv`文件，格式如下：
    ```
    string itemName
    uint8[] idArray
    uint8 idCnt
    float64[] setData
    ---
    bool complete
    ```

以下列表为`Dynamixel`的`mx-28`舵机控制协议的实现，请读者先熟悉`Dynamixel`的[mx-28](http://emanual.robotis.com/docs/en/dxl/mx/mx-28/)舵机控制协议
- `MediumSize/BodyHub/DirectMethod/SetServoLockState`
实现`mx-28`协议的舵机的`torque`控制功能，服务类型定义在`SrvServoWrite.srv`文件，格式如下：
    ```
    uint8 dxlID
    float64 setData
    ---
    bool complete
    ```
- `MediumSize/BodyHub/DirectMethod/GetServoLockStateAll`
实现`mx-28`协议的多个舵机的`torque`控制功能，服务类型定义在`SrvServoAllWrite.srv`文件，格式如下：
    ```
    uint8[] idArray
    uint8 idCnt
    float64[] setData
    ---
    bool complete
    ```
- `MediumSize/BodyHub/DirectMethod/GetServoLockStateAll`
实现`mx-28`协议的多个舵机的`torque`获取功能，服务类型定义在`SrvServoAllRead.srv`文件，格式如下：
    ```
    uint8[] idArray
    uint8 idCnt
    ---
    float64[] getData
    ```
- `MediumSize/BodyHub/DirectMethod/SetServoTarPosition`
实现`mx-28`协议的舵机的`Goal_Position`数据设置功能，服务类型定义在`SrvServoWrite.srv`文件，格式如下：
    ```
    uint8 dxlID
    float64 setData
    ---
    bool complete
    ```
- `MediumSize/BodyHub/DirectMethod/SetServoTarPositionAll`
实现`mx-28`协议的多个舵机的`Goal_Position`数据设置功能，服务类型定义在`SrvServoAllWrite.srv`文件，格式如下：
    ```
    uint8[] idArray
    uint8 idCnt
    float64[] setData
    ---
    bool complete
    ```
- `MediumSize/BodyHub/DirectMethod/GetServoPositionAll`
实现`mx-28`协议的多个舵机的`Present_Position`数据获取功能，服务类型定义在`SrvServoAllRead.srv`文件，格式如下：
    ```
    uint8[] idArray
    uint8 idCnt
    ---
    float64[] getData
    ```

[^脚注1]:状态详情请参考`BodyHub状态机`文档