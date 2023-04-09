# JsonFileBased_RobotDecision

基于Json文件解析的行为树，为机器人宏观决策设计

基于Json文件的机器人决策系统（JsonFileBased RobotDecision），为RoboMaster参赛机器人宏观决策设计。通过修改指定Json文件，快速修改机器人运行逻辑并约束移动区域，提供高自由度、高定制化的机器人决策逻辑定制功能。带有可视化GUI，可直观查看机器人决策状态。另有虚假消息发布者，以模拟虚拟环境来提供离线调试功能。程序运行在ROS2(Robot Operating System 2) Galactic框架下。

# Version：1.1a

# **节点（Node） 介绍：**

## robot_decision:

ros2 run robot_decision robot_decision_node

### 订阅：

| 话题                                    | 消息                                                           | 描述                  |
| --------------------------------------- | -------------------------------------------------------------- | --------------------- |
| /obj_hp                                 | global_interface::msg::ObjHP                                   | 场上兵种/设施血量     |
| /car_pos                                | global_interface::msg::CarPos                                  | 场上各兵种位置        |
| /game_info                              | global_interface::msg::GameInfo                                | 比赛相关信息          |
| /serial_msg                             | global_interface::msg::Serial                                  | 车辆模式、弹速等信息  |
| /joint_states                           | sensor_msgs::msg::JointState                                   | 云台yaw、pitch轴角度  |
| perception_detector/perception_array    | global_interface::msg::DetectionArray                          | 感知识别信息          |
| navigate_through_poses/_action/feedback | nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage | Nav2 Action反馈信息   |
| navigate_through_poses/_action/status   | action_msgs::msg::GoalStatusArray                              | Nav2 Action Goal 状态 |

### 发布：

| 话题                    | 消息                            | 描述             |
| ----------------------- | ------------------------------- | ---------------- |
| robot_decision/aim_yaw  | std_msgs::msg::Float32          | 车辆目标朝向偏角 |
| robot_decision/decision | global_interface::msg::Decision | 车辆当前决策信息 |

### Action:

| Name                   | Action                                  | 描述                 |
| ---------------------- | --------------------------------------- | -------------------- |
| navigate_through_poses | nav2_msgs::action::NavigateThroughPoses | 发送Nav2路径导航动作 |

### 参数：

| Name               | 类型  | 描述                 |
| ------------------ | ----- | -------------------- |
| distance_thr       | float | 路径点计算距离阈值   |
| seek_thr           | float | 感知索敌距离阈值     |
| IsRed              | bool  | 红蓝方标志位         |
| IfShowUI           | bool  | 是否显示可视化GUI    |
| SelfIndex          | int   | 车辆自身ID（索引）   |
| friendOutPostIndex | int   | 我方前哨站ID（索引） |

## fake_msg_publisher:

ros2 run fake_msg_publisher fake_msg_publisher_node

### 发布：

| 话题        | 消息                            | 描述                 |
| ----------- | ------------------------------- | -------------------- |
| /obj_hp     | global_interface::msg::ObjHP    | 场上兵种/设施血量    |
| /car_pos    | global_interface::msg::CarPos   | 场上各兵种位置       |
| /game_info  | global_interface::msg::GameInfo | 比赛相关信息         |
| /serial_msg | global_interface::msg::Serial   | 车辆模式、弹速等信息 |

# 节点关系图

![1681056559470](image/README/1681056559470.png)

# 项目结构

├── images		//图片目录
├── LICENSE		//开源协议
├── README.md	//项目自述文件
└── src			//项目源码目录
    ├── fake_msg_publisher							//假消息发布者功能包目录
    │   ├── fake_msg_publisher
    │   │   ├── fake_msg_publisher_node.py				//节点源码
    │   │   ├── init.py
    │   │   └── pycache								//python缓存
    │   │       ├── fake_msg_publisher_node.cpython-38.pyc
    │   │       └── init.cpython-38.pyc
    │   ├── package.xml								//功能包依赖xml文件
    │   ├── resource									//功能包资源文件
    │   │   └── fake_msg_publisher
    │   ├── setup.cfg									//ROS2 setup.cfg
    │   ├── setup.py									//ROS2 setup.py
    │   └── test
    │       ├── test_copyright.py
    │       ├── test_flake8.py
    │       └── test_pep257.py
    ├── global_interface			//定义全局通用消息文件
    │   ├── CMakeLists.txt			//功能包CMakeLists.txt
    │   ├── msg					//消息目录
    │   │   ├── CarPos.msg			//车辆位置消息
    │   │   ├── Decision.msg		//决策消息
    │   │   ├── DetectionArray.msg	//感知识别消息
    │   │   ├── Detection.msg		//自瞄识别消息
    │   │   ├── GameInfo.msg		//比赛信息消息
    │   │   ├── ObjHP.msg			//血量消息
    │   │   ├── Point2f.msg			//2D点消息
    │   │   └── Serial.msg			//下位机通讯消息
    │   └── package.xml			//功能包依赖xml文件
    └── robot_decision				//决策功能包
        ├── CMakeLists.txt				//功能包CMakeLists.txt
        ├── include					//头文件目录
        │   ├── Json					//Json处理库头文件
        │   │   ├── json-forwards.h
        │   │   └── json.h
        │   ├── robot_decision			//决策系统头文件目录
        │   │   ├── configs.h				//决策系统固定参数设定文件
        │   │   ├── public.h				//公共头文件
        │   │   ├── RobotDecision.h		//决策系统头文件
        │   │   └── structs.h				//结构体定义
        │   └── robot_decision_node.hpp	//决策节点头文件
        ├── launch					//launch目录
        │   └── decision_node_launch.py	//带参数节点启动launch文件
        ├── package.xml				//功能包依赖xml文件
        ├── resources					//资源目录
        │   └── RMUL.png				//RMUL赛场障碍图
        ├── sample					//Json样例目录
        │   ├── decisions.json			//决策Json样例
        │   └── waypoints.json			//路径点Json样例
        └── src						//源码目录
            ├── Json					//Json处理库源码
            │   └── jsoncpp.cpp
            ├── robot_decision			//决策系统源码
            │   └── RobotDecision.cpp
            └── robot_decision_node.cpp	//决策节点源码
