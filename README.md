# JsonFileBased_RobotDecision

基于Json文件解析的行为树，为机器人宏观决策设计

基于Json文件的机器人决策系统（JsonFileBased RobotDecision），为RoboMaster参赛机器人宏观决策设计。通过修改指定Json文件，快速修改机器人运行逻辑并约束移动区域，提供高自由度、高定制化的机器人决策逻辑定制功能。带有可视化GUI，可直观查看机器人决策状态。另有虚假消息发布者，以模拟虚拟环境来提供离线调试功能。程序运行在ROS2(Robot Operating System 2) Galactic框架下。

# Version：1.1a

# **Node 启动方法：**

## fake_msg_publisher:

ros2 run fake_msg_publisher fake_msg_publisher_node

## robot_decision:

ros2 run robot_decision robot_decision_node
