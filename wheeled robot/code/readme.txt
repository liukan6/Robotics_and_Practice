ssh ubuntu@192.168.xx.x
./configure_discovery.sh

192.168.12.3
# 192.168.12.3
28


/home/aaa/Robotics/ros2_ws/src/pubsub_package


1、导航框架已经实现(global.py+local.py),请实现补全全局规划算法（例如rrt*）、局部规划算法（例如dwa）并将其放在planner目录下。
2、进入ros2_ws，编译：
colcon build

rm -rf build log install

source ./install/setup.bash

ros2 run pubsub_package global_pub

ros2 run pubsub_package local_pub

ros2 launch turtlebot4_navigation localization.launch.py map:=/home/aaa/map.yaml 

输出：
> [lifecycle_manager-3] [INFO] [1734860624.712362293] [lifecycle_manager_localization]: Server amcl connected with bond.
[lifecycle_manager-3] [INFO] [1734860624.712518623] [lifecycle_manager_localization]: Managed nodes are active
[lifecycle_manager-3] [INFO] [1734860624.712549671] [lifecycle_manager_localization]: Creating bond timer...
下面应该要please set the initial pose, 
otherwise it cannt work

膨胀地图
ros2 launch turtlebot4_navigation nav2.launch.py​​ ​

可视化
ros2 launch turtlebot4_viz view_robot.launch.py 


7、将话题名称改为/course_agv/goal，方法见图1-4
