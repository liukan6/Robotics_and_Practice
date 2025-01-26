# Robotics_and_Practice

This is the coursework for the Zhejiang University Zhu Kezhen College Engineering High School course **Robotics and Practice** , which is mainly divided into the following Parts.

![collection](image/README/collection.gif)

## Footed Robot

In this part, we first learned basic knowledge of robotics and how to use **SolidWorks** to build model of SCARA robot, then make motion simulation of another robotic arms using **CoppeliaSim.**

We learned the forward kinematics and inverse kinematics solutions for robots

![1737866349726](image/README/forward_kinematics.png "verify the forward kinematics")

![1737867302692](image/README/inverse_kinematics.png "the angles calculated by inverse kinematics")

The following video shows the simulation of a robotic arm gripping blocks and placing them into a specific shape after a specific motion.

![demo](image/README/Group2_demo.gif)

### space station robot arm

![1737869572791](image/README/1737869572791.png)

This is the final task as the end of footed robot part, we should first calculate the inverse kinematics, then simulate the motion in CoppeliaSim which is really hard because of the details.

![space_station_robot_arm_simulation](image/README/space_station_robot_arm_simulation.gif "space_station_robot_arm_simulation")

Considering the motion is too complex, we are only required to build the real robot and make it move a step.

![space_station_robot_arm.](image/README/space_station_robot_arm.gif "space_station_robot_arm")

## Wheeled Robot

We first installed Ubuntu, ROS2, Nav2, and configured Nav2, then completed Gazebo navigation simulation and visualization of TF, sensors, maps, etc. on RVIZ. The environmemt configuration was really a headache🤯

The following video shows we use the embedded navigation algorithm of Global Planner.

![embedded_algorithm](image/README/embedded_algorithm.gif "embedded_algorithm")

### RRT* Algorithm

The following video shows we use the RRT* algorithm written by ourselves.

![RRT_algorithm](image/README/RRT_algorithm.gif "RRT*_algorithm")
