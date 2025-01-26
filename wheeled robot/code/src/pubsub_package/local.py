#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
sys.path.append("/home/aaa/Robotics/ros2_ws/src/pubsub_package/")
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose,PoseStamped
from scipy.spatial.transform import Rotation as R  # 从scipy导入Rotation类
import math
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import  Twist

from sensor_msgs.msg import LaserScan
from threading import Lock, Thread
# import planner
# Import your own planner
# from planner.dwa import DWA
# import DWA

class LocalPlanner(Node):
    def __init__(self, real):
        super().__init__('local_planner')
        self.real = real
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vw = 0.0
        self.path = Path()
        self.arrive = 0.2  # Standard for arrival
        self.threshold = 1.5  # Laser threshold
        self.robot_size = 0.2
        self.V_X = 0.5
        self.V_W = 0.5

        # self.planner = DWA()  # Initialize planner
        # self.planner.config(max_speed=self.V_X, max_yawrate=self.V_W, base=self.robot_size)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_sub = self.create_subscription(Path, '/course_agv/global_path', self.path_callback, 1)
        self.midpose_pub = self.create_publisher(PoseStamped, '/course_agv/mid_goal', 1)
        if self.real:
            self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 1)  # Real robot
            self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)  # Real robot
        else:
            self.laser_sub = self.create_subscription(LaserScan, '/course_agv/laser/scan', self.laser_callback, 1)  # Simulation
            self.vel_pub = self.create_publisher(Twist, '/course_agv/velocity', 1)  # Simulation

        self.planning_thread = None
        self.lock = Lock()
        self.laser_lock = Lock()

        self.traj_pub = self.create_publisher(Path, '/course_agv/trajectory', 1)
        self.traj = Path()
        self.traj.header.frame_id = 'map'

    def path_callback(self, msg):
        self.lock.acquire()
        self.path = msg
        self.update_global_pose(init=True)
        self.lock.release()
        if self.planning_thread is None:
            self.planning_thread = Thread(target=self.plan_thread_func)
            self.planning_thread.start()

    def plan_thread_func(self):
        self.get_logger().info("Running planning thread!")
        while True:
            self.lock.acquire()
            self.plan_once()
            self.lock.release()
            if self.goal_dis < self.arrive:
                self.lock.acquire()
                self.publish_velocity(zero=True)
                self.lock.release()
                self.get_logger().info("Arrived at goal!")
                break
        self.planning_thread = None
        self.get_logger().info("Exiting planning thread!")
        self.get_logger().info("----------------------------------------------------")

    def plan_once(self):
        self.plan_s = (0.0, 0.0, 0.0, self.vx, self.vw)
        self.update_global_pose(init=False)
        # self.update_obstacle()

        # u = self.planner.planning(
        #     pose=self.plan_s[:3], velocity=self.plan_s[3:], goal=self.plan_goal, points_cloud=self.plan_ob.tolist()
        # )

        print(self.plan_goal)
        # if isinstance(self.planner, DWA):
        # self.vx = max(min(u[0], self.V_X), -self.V_X)
        # self.vw = max(min(u[1], self.V_W), -self.V_W)
        # print("11111",self.V_X,self.vw,u[0])
        # # else:
        # dx, dy = u
        dx = self.plan_goal[0] - self.plan_s[0]
        dy = self.plan_goal[1] - self.plan_s[1]
        print(dx)
        print(dy)
        k_rho, k_alpha, k_beta = 0.6, 0.2, 0
        o_g = math.atan2(self.plan_goal[1], self.plan_goal[0])
        theta = 0
        rho = math.hypot(dx, dy)
        # alpha = (math.atan2(dy, dx) - theta + math.pi) % (2 * math.pi) - math.pi
        alpha = math.atan2(dy, dx) - theta 
        # alpha = normalize_angle(alpha)
        print("rho:",rho)
        print("alpha:",alpha)
        # beta = (o_g - theta - alpha + math.pi) % (2 * math.pi) - math.pi
        v = k_rho * rho
        w = k_alpha * alpha #+ k_beta * beta
        # if alpha < -math.pi / 2 or alpha > math.pi / 2:
        #     v = -v
        self.vx = max(min(v, self.V_X), -self.V_X)
        self.vw = max(min(w, self.V_W), -self.V_W)
        self.get_logger().info(f"v: {self.vx}, w: {self.vw}")
        self.publish_velocity(zero=False)




    def update_global_pose(self, init=False):
        try:
            # 根据是否为真实环境选择不同的坐标系
            source_frame = 'map' 
            target_frame = 'base_footprint'

            # 尝试获取坐标变换
            trans = self.tf_buffer.lookup_transform(
                'map' , 
                'base_footprint'              , 
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)  # 超时时间设置为1秒
            )

            # 更新位置信息
            self.x = trans.transform.translation.x
            self.y = trans.transform.translation.y

            # 使用scipy将四元数转换为欧拉角
            rotation = R.from_quat([
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ])
            # 提取欧拉角，单位为弧度
            roll, pitch, yaw = rotation.as_euler('xyz', degrees=False)
            self.yaw = yaw  # 这里只需要yaw角度

        except TransformException as e:
            # 捕获变换异常并记录错误日志
            self.get_logger().error(f"TF 错误: {e}")

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        self.traj.poses.append(pose)
        self.traj_pub.publish(self.traj)

        if init:
            self.goal_index = 1
            self.traj.poses = []
        for ind in range(len(self.path.poses) - 1, 0, -1):
            p = self.path.poses[ind].pose.position
            dis = math.hypot(p.x - self.x, p.y - self.y)
            if dis < self.robot_size:
                self.goal_index = min(max(ind + 1, self.goal_index), len(self.path.poses) - 1)
        goal = self.path.poses[self.goal_index]
        self.midpose_pub.publish(goal)
        try:
            # print(goal)
            lgoal = self.tf_buffer.transform(goal, 'base_footprint') # 超时时间设置为1秒)
            # lgoal = do_transform_pose(goal, trans) 
            # print("lgoal")
            # print("lgoal.pose.position.x:",lgoal.pose.position.x)
            self.plan_goal = (lgoal.pose.position.x, lgoal.pose.position.y)
        except Exception as e:
            self.get_logger().error(f"TF transformation error: {e}")
        self.goal_dis = math.hypot(self.x - self.path.poses[-1].pose.position.x,
                                   self.y - self.path.poses[-1].pose.position.y)

    def laser_callback(self, msg):
        self.laser_lock.acquire()
        self.ob = [(100, 100)]
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        for i, r in enumerate(msg.ranges):
            a = angle_min + angle_increment * i
            if r < self.threshold:
                self.ob.append((math.cos(a) * r, math.sin(a) * r))
        self.laser_lock.release()

    def update_obstacle(self):
        self.laser_lock.acquire()
        self.plan_ob = np.array(self.ob)
        self.laser_lock.release()

    def publish_velocity(self, zero=False):
        # if zero:
        #     self.vx = 0.1
        #     self.vw = 0.0
        cmd = Twist()
        cmd.linear.x = float(self.vx)
        cmd.angular.z = float(self.vw)
        self.vel_pub.publish(cmd)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def main(args=None):
    print("Start!")
    rclpy.init(args=args)
    real = True  # Set to your desired value
    local_planner = LocalPlanner(real)
    rclpy.spin(local_planner)
    local_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
