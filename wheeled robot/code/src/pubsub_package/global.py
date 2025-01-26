#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path, OccupancyGrid
from tf2_geometry_msgs import PoseStamped

import numpy as np
import sys
import time
import math
sys.path.append("/home/aaa/Robotics/ros2_ws/src/pubsub_package")
# import your own planner
# from planner.rrt_star import RRT_star as planner
from planner.rrt import RRT as planner

class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner')
        self.map = None
        self.plan_robot_radius = 0.15
        self.plan_ox = []  # obstacle
        self.plan_oy = []
        self.plan_sx = 0.0  # start pose
        self.plan_sy = 0.0
        self.plan_gx = 0.0  # goal pose
        self.plan_gy = 0.0
        self.plan_rx = []  # plan
        self.plan_ry = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        self.goal_sub = self.create_subscription(PoseStamped, '/course_agv/goal', self.goal_callback, 1)
        self.path_pub = self.create_publisher(Path, '/course_agv/global_path', 1)
        # self.plan_srv = self.create_service(Plan, '/course_agv/global_plan', self.replan)

        # self.update_map()

    def update_map(self):
        # # client = self.create_client(GetMap, '/static_map')
        # while not client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for /static_map service...')
        try:
            request = GetMap.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result():
                self.map_callback(future.result().map)
            else:
                self.get_logger().error('Failed to get map')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def map_callback(self, msg):
        self.map = msg

    def goal_callback(self, msg):
        self.plan_gx = msg.pose.position.x
        self.plan_gy = msg.pose.position.y
        self.get_logger().info("Received new goal!")
        self.replan()

    def replan(self):
        self.get_logger().info('Replanning...')
        self.init_planner()
        self.update_global_pose()

        res = False
        attempt = 10
        for _ in range(attempt):
            is_found, path = self.planner.plan(self.plan_sx, self.plan_sy, self.plan_gx, self.plan_gy)
            if is_found:
                plan_path = self.insert_midpoints(path)
                self.plan_rx = np.array(plan_path)[:, 0]
                self.plan_ry = np.array(plan_path)[:, 1]
                self.publish_path()
                res = True
                self.get_logger().info("Path found!")
                break
            else:
                self.get_logger().info("Retry...")

        if not res:
            self.get_logger().error("Path not found!")



    def insert_midpoints(self, path):
        plan_path = []
        for i in range(len(path) - 1):
            plan_path.append(path[i])
            x, y = path[i]
            dx = path[i + 1][0] - x
            dy = path[i + 1][1] - y
            angle = math.atan2(dy, dx)
            dist = math.hypot(dx, dy)
            step_size = self.plan_robot_radius
            steps = int(round(dist / step_size))
            for _ in range(steps - 1):
                x += step_size * math.cos(angle)
                y += step_size * math.sin(angle)
                plan_path.append((x, y))
        plan_path.append(path[-1])
        return plan_path

    def init_planner(self):
      
        map_data = np.array(self.map.data).reshape((self.map.info.height, -1)).transpose()  # 实物
      
        minx = self.map.info.origin.position.x
        maxx = self.map.info.origin.position.x + map_data.shape[0] * self.map.info.resolution
        miny = self.map.info.origin.position.y
        maxy = self.map.info.origin.position.y + map_data.shape[1] * self.map.info.resolution
      
        ox, oy = np.nonzero(map_data != 0)  # 实物
    
        self.plan_ox = ox * self.map.info.resolution + self.map.info.origin.position.x
        self.plan_oy = oy * self.map.info.resolution + self.map.info.origin.position.y
        obstacles = list(zip(self.plan_ox, self.plan_oy))
        obstacles.append((-9999, -9999))
        self.planner = planner(minx=minx, maxx=maxx, miny=miny, maxy=maxy, obstacles=obstacles,
                               robot_size=self.plan_robot_radius, safe_dist=self.map.info.resolution * 4)

    def update_global_pose(self):
        try:
          
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time(), rclpy.duration.Duration(seconds=4.0))
            
            self.plan_sx = trans.transform.translation.x
            self.plan_sy = trans.transform.translation.y
        except Exception as e:
            self.get_logger().error(f"TF error: {e}")

    def publish_path(self):
        path = Path()
        path.header.stamp = rclpy.time.Time().to_msg()
        path.header.frame_id = 'map'
        for i in range(len(self.plan_rx)):
            pose = PoseStamped()
            pose.header.stamp = rclpy.time.Time().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = self.plan_rx[i]
            pose.pose.position.y = self.plan_ry[i]
            pose.pose.position.z = 0.01
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)
        


def main(args=None):
    print("Start!")
    rclpy.init(args=args)
    global_planner = GlobalPlanner()
    rclpy.spin(global_planner)
    global_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
