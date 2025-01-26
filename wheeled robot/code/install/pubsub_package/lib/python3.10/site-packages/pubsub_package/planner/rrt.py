#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RRT*
"""

import random
import math
import time
import numpy as np
from scipy.spatial import KDTree


class RRT:
    def __init__(self, minx, maxx, miny, maxy, obstacles, robot_size, safe_dist):
        """
        初始化 RRT 规划器

        :param minx: 环境最小 x 坐标
        :param maxx: 环境最大 x 坐标
        :param miny: 环境最小 y 坐标
        :param maxy: 环境最大 y 坐标
        :param obstacles: 障碍物列表 [(ox1, oy1), (ox2, oy2), ...]
        :param robot_size: 机器人大小 (半径)
        :param safe_dist: 安全距离
        """
        self.minx = minx
        self.maxx = maxx
        self.miny = miny
        self.maxy = maxy
        self.obstacles = obstacles
        self.robot_size = robot_size
        self.safe_dist = safe_dist
        self.tree = None
        self.node_list = []

    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.parent = None

    def plan(self, start_x, start_y, goal_x, goal_y, max_iter=500, step_size=1.0):
        """
        规划路径

        :param start_x: 起点 x 坐标
        :param start_y: 起点 y 坐标
        :param goal_x: 目标 x 坐标
        :param goal_y: 目标 y 坐标
        :param max_iter: 最大迭代次数
        :param step_size: 每次扩展的步长
        :return: (是否成功找到路径, 路径点列表)
        """
        self.node_list = [self.Node(start_x, start_y)]
        self.tree = KDTree([(start_x, start_y)])

        for i in range(max_iter):
            rand_x, rand_y = self.sample()
            nearest_node = self.get_nearest_node(rand_x, rand_y)
            new_node = self.steer(nearest_node, rand_x, rand_y, step_size)

            if not self.check_collision(new_node.x, new_node.y, nearest_node.x, nearest_node.y):
                continue

            self.node_list.append(new_node)
            self.tree = KDTree([(node.x, node.y) for node in self.node_list])

            if self.is_goal_reached(new_node, goal_x, goal_y, step_size):
                return True, self.get_final_path(new_node)

        return False, []

    def sample(self):
        """
        随机采样点
        :return: (x, y)
        """
        x = random.uniform(self.minx, self.maxx)
        y = random.uniform(self.miny, self.maxy)
        return x, y

    def get_nearest_node(self, x, y):
        """
        获取最近的树节点
        :param x: 目标 x 坐标
        :param y: 目标 y 坐标
        :return: 最近节点
        """
        _, index = self.tree.query((x, y))
        return self.node_list[index]

    def steer(self, from_node, to_x, to_y, step_size):
        """
        扩展树
        :param from_node: 起始节点
        :param to_x: 目标 x 坐标
        :param to_y: 目标 y 坐标
        :param step_size: 扩展步长
        :return: 新节点
        """
        angle = math.atan2(to_y - from_node.y, to_x - from_node.x)
        new_x = from_node.x + step_size * math.cos(angle)
        new_y = from_node.y + step_size * math.sin(angle)

        new_node = self.Node(new_x, new_y)
        new_node.parent = from_node
        return new_node

    def check_collision(self, x1, y1, x2, y2):
        """
        检查路径是否与障碍物碰撞
        :param x1: 起点 x 坐标
        :param y1: 起点 y 坐标
        :param x2: 终点 x 坐标
        :param y2: 终点 y 坐标
        :return: 是否安全
        """
        for ox, oy in self.obstacles:
            dist = self.point_to_line_distance(ox, oy, x1, y1, x2, y2)
            if dist < self.robot_size + self.safe_dist:
                return False
        return True

    def point_to_line_distance(self, px, py, x1, y1, x2, y2):
        """
        计算点到线段的最短距离
        :param px: 点 x 坐标
        :param py: 点 y 坐标
        :param x1: 线段起点 x 坐标
        :param y1: 线段起点 y 坐标
        :param x2: 线段终点 x 坐标
        :param y2: 线段终点 y 坐标
        :return: 最短距离
        """
        if (x2 - x1) == 0 and (y2 - y1) == 0:
            return math.hypot(px - x1, py - y1)

        t = max(0, min(1, ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) /
                       ((x2 - x1) ** 2 + (y2 - y1) ** 2)))
        proj_x = x1 + t * (x2 - x1)
        proj_y = y1 + t * (y2 - y1)
        return math.hypot(px - proj_x, py - proj_y)

    def is_goal_reached(self, node, goal_x, goal_y, threshold):
        """
        检查目标是否到达
        :param node: 当前节点
        :param goal_x: 目标 x 坐标
        :param goal_y: 目标 y 坐标
        :param threshold: 到达目标的阈值
        :return: 是否到达目标
        """
        dist = math.hypot(node.x - goal_x, node.y - goal_y)
        return dist < threshold

    def get_final_path(self, node):
        """
        获取最终路径
        :param node: 当前节点
        :return: 路径点列表
        """
        path = []
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]

# class RRT_star:
#    ''''''
#    ''''''

class Node:
    def __init__(self, x, y, cost, parent):
        """
        Define the nodes of tree

        :param x: the x coordinate of node
        :param y: the y coordinate of node
        :param parent: the sequence number of the parent node (if none, set to -1)
        """
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent
