#!/usr/bin/env python3
"""
A*路径规划节点

功能：
- 订阅map2、kfs_data和trigger话题
- 使用A*算法规划从[0][1]到[3][1]的路径
- 基于KFS网格判定障碍物和移动代价
- 发布规划路径到/planning/path

障碍物规则：
- kfs=3（假的kfs）或kfs=1（r1的kfs）: 障碍物，不可通过
- kfs=2（r2的kfs）: cost=2
- kfs=0（无kfs）: cost=1
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import json
import numpy as np
import heapq


class AStarPlannerNode(Node):
    """A*路径规划节点"""
    
    def __init__(self):
        super().__init__('astar_planner')
        
        # 网格配置
        self.grid_rows = 4
        self.grid_cols = 3
        
        # Map2配置（用于坐标转换）
        self.map2_origin = [3.2, 1.2, 0.0]  # [x, y, theta]
        self.grid_resolution = 1.2  # 米/单元格
        
        # 固定起点和终点（grid索引，row和col）
        self.start_grid = [0, 1]  # [row, col]
        self.goal_grid = [3, 1]   # [row, col]
        
        # 数据标志
        self.has_map2 = False
        self.has_kfs_data = False
        self.trigger_received = True
        
        # 存储KFS网格数据
        self.kfs_grid = np.zeros((self.grid_rows, self.grid_cols), dtype=int)
        
        # 配置QoS（使用TRANSIENT_LOCAL以接收初始消息）
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # 创建订阅者
        self.map2_sub = self.create_subscription(
            OccupancyGrid,
            '/map2',
            self.map2_callback,
            qos_profile
        )
        
        self.kfs_data_sub = self.create_subscription(
            String,
            '/kfs_grid_data',
            self.kfs_data_callback,
            qos_profile
        )
        
        self.trigger_sub = self.create_subscription(
            String,
            '/task/trigger',
            self.trigger_callback,
            10
        )
        
        # 创建发布者（使用TRANSIENT_LOCAL QoS）
        path_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_pub = self.create_publisher(
            Path,
            '/planning/path',
            path_qos
        )
        
        self.get_logger().info('A* Planner Node started')
        self.get_logger().info(f'Start: {self.start_grid}, Goal: {self.goal_grid}')
    
    def map2_callback(self, msg):
        """处理map2消息（仅用于触发，实际不使用地图数据）"""
        self.has_map2 = True
        self.get_logger().info('Received map2')
        self.check_and_plan()
    
    def kfs_data_callback(self, msg):
        """处理kfs_grid_data消息"""
        try:
            grid_data = json.loads(msg.data)
            grid = np.array(grid_data['grid'])
            
            if grid.shape != (self.grid_rows, self.grid_cols):
                self.get_logger().warn(f'Invalid grid dimensions: {grid.shape}')
                return
            
            self.kfs_grid = grid
            self.has_kfs_data = True
            
            self.get_logger().info(f'Received kfs_data:\n{grid}')
            self.check_and_plan()
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing kfs_data: {e}')
    
    def trigger_callback(self, msg):
        """处理trigger消息"""
        self.trigger_received = True
        self.get_logger().info('Received trigger signal')
        self.check_and_plan()
    
    def check_and_plan(self):
        """检查是否所有数据都准备好，如果是则执行规划"""
        if self.has_map2 and self.has_kfs_data and self.trigger_received:
            self.get_logger().info('All data ready, starting A* planning...')
            path = self.plan_path()
            if path is not None:
                self.publish_path(path)
            else:
                self.get_logger().warn('Path planning failed')
            # 重置trigger标志，避免重复规划
            self.trigger_received = False
    
    def is_obstacle(self, row, col):
        """
        检查grid坐标是否为障碍物
        
        Args:
            row, col: grid坐标
            
        Returns:
            bool: True表示障碍物，False表示可通过
        """
        if row < 0 or row >= self.grid_rows or col < 0 or col >= self.grid_cols:
            return True
        
        kfs_value = self.kfs_grid[row, col]
        # kfs=3（假的kfs）或kfs=1（r1的kfs）视为障碍物
        return kfs_value == 3 or kfs_value == 1
    
    def get_cost(self, row, col):
        """
        获取移动到该格子的代价
        
        Args:
            row, col: grid坐标
            
        Returns:
            float: 移动代价（障碍物返回inf）
        """
        if self.is_obstacle(row, col):
            return float('inf')
        
        kfs_value = self.kfs_grid[row, col]
        if kfs_value == 2:  # r2的kfs
            return 2.0
        else:  # kfs=0，无kfs
            return 1.0
    
    def heuristic(self, row1, col1, row2, col2):
        """
        启发式函数：曼哈顿距离（绝对值边长相加）
        
        Args:
            row1, col1: 起点坐标
            row2, col2: 终点坐标
            
        Returns:
            float: 曼哈顿距离
        """
        return abs(row1 - row2) + abs(col1 - col2)
    
    def get_neighbors(self, row, col):
        """
        获取四联通邻居（上下左右）
        
        Args:
            row, col: 当前坐标
            
        Returns:
            list: [(row, col), ...] 邻居坐标列表
        """
        neighbors = []
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # 上下左右
            new_row = row + dr
            new_col = col + dc
            if 0 <= new_row < self.grid_rows and 0 <= new_col < self.grid_cols:
                neighbors.append((new_row, new_col))
        return neighbors
    
    def plan_path(self):
        """
        执行A*路径规划
        
        Returns:
            list: [(row, col), ...] 路径点列表，失败返回None
        """
        start_row, start_col = self.start_grid
        goal_row, goal_col = self.goal_grid
        
        # 检查起点和终点
        if self.is_obstacle(start_row, start_col):
            self.get_logger().error(f'Start [{start_row}, {start_col}] is obstacle!')
            return None
        
        if self.is_obstacle(goal_row, goal_col):
            self.get_logger().error(f'Goal [{goal_row}, {goal_col}] is obstacle!')
            return None
        
        # A*算法
        open_set = []  # (f_score, row, col)
        closed_set = set()  # 已处理的节点集合
        came_from = {}  # (row, col) -> (row, col)
        g_score = {}  # (row, col) -> float
        
        start_key = (start_row, start_col)
        goal_key = (goal_row, goal_col)
        
        g_score[start_key] = 0.0
        f_score = self.heuristic(start_row, start_col, goal_row, goal_col)
        
        heapq.heappush(open_set, (f_score, start_row, start_col))
        
        while open_set:
            current_f, current_row, current_col = heapq.heappop(open_set)
            current_key = (current_row, current_col)
            
            # 检查是否已经在closed_set中（避免重复处理）
            if current_key in closed_set:
                continue
            
            # 将当前节点加入closed_set
            closed_set.add(current_key)
            
            # 检查是否到达目标
            if current_row == goal_row and current_col == goal_col:
                # 重构路径
                path = []
                while current_key in came_from:
                    path.append((current_key[0], current_key[1]))
                    current_key = came_from[current_key]
                path.append((start_row, start_col))
                path.reverse()
                
                self.get_logger().info(f'Path found with {len(path)} points')
                return path
            
            # 检查邻居
            for neighbor_row, neighbor_col in self.get_neighbors(current_row, current_col):
                neighbor_key = (neighbor_row, neighbor_col)
                
                # 跳过已处理的节点
                if neighbor_key in closed_set:
                    continue
                
                # 获取移动代价
                move_cost = self.get_cost(neighbor_row, neighbor_col)
                if move_cost == float('inf'):
                    continue  # 跳过障碍物
                
                tentative_g = g_score.get(current_key, float('inf')) + move_cost
                
                # 如果找到更短的路径，更新
                if neighbor_key not in g_score or tentative_g < g_score[neighbor_key]:
                    came_from[neighbor_key] = current_key
                    g_score[neighbor_key] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor_row, neighbor_col, goal_row, goal_col)
                    heapq.heappush(open_set, (f_score, neighbor_row, neighbor_col))
        
        self.get_logger().warn('No path found!')
        return None
    
    def grid_to_map_coords(self, row, col):
        """
        将grid索引转换为map坐标
        
        Args:
            row, col: grid索引
            
        Returns:
            tuple: (x, y) map坐标
        """
        x = self.map2_origin[0] + row * self.grid_resolution + 0.5 * self.grid_resolution
        y = self.map2_origin[1] + col * self.grid_resolution + 0.5 * self.grid_resolution
        return x, y
    
    def publish_path(self, path):
        """
        发布路径（将grid索引转换为map坐标）
        
        Args:
            path: [(row, col), ...] 路径点列表（grid索引）
        """
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for row, col in path:
            # 将grid索引转换为map坐标
            x, y = self.grid_to_map_coords(row, col)
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published path with {len(path_msg.poses)} poses')


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
