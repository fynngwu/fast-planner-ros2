# Fast-Planner ROS2 运行说明

## 系统架构

Fast-Planner 系统包含以下节点：

1. **fast_planner_node**: 路径规划和轨迹优化节点
2. **traj_server_node**: 轨迹跟踪和控制命令发布节点
3. **odom_simulator**: 里程计仿真节点（可选，用于测试）

## 话题说明

### 输入话题

- **`/odom_world`** (nav_msgs/msg/Odometry)
  - 机器人当前位姿和速度信息
  - 由 odom_simulator 或实际里程计节点发布

- **`/map1`** (nav_msgs/msg/OccupancyGrid) - 必需
  - 占用栅格地图
  - 由您的地图发布节点发布
  - SdfNode 会订阅此话题来更新地图和 ESDF

- **`/waypoint_generator/waypoints`** (nav_msgs/msg/Path) - 可选
  - 路径点序列
  - 如果不提供，系统会使用配置文件中预设的路径点

### 输出话题

- **`/cmd_vel`** (geometry_msgs/msg/Twist)
  - 控制命令（线速度和角速度）
  - 由 traj_server_node 发布

- **`/planning/bspline`** (plan_manage/msg/Bspline)
  - B样条轨迹
  - 由 fast_planner_node 发布，traj_server_node 订阅

- **`/traj_server/trajectory`** (visualization_msgs/msg/Marker)
  - 轨迹可视化标记

- **`/traj_server/reference_path`** (nav_msgs/msg/Path)
  - 参考路径可视化

- **`/planning_vis/trajectory`** (visualization_msgs/msg/Marker)
  - 规划轨迹可视化

- **`/planning_vis/pos_cmd`** (visualization_msgs/msg/Marker)
  - 位置命令可视化

## 运行步骤

### 1. 确保工作空间已编译

```bash
cd /home/wufy/ros2_ws
colcon build --packages-select plan_manage
source install/setup.bash
```

### 2. 启动 Fast-Planner 节点

```bash
ros2 launch plan_manage kino_replan.launch.py
```

或者指定参数文件：

```bash
ros2 launch plan_manage kino_replan.launch.py params_file:=/path/to/your/params.yaml
```

### 3. 在 RViz2 中添加可视化

由于您已经打开了 RViz2，需要添加以下显示项：

#### 添加显示项：

1. **轨迹可视化**：
   - Type: `Marker`
   - Topic: `/traj_server/trajectory`
   - Color: 自定义

2. **参考路径**：
   - Type: `Path`
   - Topic: `/traj_server/reference_path`
   - Color: 绿色

3. **规划轨迹**：
   - Type: `Marker`
   - Topic: `/planning_vis/trajectory`
   - Color: 红色

4. **位置命令**：
   - Type: `Marker`
   - Topic: `/planning_vis/pos_cmd`
   - Color: 蓝色

5. **机器人位姿**（如果发布）：
   - Type: `TF` 或 `Odometry`
   - Topic: `/odom_world`

### 4. 检查系统状态

使用以下命令检查节点和话题：

```bash
# 查看所有节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看话题数据
ros2 topic echo /odom_world
ros2 topic echo /planning/bspline
ros2 topic echo /cmd_vel

# 查看节点信息
ros2 node info /fast_planner_node
ros2 node info /traj_server
```

## 配置说明

### 路径点配置

在 `config/fast_planner_params.yaml` 中配置预设路径点：

```yaml
fsm:
  target_type: 2  # 2 = PRESET_TARGET (预设目标模式)
  waypoint_num: 3  # 路径点数量
  waypoint0:
    x: 2.0
    y: 0.0
    z: 0.5
  waypoint1:
    x: 2.0
    y: 2.0
    z: 0.5
  waypoint2:
    x: 0.0
    y: 2.0
    z: 0.5
```

### 控制器参数

```yaml
traj_server:
  ros__parameters:
    controller.kp: 4.0  # 位置比例增益
    controller.kd: 1.5  # 速度微分增益
    publish_rate: 100.0  # 控制命令发布频率 (Hz)
```

## 故障排查

### 节点未启动

检查编译是否成功：
```bash
ros2 pkg list | grep plan_manage
```

### 话题未发布

检查节点是否正常运行：
```bash
ros2 node list
ros2 topic list
```

### 没有轨迹生成

1. 检查是否有 `odom_world` 话题数据
2. 检查路径点配置是否正确
3. 查看节点日志：
```bash
ros2 topic echo /fast_planner_node/parameter_events
```

### 可视化问题

确保 RViz2 中：
1. Fixed Frame 设置为 `world` 或 `map`
2. 所有显示项的 Topic 名称正确
3. 显示项已启用（勾选框）

## 测试建议

1. **首先测试 odom_simulator**：
   - 启动系统后，检查 `/odom_world` 话题是否有数据
   - 在 RViz2 中应该能看到机器人位姿

2. **测试路径规划**：
   - 等待系统自动规划路径（使用预设路径点）
   - 检查 `/planning/bspline` 话题是否有数据

3. **测试轨迹跟踪**：
   - 检查 `/cmd_vel` 话题是否有控制命令发布
   - 在 RViz2 中观察轨迹可视化

## 注意事项

- 系统需要地图信息才能进行路径规划，确保地图发布节点正在运行
- 如果使用外部里程计，确保话题名称为 `/odom_world`
- 控制命令发布在 `/cmd_vel` 话题，确保您的机器人控制器订阅此话题
- 系统使用预设路径点模式（target_type=2），会自动按顺序访问配置的路径点

