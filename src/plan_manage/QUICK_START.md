# Fast-Planner 快速启动指南

## 前提条件

1. ✅ 已编译所有包
2. ✅ 地图发布节点正在运行（发布 `/map1` 话题）
3. ✅ RViz2 已打开

## 启动步骤

### 1. Source 工作空间

```bash
cd /home/wufy/ros2_ws
source install/setup.bash
```

### 2. 启动 Fast-Planner 系统

```bash
ros2 launch plan_manage kino_replan.launch.py
```

这将启动以下节点：
- `fast_planner_node` - 路径规划节点
- `traj_server` - 轨迹跟踪节点
- `odom_simulator` - 里程计仿真节点

## 在 RViz2 中添加可视化

### 必需的可视化话题：

1. **轨迹可视化**
   - 类型：`Marker`
   - 话题：`/traj_server/trajectory`
   - 颜色：建议使用红色或绿色

2. **参考路径**
   - 类型：`Path`
   - 话题：`/traj_server/reference_path`
   - 颜色：建议使用蓝色

3. **规划轨迹**
   - 类型：`Marker`
   - 话题：`/planning_vis/trajectory`
   - 颜色：建议使用黄色

4. **位置命令**
   - 类型：`Marker`
   - 话题：`/planning_vis/pos_cmd`
   - 颜色：建议使用紫色

### 可选的可视化话题：

- `/planning_vis/topo_path` - 拓扑路径
- `/planning_vis/prediction` - 预测轨迹
- `/planning_vis/visib_constraint` - 可见性约束
- `/planning_vis/frontier` - 前沿点
- `/planning_vis/yaw` - 偏航角轨迹

## 检查系统状态

### 查看节点

```bash
ros2 node list
```

应该看到：
- `/fast_planner_node`
- `/traj_server`
- `/odom_simulator`

### 查看话题

```bash
ros2 topic list
```

关键话题：
- `/odom_world` - 里程计（应该有数据）
- `/map1` - 地图（应该有数据）
- `/planning/bspline` - B样条轨迹（规划后会有数据）
- `/cmd_vel` - 控制命令（跟踪时会有数据）

### 实时监控

```bash
# 监控里程计
ros2 topic echo /odom_world

# 监控控制命令
ros2 topic echo /cmd_vel

# 监控轨迹
ros2 topic echo /planning/bspline
```

## 系统工作流程

1. **初始化**：系统启动后，等待 `/odom_world` 和 `/map1` 话题数据
2. **路径规划**：收到里程计后，使用预设路径点进行路径规划
3. **轨迹优化**：生成 B样条轨迹并优化
4. **轨迹跟踪**：`traj_server` 接收轨迹并发布控制命令
5. **循环执行**：按顺序访问所有预设路径点

## 配置路径点

编辑 `config/fast_planner_params.yaml`：

```yaml
fsm:
  waypoint_num: 3  # 修改路径点数量
  waypoint0:
    x: 2.0  # 修改坐标
    y: 0.0
    z: 0.5
  # ... 更多路径点
```

## 故障排查

### 问题：没有轨迹生成

**检查**：
1. 是否有 `/odom_world` 数据？
   ```bash
   ros2 topic echo /odom_world --once
   ```

2. 是否有 `/map1` 数据？
   ```bash
   ros2 topic echo /map1 --once
   ```

3. 查看节点日志：
   ```bash
   ros2 node info /fast_planner_node
   ```

### 问题：没有控制命令

**检查**：
1. 是否有 `/planning/bspline` 话题数据？
   ```bash
   ros2 topic echo /planning/bspline --once
   ```

2. 检查 `traj_server` 节点状态：
   ```bash
   ros2 node info /traj_server
   ```

### 问题：可视化不显示

**检查**：
1. RViz2 Fixed Frame 是否设置为 `world` 或 `map`？
2. 话题名称是否正确？
3. 显示项是否已启用（勾选框）？

## 停止系统

按 `Ctrl+C` 停止 launch 文件启动的所有节点。

## 注意事项

- 确保地图发布节点正在运行并发布 `/map1` 话题
- 系统使用预设路径点模式，会自动按顺序访问路径点
- 控制命令发布在 `/cmd_vel` 话题
- 如果使用实际机器人，确保机器人控制器订阅 `/cmd_vel` 话题

