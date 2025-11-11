# KinodynamicAstar算法详细流程图

## 1. f、g、h值计算详解

### 1.1 g值计算 (实际代价)
```mermaid
flowchart TD
    A["起始节点 g=0"] --> B["状态传播"]
    B --> C["计算时间步长 τ"]
    C --> D["g_new = g_parent + τ"]
    D --> E["更新节点g值"]
```

**代码实现**:
```cpp
// 在状态传播过程中
double duration = tau;  // 时间步长
neighbor->g_score = cur_node->g_score + duration;
```

### 1.2 h值计算 (启发式估计)
```mermaid
flowchart TD
    A["当前状态"] --> B["目标状态"]
    B --> C["计算位置差 dp"]
    C --> D["计算速度差 dv"]
    D --> E["构建四次多项式"]
    E --> F["求解最优时间 t_optimal"]
    F --> G["h = λ||dp|| + w_time*t_optimal"]
    G --> H["返回h值"]
```

**代码实现**:
```cpp
double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time) {
    Eigen::Vector3d dp = x2.head(3) - x1.head(3);
    Eigen::Vector3d v0 = x1.tail(3);
    Eigen::Vector3d v1 = x2.tail(3);
    
    // 构建四次多项式系数
    double c1 = -36.0 * dp.dot(dp);
    double c2 = 24.0 * (v0 + v1).dot(dp);
    double c3 = -4.0 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
    double c4 = 0;
    double c5 = w_time_;
    
    // 求解最优时间
    std::vector<double> ts = quartic(c5, c4, c3, c2, c1);
    
    // 选择最小代价
    double cost = std::numeric_limits<double>::max();
    for (double t : ts) {
        if (t < 0) continue;
        double c = w_time_ * t + lambda_heu_ * dp.norm() / t;
        if (c < cost) {
            cost = c;
            optimal_time = t;
        }
    }
    
    return cost;
}
```

### 1.3 f值计算 (总代价)
```mermaid
flowchart TD
    A["g值"] --> C["f = g + h"]
    B["h值"] --> C
    C --> D["f值"]
    D --> E["用于OPEN集合排序"]
```

**代码实现**:
```cpp
// 在节点创建时
neighbor->f_score = neighbor->g_score + h_score;
```

## 2. 完整算法流程图

### 2.1 主搜索循环
```mermaid
flowchart TD
    A["开始搜索"] --> B["初始化起始节点"]
    B --> C["计算起始节点h值"]
    C --> D["f = g + h = 0 + h"]
    D --> E["加入OPEN集合"]
    E --> F{"OPEN集合为空?"}
    F -->|是| G["搜索失败<br/>返回NO_PATH"]
    F -->|否| H["取出f值最小节点"]
    H --> I["检查是否到达目标"]
    I -->|是| J["搜索成功<br/>返回REACH_END"]
    I -->|否| K["状态传播"]
    K --> L["碰撞检测"]
    L --> M["状态有效?"]
    M -->|否| N["丢弃状态"]
    M -->|是| O["计算h值"]
    O --> P["g = g_parent + τ"]
    P --> Q["f = g + h"]
    Q --> R["加入OPEN集合"]
    R --> F
    N --> F
```

### 2.2 状态传播详细过程
```mermaid
flowchart TD
    A["当前节点状态<br/>[x,y,z,vx,vy,vz]"] --> B["生成控制输入<br/>[ax,ay,az]"]
    B --> C["双积分器模型<br/>x(t+τ) = x(t) + v(t)τ + 0.5a(t)τ²<br/>v(t+τ) = v(t) + a(t)τ"]
    C --> D["计算下一状态<br/>[x',y',z',vx',vy',vz']"]
    D --> E["检查速度约束<br/>||v|| ≤ v_max"]
    E --> F["检查加速度约束<br/>||a|| ≤ a_max"]
    F --> G["约束满足?"]
    G -->|否| H["丢弃状态"]
    G -->|是| I["返回新状态"]
```

### 2.3 碰撞检测过程
```mermaid
flowchart TD
    A["新状态位置<br/>[x,y,z]"] --> B["ESDF距离查询<br/>getDistance(pos)"]
    B --> C["距离 > 安全阈值?"]
    C -->|否| D["碰撞<br/>返回false"]
    C -->|是| E["膨胀占用查询<br/>getInflateOccupancy(pos)"]
    E --> F["占用状态 == 0?"]
    F -->|否| G["碰撞<br/>返回false"]
    F -->|是| H["安全<br/>返回true"]
```

### 2.4 Shot Trajectory计算
```mermaid
flowchart TD
    A["当前状态"] --> B["目标状态"]
    B --> C["计算时间到目标<br/>time_to_goal"]
    C --> D["三次多项式插值<br/>p(t) = at³ + bt² + ct + d"]
    D --> E["计算多项式系数<br/>a, b, c, d"]
    E --> F["分段碰撞检测<br/>check_num个点"]
    F --> G["所有点都安全?"]
    G -->|否| H["轨迹无效<br/>返回false"]
    G -->|是| I["轨迹有效<br/>返回true"]
```

## 3. 关键数据结构流程图

### 3.1 节点创建和管理
```mermaid
flowchart TD
    A["需要新节点"] --> B["从节点池获取<br/>path_node_pool_[use_node_num_]"]
    B --> C["初始化节点状态<br/>state, g_score, f_score"]
    C --> D["设置父节点<br/>parent = cur_node"]
    D --> E["设置节点状态<br/>node_state = OPEN"]
    E --> F["use_node_num_++"]
    F --> G["返回新节点"]
```

### 3.2 OPEN集合管理
```mermaid
flowchart TD
    A["新节点"] --> B["计算f值<br/>f = g + h"]
    B --> C["加入OPEN集合<br/>open_set_.push(neighbor)"]
    C --> D["OPEN集合自动排序<br/>按f值从小到大"]
    D --> E["下次取出f值最小节点<br/>open_set_.top()"]
```

### 3.3 已扩展节点管理
```mermaid
flowchart TD
    A["新节点"] --> B["计算哈希键<br/>index = [x,y,z] 或 [x,y,z,t]"]
    B --> C["检查是否已扩展<br/>expanded_nodes_.find(index)"]
    C --> D{"已存在?"}
    D -->|是| E["比较f值<br/>f_new < f_old?"]
    E -->|是| F["更新节点<br/>expanded_nodes_[index] = neighbor"]
    E -->|否| G["丢弃新节点"]
    D -->|否| H["插入新节点<br/>expanded_nodes_[index] = neighbor"]
    F --> I["继续处理"]
    G --> I
    H --> I
```

## 4. 算法参数影响分析

### 4.1 关键参数对算法性能的影响

| 参数 | 影响 | 建议值 | 说明 |
|------|------|--------|------|
| `max_tau_` | 时间步长 | 0.6s | 影响搜索精度和速度 |
| `max_vel_` | 最大速度 | 3.0 m/s | 动力学约束 |
| `max_acc_` | 最大加速度 | 2.0 m/s² | 动力学约束 |
| `w_time_` | 时间权重 | 10.0 | 平衡时间和距离 |
| `lambda_heu_` | 启发式权重 | 5.0 | 影响搜索方向 |
| `allocate_num_` | 节点池大小 | 100000 | 内存管理 |
| `check_num_` | 碰撞检测点数 | 5 | 轨迹安全性 |

### 4.2 参数调优建议

```mermaid
flowchart TD
    A["算法性能问题"] --> B{"搜索速度慢?"}
    B -->|是| C["增加max_tau_<br/>减少check_num_"]
    B -->|否| D{"路径质量差?"}
    D -->|是| E["减少max_tau_<br/>增加w_time_"]
    D -->|否| F{"内存不足?"}
    F -->|是| G["增加allocate_num_"]
    F -->|否| H{"碰撞频繁?"}
    H -->|是| I["增加check_num_<br/>调整安全阈值"]
    H -->|否| J["算法运行正常"]
```

## 5. 调试和可视化建议

### 5.1 关键调试点
1. **状态传播**: 检查双积分器模型计算
2. **启发式计算**: 验证四次多项式求解
3. **碰撞检测**: 确认ESDF查询结果
4. **f值更新**: 检查g和h值计算

### 5.2 可视化建议
1. **搜索树**: 可视化OPEN集合和已扩展节点
2. **轨迹**: 显示生成的路径和轨迹
3. **状态空间**: 6维状态空间的可视化
4. **性能**: 搜索时间和内存使用情况

这个详细的流程图和说明应该能帮助你更好地理解KinodynamicAstar算法的实现细节和源码阅读顺序。
