# KinodynamicAstar函数输入输出详细说明

## 1. 核心算法函数

### 1.1 主搜索函数
```cpp
int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
           Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
           Eigen::Vector3d end_vel, bool init, bool dynamic = false,
           double time_start = -1.0);
```

**输入参数**:
- `start_pt`: 起始位置 [x, y, z] (单位: 米)
- `start_vel`: 起始速度 [vx, vy, vz] (单位: 米/秒)
- `start_acc`: 起始加速度 [ax, ay, az] (单位: 米/秒²)
- `end_pt`: 目标位置 [x, y, z] (单位: 米)
- `end_vel`: 目标速度 [vx, vy, vz] (单位: 米/秒)
- `init`: 是否初始化搜索 (true: 重新开始, false: 继续搜索)
- `dynamic`: 是否为动态环境 (true: 4D搜索, false: 3D搜索)
- `time_start`: 起始时间 (单位: 秒, -1.0表示当前时间)

**输出返回值**:
- `REACH_HORIZON = 1`: 到达搜索视野边界
- `REACH_END = 2`: 成功到达目标
- `NO_PATH = 3`: 无可行路径
- `NEAR_END = 4`: 接近目标(用于动态环境)

**功能**: 执行A*搜索算法的主循环

### 1.2 状态传播函数
```cpp
void stateTransit(Eigen::VectorXd state0, Eigen::VectorXd state1,
                  Eigen::Vector3d um, double tau);
```

**输入参数**:
- `state0`: 当前状态 [x, y, z, vx, vy, vz] (单位: 米, 米/秒)
- `state1`: 下一状态 [x, y, z, vx, vy, vz] (输出参数, 单位: 米, 米/秒)
- `um`: 控制输入 [ax, ay, az] (单位: 米/秒²)
- `tau`: 时间步长 (单位: 秒)

**输出**: 无返回值，通过`state1`参数输出

**功能**: 根据双积分器模型计算状态转移

**数学模型**:
```
x(t+τ) = x(t) + v(t)τ + 0.5*a(t)τ²
v(t+τ) = v(t) + a(t)τ
```

### 1.3 启发式函数
```cpp
double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time);
```

**输入参数**:
- `x1`: 当前状态 [x, y, z, vx, vy, vz] (单位: 米, 米/秒)
- `x2`: 目标状态 [x, y, z, vx, vy, vz] (单位: 米, 米/秒)
- `optimal_time`: 最优时间估计 (输出参数, 单位: 秒)

**输出返回值**:
- `double`: 启发式代价 (单位: 秒)

**功能**: 计算从当前状态到目标状态的启发式估计

**计算公式**:
```
h = λ * ||p_goal - p_current|| + w_time * t_optimal
```

### 1.4 碰撞检测函数
```cpp
bool isFree(const Eigen::Vector3d& pos);
```

**输入参数**:
- `pos`: 位置坐标 [x, y, z] (单位: 米)

**输出返回值**:
- `true`: 位置安全，无碰撞
- `false`: 位置不安全，有碰撞

**功能**: 检查指定位置是否与障碍物碰撞

**检测步骤**:
1. ESDF距离查询: `getDistance(pos)`
2. 膨胀占用查询: `getInflateOccupancy(pos)`

### 1.5 Shot Trajectory函数
```cpp
bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal);
```

**输入参数**:
- `state1`: 当前状态 [x, y, z, vx, vy, vz] (单位: 米, 米/秒)
- `state2`: 目标状态 [x, y, z, vx, vy, vz] (单位: 米, 米/秒)
- `time_to_goal`: 到达目标的时间 (单位: 秒)

**输出返回值**:
- `true`: 轨迹有效，无碰撞
- `false`: 轨迹无效，有碰撞

**功能**: 计算从当前状态到目标状态的直达轨迹

**轨迹模型**: 三次多项式插值
```
p(t) = a*t³ + b*t² + c*t + d
```

## 2. 初始化和重置函数

### 2.1 初始化函数
```cpp
void init();
```

**输入**: 无

**输出**: 无

**功能**: 初始化算法，分配内存，重置状态

**执行步骤**:
1. 获取地图尺寸: `edt_environment_->getMapSize(map_size_3d_)`
2. 初始化节点池: `path_node_pool_.resize(allocate_num_)`
3. 重置搜索状态: `reset()`

### 2.2 重置函数
```cpp
void reset();
```

**输入**: 无

**输出**: 无

**功能**: 重置搜索状态，清空所有数据结构

**执行步骤**:
1. 清空路径节点: `path_nodes_.clear()`
2. 清空OPEN集合: `open_set_.swap(empty_queue)`
3. 重置节点池: 所有节点状态设为`NOT_EXPAND`
4. 重置计数器: `use_node_num_ = 0`, `iter_num_ = 0`

### 2.3 设置环境函数
```cpp
void setEnvironment(const std::shared_ptr<plan_env::EDTEnvironment>& env);
```

**输入参数**:
- `env`: EDT环境对象的智能指针

**输出**: 无

**功能**: 设置算法使用的环境对象

## 3. 轨迹生成函数

### 3.1 获取轨迹函数
```cpp
std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);
```

**输入参数**:
- `delta_t`: 时间步长 (单位: 秒)

**输出返回值**:
- `std::vector<Eigen::Vector3d>`: 轨迹点列表，每个点为[x, y, z]

**功能**: 从搜索路径生成连续轨迹

**生成步骤**:
1. 从目标节点回溯到起始节点
2. 反转路径顺序
3. 使用三次多项式插值生成连续轨迹

### 3.2 获取采样点函数
```cpp
void getSamples(double& ts, std::vector<Eigen::Vector3d>& point_set,
                std::vector<Eigen::Vector3d>& start_end_derivatives, int& point_num);
```

**输入参数**:
- `ts`: 时间步长 (输出参数, 单位: 秒)
- `point_set`: 轨迹点集合 (输出参数)
- `start_end_derivatives`: 起始和终止导数 (输出参数)
- `point_num`: 轨迹点数量 (输出参数)

**输出**: 无返回值，通过引用参数输出

**功能**: 获取轨迹的采样点和导数信息

### 3.3 获取访问节点函数
```cpp
std::vector<PathNodePtr> getVisitedNodes();
```

**输入**: 无

**输出返回值**:
- `std::vector<PathNodePtr>`: 已访问的节点列表

**功能**: 获取搜索过程中访问的所有节点

## 4. 多项式求解函数

### 4.1 三次多项式求解
```cpp
std::vector<double> cubic(double a, double b, double c, double d);
```

**输入参数**:
- `a`: 三次项系数
- `b`: 二次项系数
- `c`: 一次项系数
- `d`: 常数项系数

**输出返回值**:
- `std::vector<double>`: 多项式根列表

**功能**: 求解三次方程 ax³ + bx² + cx + d = 0

### 4.2 四次多项式求解
```cpp
std::vector<double> quartic(double a, double b, double c, double d, double e);
```

**输入参数**:
- `a`: 四次项系数
- `b`: 三次项系数
- `c`: 二次项系数
- `d`: 一次项系数
- `e`: 常数项系数

**输出返回值**:
- `std::vector<double>`: 多项式根列表

**功能**: 求解四次方程 ax⁴ + bx³ + cx² + dx + e = 0

## 5. 参数设置函数

### 5.1 设置参数函数
```cpp
void setParam(rclcpp::Node::SharedPtr node);
```

**输入参数**:
- `node`: ROS2节点对象

**输出**: 无

**功能**: 从ROS2参数服务器读取算法参数

**参数列表**:
```cpp
// 搜索参数
double max_tau_;           // 最大时间步长
double init_max_tau_;      // 初始最大时间步长
double max_vel_;           // 最大速度
double max_acc_;           // 最大加速度
double w_time_;            // 时间权重
double horizon_;           // 搜索视野
double lambda_heu_;        // 启发式权重
int allocate_num_;         // 预分配节点数
int check_num_;            // 碰撞检测点数
bool optimistic_;          // 乐观搜索模式

// 地图参数
double resolution_;        // 空间分辨率
double time_resolution_;   // 时间分辨率
Eigen::Vector3d origin_;   // 地图原点
Eigen::Vector3d map_size_3d_; // 地图尺寸
```

## 6. 数据结构说明

### 6.1 PathNode结构体
```cpp
struct PathNode {
    Eigen::Vector3i index;           // 3D网格索引
    Eigen::Matrix<double, 6, 1> state; // 6维状态 [x,y,z,vx,vy,vz]
    double g_score, f_score;         // A*算法的g值和f值
    Eigen::Vector3d input;           // 控制输入 [ax,ay,az]
    double duration;                 // 时间步长
    double time;                     // 时间戳
    int time_idx;                    // 时间索引
    PathNode* parent;                // 父节点指针
    char node_state;                 // 节点状态
};
```

**字段说明**:
- `index`: 3D网格索引，用于哈希表查找
- `state`: 6维状态向量 [位置(3D) + 速度(3D)]
- `g_score`: 从起始节点到当前节点的实际代价
- `f_score`: 总代价 f = g + h
- `input`: 控制输入 [加速度向量]
- `duration`: 从父节点到当前节点的时间步长
- `time`: 当前节点的时间戳
- `time_idx`: 时间索引 (用于动态环境)
- `parent`: 父节点指针 (用于路径回溯)
- `node_state`: 节点状态 (OPEN/CLOSE/NOT_EXPAND)

### 6.2 节点状态枚举
```cpp
enum NodeState {
    NOT_EXPAND = 0,  // 未扩展
    OPEN = 1,        // 在OPEN集合中
    CLOSE = 2        // 已扩展
};
```

## 7. 使用示例

### 7.1 基本使用流程
```cpp
// 1. 创建算法对象
auto kino_astar = std::make_shared<path_searching::KinodynamicAstar>();

// 2. 设置环境
kino_astar->setEnvironment(edt_env);

// 3. 设置参数
kino_astar->setParam(node);

// 4. 初始化
kino_astar->init();

// 5. 执行搜索
Eigen::Vector3d start_pt(0, 0, 0);
Eigen::Vector3d start_vel(0, 0, 0);
Eigen::Vector3d start_acc(0, 0, 0);
Eigen::Vector3d end_pt(10, 10, 5);
Eigen::Vector3d end_vel(0, 0, 0);

int status = kino_astar->search(start_pt, start_vel, start_acc, 
                                end_pt, end_vel, true);

// 6. 获取轨迹
if (status == path_searching::KinodynamicAstar::REACH_END) {
    std::vector<Eigen::Vector3d> traj = kino_astar->getKinoTraj(0.1);
    // 使用轨迹...
}
```

### 7.2 动态环境使用
```cpp
// 动态环境搜索
int status = kino_astar->search(start_pt, start_vel, start_acc, 
                                end_pt, end_vel, true, true, 0.0);
```

这个详细的函数说明应该能帮助你理解每个函数的输入输出和功能。
