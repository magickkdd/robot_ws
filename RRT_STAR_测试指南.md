# RRT* 规划服务启动和测试指南

## 启动流程

### 步骤 1：启动仿真环境和 RViz

```bash
# 在终端1中执行
cd /home/czx/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch battery_disassembly_arm sim.launch.py
```

这将启动：
- robot_state_publisher（发布机器人模型）
- joint_state_publisher_gui（关节控制滑块）
- RViz（3D可视化）

### 步骤 2：启动 RRT* 规划服务

```bash
# 在终端2中执行
cd /home/czx/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch battery_disassembly_arm rrt_star_service.launch.py
```

这将启动：
- rrt_star_service_node（RRT*规划服务）

### 步骤 3：配置 RViz 可视化 RRT* 路径

在 RViz 中：
1. 点击左下角 "Add" 按钮
2. 选择 "By topic" 选项卡
3. 找到 `/rrt_star_path` 话题
4. 展开并选择 "MarkerArray"
5. 点击 "OK"

### 步骤 4：测试 RRT* 规划服务

```bash
# 在终端3中执行
cd /home/czx/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 测试规划服务
ros2 service call /rrt_star_plan battery_disassembly_arm/srv/RRTStarPlan \
  "{start_joint_values: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    goal_joint_values: [0.5, -0.5, 0.5, 0.0, 0.5, 0.0]}"
```

## 预期结果

### 服务响应
- `success: True`
- `message: 'RRT* 规划成功.'`
- `num_points: X`（路径点数）
- `dof: 6`（6个关节）
- `path_joint_values: [...]`（完整关节轨迹）

### RViz 显示
- 在 RViz 中应该看到红色的路径线（LINE_STRIP marker）
- 路径连接起点和终点
- 每次调用服务时，路径会更新

## 可用的测试样例

```bash
# 样例 1：简单路径
ros2 service call /rrt_star_plan battery_disassembly_arm/srv/RRTStarPlan \
  "{start_joint_values: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    goal_joint_values: [0.5, -0.5, 0.5, 0.0, 0.5, 0.0]}"

# 样例 2：不同路径
ros2 service call /rrt_star_plan battery_disassembly_arm/srv/RRTStarPlan \
  "{start_joint_values: [0.1, -0.2, 0.3, -0.1, 0.2, -0.3],
    goal_joint_values: [0.6, -0.4, 0.7, -0.2, 0.3, 0.1]}"
```

## 故障排除

### 如果服务调用失败
1. 检查 rrt_star_service_node 是否运行：`ros2 node list | grep rrt_star`
2. 检查服务是否可用：`ros2 service list | grep rrt_star`

### 如果 RViz 中看不到路径
1. 确保添加了 MarkerArray 显示
2. 检查话题：`ros2 topic list | grep rrt_star`
3. 检查 MarkerArray 数据：`ros2 topic echo /rrt_star_path`

### 当前限制
- 碰撞检测已临时禁用（仅用于测试规划算法）
- 路径显示为关节空间轨迹的简化可视化

## 日志信息

正常运行时应看到：
```
[rrt_star_service]: RRT*服务启动.
[RRT*] 警告：碰撞检测已临时禁用，仅供测试
[RRT*] 开始规划，最大迭代: 5000
[RRT*] 找到路径! 迭代次数: X, 节点数: Y
[rrt_star_service]: [服务] 规划结果: 成功
[rrt_star_service]: [服务] 路径点数: Z
```