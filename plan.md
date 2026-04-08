# 山猫M20 Pro 单点导航实现方案

## Context

为云深处山猫M20 Pro轮足机器人实现单点导航功能。核心挑战：

1. **转向死区**：低角速度指令时机器人不响应转向——这是RL运动控制策略的特性，小角速度被"吸收"
2. **非标准转向**：M20是4腿+4轮的轮足机器人（滑移转向），不同于普通差速/阿克曼小车
3. **纯Python开发**：不使用ROS，通过UDP直接与运动主机通信

### M20关键信息（来自URDF和SDK）

- 4条腿，每腿3个关节(hipx/hipy/knee) + 1个轮子(wheel)
- 轮子为continuous关节，无独立转向机构 → **滑移转向**
- 控制接口：`geometry_msgs/Twist` 格式
  - `linear.x`：纵向速度（正=前进）
  - `linear.y`：横向速度（正=左移）← **轮足特有能力**
  - `angular.z`：角速度（正=左转）
- 运动主机通信：UDP协议，目标IP `10.21.31.103`（WiFi连接）
- 推荐控制频率：10 Hz
- 需要通过手柄切换到"SDK模式"才能接受程序控制

---

## 整体架构

```
┌─────────────────────────────────────────────────┐
│                 point_nav.py                     │
│                                                  │
│  ┌───────────┐   ┌───────────┐   ┌───────────┐  │
│  │ UDPComm   │   │ Navigator │   │ DeadZone  │  │
│  │ (收发UDP) │   │ (状态机)  │   │ Compensat │  │
│  └─────┬─────┘   └─────┬─────┘   └─────┬─────┘  │
│        │               │               │         │
│  ┌─────▼─────────────────────────────────────┐   │
│  │           MainLoop (10Hz)                 │   │
│  │  1. 接收机器人状态(位置/航向)              │   │
│  │  2. Navigator计算目标速度                  │   │
│  │  3. DeadZone补偿角速度                     │   │
│  │  4. 发送速度指令                           │   │
│  └───────────────────────────────────────────┘   │
└─────────────────────────────────────────────────┘
```

### 文件结构

```
m20_point_nav/
├── udp_comm.py          # UDP通信层（收发运动指令和状态）
├── state_reader.py      # 解析机器人状态数据（位姿、IMU）
├── dead_zone.py         # 转向死区补偿
├── navigator.py         # 三阶段导航状态机（核心逻辑）
├── point_nav.py         # 主程序入口
├── calibrate.py         # 死区标定工具
└── config.py            # 所有可调参数
```

---

## 第一步：UDP通信层 (`udp_comm.py`)

参考 DeepRoboticsLab/Lite3_ROS 的实现方式，M20运动主机通过UDP收发数据。

```python
# 核心接口
class M20Comm:
    def __init__(self, robot_ip="10.21.31.103", cmd_port=43897, state_port=43893):
        ...
    
    def send_velocity(self, vx: float, vy: float, omega: float):
        """发送速度指令 (纵向, 横向, 角速度)"""
        # 将Twist格式打包为M20 UDP数据包
        ...
    
    def get_robot_state(self) -> RobotState:
        """获取机器人状态 (x, y, yaw, vx, vy, omega)"""
        # 解析UDP返回的状态数据
        ...
```

> **注意**：UDP端口和数据格式需要参照M20实际SDK确认。Lite3使用 `target_port=43897, local_port=43893`，M20可能不同。如果M20使用的是drdds通信格式，需要参考 `sdk_deploy/src/drdds/` 中的序列化方式。

### 位姿获取方案

由于不确定是否能正常获取里程计，设计两级方案：

1. **首选**：从UDP状态反馈中读取里程计数据（位置+航向）
2. **备选**：如果无法获取位置，仅用IMU航向角 + 轮速积分估算位置
3. **最简**：如果以上都不行，先实现纯航向控制（只管转向和直走），用外部定位（如激光雷达）补充位置

---

## 第二步：转向死区补偿 (`dead_zone.py`)

这是解决核心问题的关键模块。

### 死区问题分析

M20的RL运动策略在接收小角速度指令时不产生实际转向，原因：
- RL策略的奖励函数可能对小角速度不敏感
- 轮足机器人的转向需要协调4条腿和4个轮子，惯性大
- 存在一个隐式的最小有效角速度 `ω_min`

### 补偿策略：最小值钳位 + 脉冲式小角度修正

```python
def compensate_angular(omega_raw: float, omega_min: float = 0.15, epsilon: float = 0.02) -> float:
    """
    死区补偿：
    - |omega_raw| < epsilon: 返回0（不需要转向）
    - epsilon <= |omega_raw| < omega_min: 钳位到omega_min（跨越死区）
    - |omega_raw| >= omega_min: 直接使用
    """
    if abs(omega_raw) < epsilon:
        return 0.0
    if abs(omega_raw) < omega_min:
        return omega_min * (1.0 if omega_raw > 0 else -1.0)
    return omega_raw
```

### 关键参数

| 参数 | 含义 | 初始值 | 说明 |
|------|------|--------|------|
| `omega_min` | 最小有效角速度 | 0.15 rad/s | **需要标定** |
| `epsilon` | 航向误差忽略阈值 | 0.02 rad (~1°) | 小于此值认为已对准 |
| `omega_max` | 最大角速度限制 | 0.8 rad/s | 安全限制 |

---

## 第三步：三阶段导航状态机 (`navigator.py`)

```
         ┌──────────────┐
         │   IDLE       │ ← 等待目标点
         └──────┬───────┘
                │ 收到目标(x_goal, y_goal)
                ▼
         ┌──────────────┐
         │  ROTATE      │ ← Phase 1: 原地旋转对准目标
         │              │   发送: vx=0, vy=0, omega=±ω
         └──────┬───────┘
                │ |heading_error| < threshold_1 (如0.1rad≈6°)
                ▼
         ┌──────────────┐
         │  DRIVE       │ ← Phase 2: 前进 + 航向微调
         │              │   发送: vx=v, vy=0, omega=PD修正
         └──────┬───────┘
                │ distance < approach_radius (如0.5m)
                ▼
         ┌──────────────┐
         │  APPROACH    │ ← Phase 3: 低速精确到达
         │              │   发送: vx=v_slow, vy=vy_lateral, omega=小修正
         └──────┬───────┘
                │ distance < goal_tolerance (如0.2m)
                ▼
         ┌──────────────┐
         │  REACHED     │ ← 到达！发送: vx=0, vy=0, omega=0
         └──────────────┘
```

### Phase 1: ROTATE（原地转向）

```python
def phase_rotate(self):
    heading_error = normalize_angle(target_bearing - current_yaw)
    
    # PD控制器计算期望角速度
    omega_raw = self.kp_heading * heading_error + self.kd_heading * (heading_error - self.prev_error)
    
    # 死区补偿
    omega = compensate_angular(omega_raw, self.omega_min)
    
    # 限幅
    omega = clamp(omega, -self.omega_max, self.omega_max)
    
    return (0.0, 0.0, omega)  # vx=0, vy=0, 只转向
```

**进入DRIVE条件**：`|heading_error| < 0.1 rad` 且持续 `0.3s`（防抖）

### Phase 2: DRIVE（前进+航向修正）

```python
def phase_drive(self):
    distance = calc_distance(current_pos, goal_pos)
    heading_error = normalize_angle(target_bearing - current_yaw)
    
    # 航向误差过大 → 回到ROTATE
    if abs(heading_error) > 0.5:  # ~30°
        return TRANSITION_TO_ROTATE
    
    # 速度规划：远处快、近处慢
    vx = min(self.vx_max, self.kp_distance * distance)
    
    # 航向PD修正（行进中微调）
    omega_raw = self.kp_heading * heading_error
    omega = compensate_angular(omega_raw, self.omega_min)
    
    return (vx, 0.0, omega)
```

**回退ROTATE条件**：`|heading_error| > 0.5 rad`（~30°偏差过大）
**进入APPROACH条件**：`distance < approach_radius`

### Phase 3: APPROACH（精确接近 — 利用侧向移动）

这是轮足机器人的优势。近距离时利用 `linear.y` 侧向速度精确定位，无需精确对齐航向：

```python
def phase_approach(self):
    # 将目标点转换到机器人本体坐标系
    dx_body, dy_body = world_to_body(goal_pos - current_pos, current_yaw)
    
    # 纵向+横向同时控制
    vx = clamp(self.kp_approach * dx_body, -self.vx_slow, self.vx_slow)
    vy = clamp(self.kp_lateral * dy_body, -self.vy_max, self.vy_max)
    
    return (vx, vy, 0.0)  # 利用侧向移动，不需要转向
```

---

## 第四步：关键参数配置 (`config.py`)

```python
NAV_CONFIG = {
    # 通信
    "robot_ip": "10.21.31.103",
    "cmd_port": 43897,
    "state_port": 43893,
    "control_freq": 10,          # Hz
    
    # 死区补偿（需标定）
    "omega_min": 0.15,           # rad/s，最小有效角速度
    "omega_max": 0.8,            # rad/s，最大角速度
    "epsilon": 0.02,             # rad，角度忽略阈值
    
    # 航向PD控制
    "kp_heading": 1.5,           # 航向P增益
    "kd_heading": 0.1,           # 航向D增益
    
    # 速度控制
    "vx_max": 0.5,               # m/s，最大前进速度
    "vx_slow": 0.15,             # m/s，接近阶段速度
    "vy_max": 0.2,               # m/s，最大侧向速度
    "kp_distance": 0.8,          # 距离P增益
    "kp_approach": 0.5,          # 接近阶段P增益
    "kp_lateral": 0.5,           # 侧向P增益
    
    # 状态转换阈值
    "heading_threshold": 0.1,    # rad，转向→前进阈值
    "heading_revert": 0.5,       # rad，前进→回转阈值
    "approach_radius": 0.5,      # m，进入接近阶段
    "goal_tolerance": 0.2,       # m，到达判定
    "settle_time": 0.3,          # s，状态转换防抖
}
```

---

## 第五步：死区标定工具 (`calibrate.py`)

使用前必须先标定 `omega_min`：

```
标定流程：
1. 连接机器人，切换到SDK模式
2. 从 omega=0 开始，以0.01为步长递增角速度指令
3. 每个步长保持2秒，记录IMU实际角速度
4. 找到机器人开始实际转动的最小omega值 → 即omega_min
5. 分别标定左转和右转（可能不对称）
```

---

## 第六步：验证方案

### 阶段1：通信验证
1. 运行UDP通信模块，确认能收到机器人状态数据
2. 发送简单的前进指令(vx=0.2)，观察机器人是否响应
3. 确认位姿数据（x, y, yaw）的更新频率和精度

### 阶段2：死区标定
1. 运行 `calibrate.py`
2. 记录 `omega_min` 的值（左转和右转分别记录）
3. 更新 `config.py`

### 阶段3：单阶段测试
1. 只测试ROTATE：发送目标航向，观察原地转向是否准确
2. 只测试DRIVE：手动对齐后，测试直线行驶
3. 只测试APPROACH：近距离测试侧向移动定位

### 阶段4：完整导航测试
1. 设置2m外的目标点，运行完整导航
2. 测量到达精度，调整参数
3. 测试不同方向（前、后、侧方）的目标点

---

## 关于"转向不同于一般小车"的补充说明

M20的转向与普通轮式机器人不同的根本原因：

1. **RL策略中介**：速度指令不是直接驱动电机，而是经过RL策略"翻译"为腿+轮协调动作，策略有自己的响应特性
2. **滑移转向**：没有独立转向机构，通过左右轮速差+腿部姿态调整实现转向，响应滞后
3. **地形适应**：RL策略会根据地面情况动态调整，同样的角速度指令在不同地面上效果不同
4. **惯性大**：轮足机器人质量大、重心高，转向惯性远大于小型差速小车

因此设计中采用了：
- **保守的速度参数**（初始vx_max=0.5, omega_max=0.8）
- **死区补偿**（确保角速度指令超过最小有效值）
- **PD而非PID控制**（避免积分项导致过冲，轮足机器人对过冲敏感）
- **状态转换防抖**（0.3s确认，避免频繁切换）
- **三阶段分离**（不要求同时快速前进和大幅转向）
