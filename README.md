# M20 单目标点控制

这个仓库实现了基于山猫 M20/M20 Pro 巡检通信协议的单目标点运动控制。实现方式不是调用机器人自带地图导航，而是在外部系统作为 TCP/UDP 客户端时：

- 持续发送 heartbeat，维持机器人向客户端回传状态
- 在 `Regular Mode` 下发送 `Type=2, Command=21` 的轴向运动指令
- 读取 `1002/6` 基础状态和 `1002/4` 运动状态
- 用 `Yaw + LinearX + LinearY` 做相对位姿积分
- 按 `ROTATE -> DRIVE -> APPROACH -> REACHED` 状态机把机器人驱动到目标点

## 协议约束

- 机器人本体是服务端，外部板卡或系统是客户端
- UDP 服务端地址端口默认是 `10.21.31.103:30000`
- TCP 服务端地址端口默认是 `10.21.31.103:30001`
- 报文使用 16 字节协议头加 ASDU，本实现默认使用 JSON ASDU
- heartbeat 建议至少 `1 Hz`
- 轴向运动控制建议 `20 Hz`

## 运行

UDP:

```bash
python3 -m m20control.point_nav --transport udp --goal-x 2.0 --goal-y 0.0
```

TCP:

```bash
python3 -m m20control.point_nav --transport tcp --goal-x 2.0 --goal-y 0.0
```

常用参数：

- `--full-scale-x/--full-scale-y/--full-scale-yaw`：把协议轴值 `1.0` 映射到实际最大速度的标定参数
- `--gait`：默认 `1`，对应 Basic gait
- `--motion-state`：默认 `1`，对应 Stand
- `--local-port`：UDP 模式下可固定本地端口，便于联调抓包

## 代码结构

- `m20control/protocol.py`：协议头与帧编解码
- `m20control/transport.py`：TCP/UDP 传输层
- `m20control/client.py`：heartbeat、状态接收、模式切换、轴向控制、相对里程估计
- `m20control/navigator.py`：单目标点状态机
- `m20control/point_nav.py`：命令行入口

## 限制

这是相对位姿导航，不是绝对地图导航。当前实现没有接入外部 SLAM/RTK/视觉定位，所以运行距离越长，积分漂移越大。要做稳定的绝对到点，需要把 `client.py` 里的里程估计替换成外部定位源，或者切到机器人内建的导航任务接口。
