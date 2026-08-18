# 左臂局放自动检测系统使用说明

## 1. 系统用途

本系统用于左侧 RM75-B 七轴机械臂执行局放自动检测任务。

完整流程：

1. 好管家 Java 后台发送局放任务。
2. HTTP 服务接收任务。
3. 左臂恢复到固定初始位姿。
4. D435 检查目标距离和深度数据有效性。
5. 根据深度规划前伸轨迹。
6. 左臂沿 base_link +Z 方向前伸。
7. D435 距离达到 0.140 m 后停止。
8. 左臂保持当前检测位姿。
9. G300 执行局放检测。
10. G300 检测完成后左臂沿原轨迹返回。
11. 验证机械臂返回。
12. 整个任务成功后 HTTP 才向好管家返回成功。

当前 HTTP 接口为同步执行模式。

---

## 2. 设备信息

### RK / 上位机

本机同时承担：

- 好管家 Java 后台
- ROS 2
- 左臂任务服务
- D435
- G300
- HTTP 局放服务

局放 HTTP 服务地址：

    127.0.0.1:8080

只监听本机 loopback，不对外部网络开放。

### 左机械臂

    型号：RM75-B
    IP：192.168.3.18

### 右机械臂

    型号：RM75-B
    IP：192.168.3.19

右臂不属于本说明中的左臂局放执行链。

注意：

    127.0.0.1:8080
        = 左臂局放 HTTP 服务

    192.168.3.19:8080
        = 右臂睿尔曼控制器 TCP/JSON 接口

二者不是同一个服务。

### 左侧 D435

    Serial：348122075281

ROS namespace：

    /left_probe

主要距离话题：

    /left_probe/distance_m
    /left_probe/near_distance_m
    /left_probe/distance_valid

### G300

    接口：/dev/ttyUSB0
    通信：Modbus RTU
    波特率：115200
    数据位：8
    停止位：1
    校验：None
    Slave ID：2

---

## 3. 主要程序

### HTTP 接口

    ~/ros2_ws/left_probe_http_server.py

功能：

- 接收好管家 Java 的 HTTP 请求
- 接收 task_id
- 固定局放检测类型
- 调用 ROS 2 Task Server
- 同步等待完整任务结束
- 最终返回 SUCCESS / FAILED

### ROS 任务服务器

    ~/ros2_ws/left_probe_task_server.py

ROS 服务：

    /left_probe/task/start
    /left_probe/task/status
    /left_probe/task/cancel
    /left_probe/task/reset
    /left_probe/task/plan

### 左臂完整检测周期

    ~/ros2_ws/left_arm_depth_cycle.py

负责：

- D435 距离闭环
- 前伸
- 停止确认
- G300 调用
- 检测位置保持
- 原轨迹返回
- 结果保存

### 七轴机械臂轨迹

    ~/ros2_ws/left_arm_smooth_canfd_forward.py

采用：

- 固定臂角 IK
- 约 0.5 mm 关键点
- 100 Hz CANFD
- 五次 S 曲线
- 返回阶段复用前伸关节轨迹反向执行

### 初始位姿恢复

    ~/ros2_ws/left_arm_ensure_initial_pose.py

初始位姿文件：

    ~/ros2_ws/left_arm_longstroke_start_retracted_150mm.json

### D435 距离节点

    ~/ros2_ws/left_d435_distance.py

### G300

    ~/ros2_ws/g300_full_acquire.py

---

## 4. 启动系统

进入工作空间：

    cd ~/ros2_ws

启动全部左臂局放常驻服务：

    ./start_left_probe_service.sh

启动流程包括：

1. ROS 环境
2. RM75 左臂 driver
3. 左臂初始位姿检查/恢复
4. D435
5. D435 距离节点
6. Task Server
7. HTTP Server

启动成功后 shell 可以退出，服务继续常驻。

---

## 5. 查看运行状态

执行：

    cd ~/ros2_ws
    ./status_left_probe_service.sh

也可以单独检查 HTTP：

    curl -sS \
      http://127.0.0.1:8080/health \
      | python3 -m json.tool

正常情况下应包含：

    "ok": true
    "ready": true

以及：

    "start": true
    "status": true
    "cancel": true

D435 正常时还应满足：

    "depth_pipeline_fresh": true
    "distance_valid": true
    "distance_fresh": true

---

## 6. 好管家正式调用协议

好管家 Java 实际调用：

    POST http://127.0.0.1:8080/start_task

Header：

    Content-Type: application/json

JSON Body：

    {
      "task_type": "detection",
      "task_id": "0f30754fadc14b1abbe11f759c5cbf76"
    }

其中：

### task_type

当前左臂服务只执行：

    detection

### task_id

由好管家 Java 生成。

每个任务必须使用唯一 task_id。

例如：

    0f30754fadc14b1abbe11f759c5cbf76

task_id 同时用于：

- HTTP任务追踪
- ROS任务关联
- 日志目录
- G300结果目录
- 故障追溯

---

## 7. detection_type

好管家 Java 当前不会发送 detection_type。

因此左臂服务默认：

    detection_type = all

即一次完整任务依次执行：

    开放式超声波
        0x01

    TEV
        0x11

    接触式超声波
        0x21

    UHF
        0x31

---

## 8. 同步调用规则

当前 /start_task 是同步接口。

好管家发送：

    POST /start_task

之后 HTTP 连接不会马上返回。

服务器会等待：

    左臂恢复初始位姿
        ↓
    D435检查
        ↓
    轨迹规划
        ↓
    左臂前伸
        ↓
    0.140 m停止
        ↓
    G300四通道检测
        ↓
    左臂返回
        ↓
    返回验证
        ↓
    SUCCESS / FAILED

任务完整结束后 HTTP 才返回。

因此：

    好管家显示“操作成功”

现在代表：

    整个左臂局放任务已经成功执行完成

而不是仅代表“请求已经收到”。

---

## 9. HTTP 成功响应

完整任务成功：

HTTP：

    200 OK

典型 JSON：

    {
      "task_id": "0f30754fadc14b1abbe11f759c5cbf76",
      "status": "success",
      "type": "detection",
      "detection_type": "all",
      "message": "Task completed successfully"
    }

注意：

    status = success

表示：

    机械臂 + D435 + G300 + 返回流程执行成功

它不等价于：

    检测到了局放

是否存在局放应查看 G300 检测结果。

---

## 10. HTTP 失败响应

以下情况可能导致失败：

- D435无有效数据
- D435数据停止刷新
- 初始目标距离不可达
- 左臂恢复失败
- IK规划失败
- 前伸运动失败
- D435硬安全距离触发
- G300通信失败
- G300检测失败
- 左臂返回失败
- 最终位姿校验失败
- 任务超时

失败时：

    status = failed

并通过 HTTP 4xx / 5xx 返回。

---

## 11. Java HTTP 超时

因为当前接口为同步接口，Java 侧不能设置很短的 read timeout。

HTTP Server 当前任务等待上限：

    360 s

因此 Java read timeout 建议：

    >= 400 s

否则可能出现：

    左臂仍在正常执行
        ↓
    Java HTTP连接提前超时
        ↓
    管理系统认为任务失败

注意：

客户端提前断开 HTTP 连接，不代表机器人任务自动取消。

---

## 12. 左臂距离参数

当前正式参数：

    stop_depth             = 0.140 m
    hard_min_depth         = 0.090 m
    slow_depth             = 0.200 m

    max_distance           = 0.350 m
    reach_margin           = 0.010 m

    depth_motion_scale     = 0.80

    fast_speed             = 0.080 m/s
    slow_speed             = 0.015 m/s
    return_speed           = 0.080 m/s

    control rate           = 100 Hz
    key_step               = 0.0005 m

机械臂实际安全允许前伸约：

    0.340 m

按照当前 depth_motion_scale，初始 D435 距离理论上限约：

    0.412 m

现场推荐开始任务时目标距离：

    0.25 ~ 0.35 m

如果明显超过约 0.412 m，任务应在运动前拒绝，而不是盲目前伸。

---

## 13. 停止距离

正常检测时：

    D435 <= 0.140 m

第一次达到停止阈值后：

1. 停止继续前伸。
2. 等待新的深度帧。
3. 连续确认 3 帧。
4. 确认停止。
5. 开始 G300 检测。

硬安全距离：

    0.090 m

达到硬安全条件时不应继续向目标前伸。

---

## 14. G300 检测

G300 每种检测执行：

    20 个采集周期

最终放电类型：

    0 = 无放电
    1 = 内部放电
    2 = 表面放电
    3 = 悬浮电位放电
    4 = 电晕放电
    9 = 分析未完成

判断是否存在局放时，应以最终 G300 分析结果为准。

不能仅根据 basic、脉冲数量等中间数值判定存在局放。

---

## 15. G300检测期间机械臂行为

到达停止位置后，机械臂不会卸掉保持。

G300检测期间程序持续保持：

    stop_q

通过 CANFD 持续发送目标关节位置。

因此流程为：

    到达检测位置
        ↓
    主动保持
        ↓
    G300采集
        ↓
    G300完成
        ↓
    原轨迹返回

---

## 16. 返回方式

左臂返回时不重新求 IK。

返回直接使用：

    前伸过程中实际执行/规划的关节轨迹
        ↓
    倒序
        ↓
    返回初始位置

这样可以避免七轴冗余机械臂重新求逆解时跳到其他关节分支。

---

## 17. 手动模拟好管家

测试时生成一个新 task_id：

    TASK_ID="$(python3 - <<'PY'
    import uuid
    print(uuid.uuid4().hex)
    PY
    )"

发送：

    time curl -sS \
      -X POST \
      http://127.0.0.1:8080/start_task \
      -H 'Content-Type: application/json' \
      -d "{
        \"task_type\":\"detection\",
        \"task_id\":\"$TASK_ID\"
      }" \
      | python3 -m json.tool

注意：

该命令会一直等待完整机械臂任务结束，这是正常现象。

---

## 18. 查询任务状态

同步接口正常情况下好管家不需要额外轮询。

现场调试仍可使用：

    curl -sS \
      "http://127.0.0.1:8080/status?task_id=$TASK_ID" \
      | python3 -m json.tool

可能状态：

    idle
    executing
    success
    failed

内部 ROS 状态可能包括：

    QUEUED
    RESTORING_INITIAL
    VALIDATING
    PLANNING
    APPROACHING
    G300_DETECTING
    G300_COMPLETE
    RETURNING
    VERIFYING
    SUCCESS

---

## 19. 查看 ROS Task 状态

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash

    ros2 service call \
      /left_probe/task/status \
      std_srvs/srv/Trigger \
      '{}'

---

## 20. 日志

常驻服务日志：

    ~/ros2_ws/left_probe_service/logs/

HTTP：

    ~/ros2_ws/left_probe_service/logs/http_server.log

Task Server：

    ~/ros2_ws/left_probe_service/logs/task_server.log

实时查看 HTTP：

    tail -f \
      ~/ros2_ws/left_probe_service/logs/http_server.log

实时查看 Task Server：

    tail -f \
      ~/ros2_ws/left_probe_service/logs/task_server.log

---

## 21. 每个任务的结果目录

每个 task_id 对应：

    ~/ros2_ws/left_probe_service/task_logs/<task_id>/

主要包括：

    task_result.json

以及：

    g300/<时间戳>/

G300目录主要包含：

    full_result.json
    samples.csv
    spectrum_360.csv
    summary.csv
    modbus_transactions.csv
    modbus_raw.log
    terminal.log

其中：

    task_result.json

是整套任务最终结果。

    full_result.json

是 G300 完整检测结果。

---

## 22. 停止服务

执行：

    cd ~/ros2_ws
    ./stop_left_probe_service.sh

当前 stop 脚本已经包含 HTTP 兜底清理：

1. TERM HTTP进程
2. 等待退出
3. 必要时 KILL
4. 删除旧 PID
5. 检查 TCP 8080
6. 继续关闭其他左臂服务

停止后检查：

    pgrep -af \
      'left_probe_http_server.py' \
      || echo "HTTP stopped"

    ss -lntp \
      | grep ':8080 ' \
      || echo "8080 free"

---

## 23. 重启系统

标准方式：

    cd ~/ros2_ws

    ./stop_left_probe_service.sh

    sleep 2

    ./start_left_probe_service.sh

不要在常驻服务已经运行时再次手动执行：

    python3 left_probe_http_server.py ...

否则第二个进程会因为 8080 已占用而报：

    OSError: [Errno 98] Address already in use

---

## 24. 监听好管家真实请求

因为好管家 Java 和 HTTP 服务都在 RK 本机：

    sudo stdbuf -oL tcpdump \
      -i lo \
      -nn \
      -s 0 \
      -A \
      'tcp dst port 8080'

正常可以看到：

    POST /start_task HTTP/1.1
    Content-Type: application/json
    User-Agent: Java/1.8.0_472
    Host: 127.0.0.1:8080

以及：

    {
      "task_type":"detection",
      "task_id":"..."
    }

---

## 25. 只允许单任务执行

左臂一次只能执行一个局放任务。

执行过程中如果再次收到新的 detection：

    HTTP 409

不会同时驱动两个左臂任务。

好管家应等待当前调用返回后，再执行下一巡检动作。

---

## 26. 上电后的推荐操作

每次系统重新上电后按以下顺序检查：

1. 启动服务

       ./start_left_probe_service.sh

2. 查看状态

       ./status_left_probe_service.sh

3. HTTP健康检查

       curl -sS \
         http://127.0.0.1:8080/health \
         | python3 -m json.tool

4. 确认

       ready = true

5. 确认 D435 距离合理

       建议约 0.25 ~ 0.35 m

6. 再允许好管家发送 detection。

---

## 27. 安全注意事项

1. D435无有效距离时禁止局放动作。
2. 初始目标明显不可达时禁止机械臂前伸。
3. 不要手动修改 0.090 m 硬安全距离后直接实机测试。
4. G300失败后机械臂仍应优先安全返回。
5. HTTP客户端中断不等于机械臂任务取消。
6. 执行中不要重复点击局放任务。
7. 调试机械臂轨迹时优先使用吊机/低速条件。
8. 不要直接删除 task_logs，正式测试结果需保留用于追溯。

---

## 28. 当前正式调用关系

    好管家 Java
          |
          | POST /start_task
          | task_type=detection
          | task_id=<UUID>
          v
    127.0.0.1:8080
          |
          v
    left_probe_http_server.py
          |
          v
    /left_probe/task/start
          |
          v
    left_probe_task_server.py
          |
          +---- 初始位姿恢复
          |
          +---- D435
          |
          +---- 左臂轨迹控制
          |
          +---- G300
          |
          +---- 原轨迹返回
          |
          v
       SUCCESS
          |
          v
      HTTP 200
          |
          v
       好管家

这是当前正式左臂局放运行链路。
