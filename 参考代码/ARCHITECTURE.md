# ARCHITECTURE

## 项目概述
- 多摄像头视觉检测 → 云台自动指向 → 激光测距/反馈的控制闭环。
- 通过 UDP 接收识别结果（目标框中心与摄像头 ID），解算目标在云台坐标系的姿态，平滑输出俯仰/滚转命令到云台；可选并行读取激光测距数据以做距离补偿/盲区保护。
- 采用适配器模式解耦云台驱动（`gimbal_interface.py`），便于更换不同品牌/协议的云台；激光模块驱动独立于控制主循环。

## 核心模块关系
- `main_trackingV1.5.py`
  - 业务入口与主循环；管理线程、数据融合、平滑控制与日志。
  - 通过 `gimbal_interface.XFGimbalAdapter` 控制云台；通过 `sddm_laser.SDDMLaser` 读取距离（当前线程注释，可随时启用）。
  - 启动 UDP 线程接收检测结果；（可选）激光线程读取距离。
- `gimbal_interface.py`
  - 抽象基类 `GimbalBase` 定义标准接口：`connect / wait_ready / set_attitude / get_attitude / close`。
  - 适配器 `XFGimbalAdapter`：将具体驱动 `xf_gimbal.XFGimbal` 包装为统一接口，隐藏协议细节。
- `xf_gimbal.py`
  - 底层 XF 云台协议实现（私有协议 A9 5B / B5 9A）：组包、CRC、串口读写、状态解析、就绪判定。
  - 提供高阶控制类 `XFGimbal`：管理串口、心跳、目标角、启动/停止、绝对/相对角度指令、状态读取。
- `sddm_laser.py`
  - SDDM 激光测距模块串口驱动：启动/停止测量、CRC 校验、距离帧解析；提供连续/单次测量接口。

## 数据流向
- 视觉输入：
  - 外部算法通过 UDP(JSON) 发送 `{id, objs:[x1,y1,x2,y2], ...}`。
  - `udp_server_thread` 解析后更新共享状态 `SharedData`（摄像头 ID、像素中心、时间戳，线程安全）。
- 激光输入（可选）：
  - `laser_reader_thread` 读取串口帧，更新 `SharedData.dist_m` 与时间戳；异常/盲区时回退到安全默认值。
- 融合与控制：
  - 主循环每帧锁读 `SharedData`，检查时效（`DATA_TIMEOUT`）。
  - 调用 `calculate_destination_angles`：基准角（摄像头阵列标定偏移） + 像素偏差换算角度 + 距离相关的视差补偿 → 目标俯仰/滚转。
  - 平滑插值（每周期最大步长 `MAX_STEP_PER_LOOP`），避免瞬时大跳变。
  - 通过 `gimbal.set_attitude` 下发到云台；按需回读姿态用于日志。
- 输出：
  - 云台实际姿态（来自 XF 协议 ACK）用于监控；激光距离可用于后续算法或安全逻辑。

## 关键类与函数
- `main_trackingV1.5.py`
  - `SharedData`: 线程安全的全局状态（视觉/激光数据）。
  - `udp_server_thread()`: UDP 接收解析线程，更新目标像素与摄像头 ID。
  - `laser_reader_thread()`: 激光测距读取线程，更新距离/盲区保护。
  - `calculate_destination_angles(cam_id, target_cx, target_cy, distance_m)`: 将像素偏差和距离换算为云台指令角，含视差补偿与限幅。
  - `main()`: 初始化硬件，启动线程，50Hz 控制循环（数据获取→解算→平滑→执行→监控）。
- `gimbal_interface.py`
  - `GimbalBase`: 云台接口抽象定义。
  - `XFGimbalAdapter`: 适配 XF 驱动到统一接口，实现 `connect / wait_ready / set_attitude / get_attitude / close`。
- `xf_gimbal.py`
  - `GimbalTarget`: 目标姿态数据类（pitch/roll 等）。
  - `build_packet_from_target()`: 组装 A9 5B 控制帧（含 CRC）。
  - `read_gimbal_status() / _parse_gimbal_status_frame()`: 解析 B5 9A 反馈帧为 `GimbalStatus`。
  - `XFGimbal`: 高阶封装，提供 `wait_ready()`, `update()`, `set_angle_abs()`, `start()/stop()`, `get_last_angles_deg()` 等，并维护心跳线程。
- `sddm_laser.py`
  - `SDDMLaser`: 激光测距驱动，提供 `start_measurement()`, `stop_measurement()`, `read_distance()`（含 CRC 校验与帧解析）。

## 交互要点
- 云台解耦：主循环只依赖适配器接口；更换云台仅需新增适配器类（如 DJI）并在入口替换。
- 线程安全：共享状态通过 `SharedData.lock` 保护；主循环读数据前加锁，过期数据会置为无效。
- 时序与频率：主循环 50Hz 控制；UDP 非阻塞轮询；激光线程阻塞读但在独立线程中不影响主循环。
- 物理与标定：摄像头阵列偏移表 `CAMERA_OFFSETS` 和 FOV → 每像素角度；视差高度按行区分，上中下三排。

## 可选改进
- 恢复并行启动激光线程（解除注释），在日志中加入距离时效检查。
- 将 UDP 包格式/字段校验与异常日志化，便于联调。
- 为 `GimbalBase` 再增加“健康检查/重连”标准接口，方便适配其他品牌。
