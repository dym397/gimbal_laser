import random
import socket
import threading
import time
import math
import struct
import json
import queue
from collections import deque

# ==========================================
# 0. 驱动引入
# ==========================================
try:
    from gimbal_interface import GT06ZAdapter
except ImportError as e:
    print(f"[System] 驱动接口加载失败: {e}")
    exit(1) # 驱动加载失败直接退出，防止后续报错
try:
    from mock_gimbal import MockGimbalAdapter
except ImportError:
    MockGimbalAdapter = None
# ==========================================
# 配置
# ==========================================
UI_IP = "192.168.1.200"
UI_PORT = 9999
LOCAL_PORT = 8888       
GIMBAL_PORT = "COM3"    # 请确认这是 GT06Z 实际连接的端口
LASER_PORT = "COM4"
USE_MOCK_GIMBAL = True  # True: 使用 mock_gimbal.py; False: 使用真实 GT06Z
ENABLE_IMU = False      # Manual switch: True to enable IMU read/print
IMU_PORT = "COM5"
IMU_BAUDRATE = 9600
IMU_PRINT_INTERVAL = 0.2
GIMBAL_CMD_DEADBAND_AZ = 0.3
GIMBAL_CMD_DEADBAND_EL = 0.2
GIMBAL_PREEMPT_DEG = 1.5
GIMBAL_SETTLE_THRESHOLD = 1.0
GIMBAL_SETTLE_TIMEOUT = 0.8
GIMBAL_THREAD_SLEEP = 0.02
LASER_DIST_TTL = 1.5

IMG_W = 3840.0
IMG_H = 2160.0
FOV_X = 17.5
FOV_Y = 9.9
DEG_PER_PIXEL_X = FOV_X / IMG_W
DEG_PER_PIXEL_Y = FOV_Y / IMG_H

# ==========================================
#  摄像头物理位置配置 (不变)
# ==========================================
DEVICE_THETA = {
    1: {"theta_vertical": 0.0, "theta_horizontal": 32.727},
    2: {"theta_vertical": 0.0, "theta_horizontal": 16.364},
    3: {"theta_vertical": 0.0, "theta_horizontal": 0.0},
    4: {"theta_vertical": 0.0, "theta_horizontal": 343.636},
    5: {"theta_vertical": 0.0, "theta_horizontal": 327.273},
    6: {"theta_vertical": 9.5, "theta_horizontal": 34.286},
    7: {"theta_vertical": 9.5, "theta_horizontal": 17.143},
    8: {"theta_vertical": 9.5, "theta_horizontal": 0.0},
    9: {"theta_vertical": 9.5, "theta_horizontal": 342.857},
    10: {"theta_vertical": 9.5, "theta_horizontal": 325.714},
    11: {"theta_vertical": 28.5, "theta_horizontal": 36.0},
    12: {"theta_vertical": 28.5, "theta_horizontal": 18.0},
    13: {"theta_vertical": 28.5, "theta_horizontal": 0.0},
    14: {"theta_vertical": 28.5, "theta_horizontal": 342.0},
    15: {"theta_vertical": 28.5, "theta_horizontal": 324.0},
    16: {"theta_vertical": 38.0, "theta_horizontal": 37.895},
    17: {"theta_vertical": 38.0, "theta_horizontal": 18.947},
    18: {"theta_vertical": 38.0, "theta_horizontal": 0.0},
    19: {"theta_vertical": 38.0, "theta_horizontal": 341.053},
    20: {"theta_vertical": 38.0, "theta_horizontal": 322.105},
    21: {"theta_vertical": 47.5, "theta_horizontal": 40.0},
    22: {"theta_vertical": 47.5, "theta_horizontal": 20.0},
    23: {"theta_vertical": 47.5, "theta_horizontal": 0.0},
    24: {"theta_vertical": 47.5, "theta_horizontal": 340.0},
    25: {"theta_vertical": 47.5, "theta_horizontal": 320.0},
    26: {"theta_vertical": 57.0, "theta_horizontal": 42.353},
    27: {"theta_vertical": 57.0, "theta_horizontal": 21.176},
    28: {"theta_vertical": 57.0, "theta_horizontal": 0.0},
    29: {"theta_vertical": 57.0, "theta_horizontal": 338.824},
    30: {"theta_vertical": 57.0, "theta_horizontal": 317.647},
    31: {"theta_vertical": 66.5, "theta_horizontal": 45.0},
    32: {"theta_vertical": 66.5, "theta_horizontal": 22.5},
    33: {"theta_vertical": 66.5, "theta_horizontal": 0.0},
    34: {"theta_vertical": 66.5, "theta_horizontal": 337.5},
    35: {"theta_vertical": 66.5, "theta_horizontal": 315.0},
    36: {"theta_vertical": 76.0, "theta_horizontal": 48.0},
    37: {"theta_vertical": 76.0, "theta_horizontal": 24.0},
    38: {"theta_vertical": 76.0, "theta_horizontal": 0.0},
    39: {"theta_vertical": 76.0, "theta_horizontal": 336.0},
    40: {"theta_vertical": 76.0, "theta_horizontal": 312.0}
}

# ==========================================
# 硬件映射表 (不变)
# ========================================== 
HARDWARE_MAP = {
    ("BOARD_1", 0): 1, ("BOARD_1", 1): 2, ("BOARD_1", 2): 3, ("BOARD_1", 3): 4, ("BOARD_1", 4): 5,
    ("BOARD_2", 0): 6, ("BOARD_2", 1): 7, ("BOARD_2", 2): 8, ("BOARD_2", 3): 9, ("BOARD_2", 4): 10,  
    ("BOARD_3", 0): 11, ("BOARD_3", 1): 12, ("BOARD_3", 2): 13, ("BOARD_3", 3): 14, ("BOARD_3", 4): 15,
    ("BOARD_4", 0): 16, ("BOARD_4", 1): 17, ("BOARD_4", 2): 18, ("BOARD_4", 3): 19, ("BOARD_4", 4): 20,
    ("BOARD_5", 0): 21, ("BOARD_5", 1): 22, ("BOARD_5", 2): 23, ("BOARD_5", 3): 24, ("BOARD_5", 4): 25,
    ("BOARD_6", 0): 26, ("BOARD_6", 1): 27, ("BOARD_6", 2): 28, ("BOARD_6", 3): 29, ("BOARD_6", 4): 30,
    ("BOARD_7", 0): 31 , ("BOARD_7", 1): 32, ("BOARD_7", 2): 33, ("BOARD_7", 3): 34, ("BOARD_7", 4): 35,
    ("BOARD_8", 0): 36, ("BOARD_8", 1): 37, ("BOARD_8", 2): 38, ("BOARD_8", 3): 39, ("BOARD_8", 4): 40,
}

# ==========================================
# 解析与计算函数
# ==========================================
def get_camera_params(board_id, cam_idx):
    key = (str(board_id), int(cam_idx))
    if key not in HARDWARE_MAP:
        print(f"[Warning] 未知的硬件组合: Board={board_id}, Cam={cam_idx}")
        return None, None
    logic_id = HARDWARE_MAP[key]
    if logic_id not in DEVICE_THETA:
        print(f"[Warning] 逻辑ID {logic_id} 没有配置偏差数据")
        return None, None
    cfg = DEVICE_THETA[logic_id]
    return logic_id, cfg

# ==========================================
# 网络发送类 (UI)
# ==========================================
class UISender:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.MSG_STATUS = 0x02 

    def send_status(self, board_str, camera_id, target_id, azimuth, elevation, distance):
            try:
                if not isinstance(board_str, str):
                    board_str = str(board_str)
                board_bytes = board_str.encode('utf-8')

                packet = struct.pack(
                    '!BB8sIfff',
                    self.MSG_STATUS,
                    int(camera_id),
                    board_bytes,
                    int(target_id),
                    float(azimuth),
                    float(elevation),
                    float(distance)
                )
                self.sock.sendto(packet, (self.ip, self.port))
            except Exception as e:
                print(f"[Sender] Error: {e}")

# ==========================================
# 3. 激光与网络
# ==========================================
class SharedHardwareState:
    def __init__(self):
        self.lock = threading.Lock()
        self.raw_laser_dist = 52.5
        self.valid_laser_dist = 52.5
        self.latest_dist = 52.5
        self.laser_ts = 0.0
        self.gimbal_az = 0.0  # UI坐标系方位角
        self.gimbal_el = 0.0
        self.gimbal_att_ts = 0.0
        self.active_cmd_id = -1
        self.settled_cmd_id = -1
        self.settled_ts = 0.0
        self.is_settled = False

shared_state = SharedHardwareState()
gimbal_cmd_queue = queue.Queue(maxsize=1)
packet_queue = deque(maxlen=20)

def rk3588_thread():
    print(f"[Net] Listening RK3588 on {LOCAL_PORT}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)

    try:
        sock.bind(("0.0.0.0", LOCAL_PORT))
    except OSError as e:
        print(f"[Net][Fatal] 端口 {LOCAL_PORT} 绑定失败: {e}")
        return

    err_decode = 0
    err_json = 0
    err_sock = 0
    err_other = 0

    while True:
        try:
            data, addr = sock.recvfrom(65535)
            pkg = json.loads(data.decode("utf-8"))
            if isinstance(pkg, dict) and "objs" in pkg:
                packet_queue.append(pkg)

        except socket.timeout:
            continue

        except UnicodeDecodeError as e:
            err_decode += 1
            if err_decode % 50 == 1:
                print(f"[Net][DecodeError] count={err_decode}, err={e}")
            continue

        except json.JSONDecodeError as e:
            err_json += 1
            if err_json % 50 == 1:
                print(f"[Net][JSONError] count={err_json}, err={e}")
            continue

        except OSError as e:
            err_sock += 1
            if err_sock % 10 == 1:
                print(f"[Net][SocketError] count={err_sock}, 网卡/底层异常: {e}")
            time.sleep(0.5)
            continue

        except Exception as e:
            err_other += 1
            print(f"[Net][Unexpected] count={err_other}, type={type(e).__name__}, err={e}")
            continue

def laser_thread_mock():
    # 这里如果您有了真实的激光驱动 sddm_laser.py，也可以替换掉 mock
    while True:
        with shared_state.lock:
            # 模拟激光测距
            shared_state.raw_laser_dist = 52.5
        time.sleep(0.05)

def push_latest_gimbal_cmd(cmd):
    while True:
        try:
            gimbal_cmd_queue.put_nowait(cmd)
            return True
        except queue.Full:
            try:
                gimbal_cmd_queue.get_nowait()
            except queue.Empty:
                return False

def drain_latest_gimbal_cmd():
    latest_cmd = None
    while True:
        try:
            latest_cmd = gimbal_cmd_queue.get_nowait()
        except queue.Empty:
            break
    return latest_cmd

def read_laser_distance():
    with shared_state.lock:
        dist = shared_state.raw_laser_dist
    if dist and dist > 0:
        return dist
    return 52.5
import numpy as np
from scipy.optimize import linear_sum_assignment

# ==========================================
# 新增模块 1：角度计算工具
# ==========================================
def angular_diff(target, source):
    """计算两个绝对角度之间的最短物理距离 (-180 到 180度)"""
    return (target - source + 180.0) % 360.0 - 180.0

def gimbal_control_thread(gimbal):
    """
    云台硬件专属线程：独占串口读写，支持可抢占执行。
    """
    print("[GimbalThread] 控制线程已启动")
    active_cmd = None#当前正在执行的指令
    target_az = 0.0
    target_el = 0.0
    cmd_start_t = 0.0

    while True:
        try:
            #当前没有指令在执行：
            if active_cmd is None:
                try:
                    active_cmd = gimbal_cmd_queue.get(timeout=0.1)
                #云台处于空闲状态则读取当前的一个静止姿态，并将结果更新到共享内存中，供主线程调度使用。
                except queue.Empty:
                    real_att = gimbal.get_attitude()
                    if real_att:
                        curr_el, curr_az, _ = real_att
                        curr_ui_az = (curr_az - 90.0) % 360.0
                        with shared_state.lock:
                            shared_state.gimbal_el = curr_el
                            shared_state.gimbal_az = curr_ui_az
                            shared_state.gimbal_att_ts = time.time()
                    continue
                #成功取到指令
                target_az = float(active_cmd["az"])
                target_el = float(active_cmd["el"])
                cmd_start_t = time.time()
                gimbal.set_attitude(elevation=target_el, azimuth=target_az)
                with shared_state.lock:
                    shared_state.active_cmd_id = int(active_cmd["cmd_id"])
                    shared_state.is_settled = False
            #当前有指令在执行
            now_t = time.time()
            newer_cmd = drain_latest_gimbal_cmd()
            if newer_cmd is not None:
                new_az = float(newer_cmd["az"])
                new_el = float(newer_cmd["el"])
                d_az = abs(angular_diff(new_az, target_az))
                d_el = abs(new_el - target_el)
                d_total = math.hypot(d_az, d_el)#计算新指令与当前指令的角度差

                if d_total > GIMBAL_PREEMPT_DEG:#如果角度差>预设的抢占阈值，则立即执行新指令
                    target_az = new_az
                    target_el = new_el
                    active_cmd = newer_cmd
                    cmd_start_t = now_t
                    gimbal.set_attitude(elevation=target_el, azimuth=target_az)
                    with shared_state.lock:
                        shared_state.active_cmd_id = int(active_cmd["cmd_id"])
                        shared_state.is_settled = False
                else:
                    active_cmd["cmd_id"] = int(newer_cmd["cmd_id"])
                    with shared_state.lock:
                        shared_state.active_cmd_id = int(active_cmd["cmd_id"])

            real_att = gimbal.get_attitude()
            if real_att:
                curr_el, curr_az, _ = real_att
                curr_ui_az = (curr_az - 90.0) % 360.0
                err_az = abs(angular_diff(target_az, curr_az))
                err_el = abs(curr_el - target_el)

                with shared_state.lock:
                    shared_state.gimbal_el = curr_el
                    shared_state.gimbal_az = curr_ui_az
                    shared_state.gimbal_att_ts = now_t

                if err_az < GIMBAL_SETTLE_THRESHOLD and err_el < GIMBAL_SETTLE_THRESHOLD:
                    with shared_state.lock:
                        shared_state.is_settled = True
                        shared_state.settled_cmd_id = shared_state.active_cmd_id
                        shared_state.settled_ts = now_t

                    laser_dist = read_laser_distance()
                    trigger_t = time.time()
                    with shared_state.lock:
                        prev_laser_ts = shared_state.laser_ts
                        shared_state.valid_laser_dist = laser_dist
                        shared_state.latest_dist = laser_dist
                        shared_state.laser_ts = trigger_t
                        active_cmd_id = shared_state.active_cmd_id
                    if prev_laser_ts > 0:
                        dt_laser = trigger_t - prev_laser_ts
                        print(f"[Laser] Triggered cmd_id={active_cmd_id}, dist={laser_dist:.2f}m, interval={dt_laser:.3f}s")
                    else:
                        print(f"[Laser] Triggered cmd_id={active_cmd_id}, dist={laser_dist:.2f}m, interval=first")
                    active_cmd = None
                    continue

            if (now_t - cmd_start_t) >= GIMBAL_SETTLE_TIMEOUT:
                with shared_state.lock:
                    shared_state.is_settled = False
                active_cmd = None
                continue

            time.sleep(GIMBAL_THREAD_SLEEP)

        except Exception as e:
            print(f"[GimbalThread][Unexpected] type={type(e).__name__}, err={e}")
            time.sleep(0.05)

def get_dynamic_tracking_params(distance_m):
    """
    根据目标距离生成动态参数（连续插值）。
    冷启动/无效测距时默认按 50m 处理。
    """
    if (distance_m is None) or (distance_m <= 0):
        distance_m = 50.0

    dist_nodes = np.array([50.0, 100.0, 200.0, 400.0, 500.0], dtype=float)
    d = float(np.clip(distance_m, dist_nodes[0], dist_nodes[-1]))

    max_res_az = float(np.interp(d, dist_nodes, [3.5, 2.0, 1.2, 0.8, 0.6]))
    max_vel_az = float(np.interp(d, dist_nodes, [40.0, 25.0, 15.0, 8.0, 5.0]))
    dist_thresh = float(np.interp(d, dist_nodes, [5.0, 3.0, 2.0, 1.5, 1.0]))

    return {
        "DIST_M": d,
        "MAX_RES_AZ": max_res_az,
        "MAX_RES_EL": max_res_az * 0.5,
        "MAX_VEL_AZ": max_vel_az,
        "MAX_VEL_EL": max_vel_az * 0.5,
        "DIST_THRESH": dist_thresh,
        "MIN_DT": 0.03,  # 15 FPS 场景下的保护下限
    }

class RangeSmoother:
    """激光测距平滑器：限速 + EMA，防止参数抖动。"""
    def __init__(self, init_d=50.0, alpha=0.2, max_rate_mps=30.0):
        self.d = float(init_d)
        self.alpha = float(alpha)
        self.max_rate_mps = float(max_rate_mps)

    def update(self, raw_d, dt):
        if (raw_d is None) or (raw_d <= 0):
            return self.d

        raw_d = float(np.clip(raw_d, 50.0, 500.0))
        dt_eff = max(float(dt), 0.03)
        max_step = self.max_rate_mps * dt_eff

        raw_d = max(self.d - max_step, min(self.d + max_step, raw_d))
        self.d = self.d + self.alpha * (raw_d - self.d)
        return self.d

def ui_to_ctrl_angles(ui_az, ui_el):
    """将绝对角度转为云台控制角度 (复用原先 calculate_angles 里的逻辑)"""
    rel_az = ui_az
    if rel_az > 180.0:
        rel_az -= 360.0
    ctrl_az = 90.0 + rel_az
    ctrl_el = ui_el
    if ctrl_az < 0.0: ctrl_az = 0.0
    if ctrl_az > 350.0: ctrl_az = 350.0
    return ctrl_az, ctrl_el

# ==========================================
# 新增模块 2：单目标卡尔曼追踪器 (AngleTracker)
# ==========================================
class Track:
    _id_count = 0
    def __init__(self, ui_az, ui_el):
        Track._id_count += 1
        self.id = Track._id_count
        
        # 状态: [方位角, 俯仰角, 方位角速度, 俯仰角速度]
        self.state = np.array([ui_az, ui_el, 0.0, 0.0])
        
        self.hit_streak = 1        # 连续命中次数 (用于建轨确认)
        self.time_since_update = 0 # 连续丢失次数 (用于注销)
        
        # 简化版卡尔曼增益 (Alpha-Beta 滤波参数)
        self.alpha = 0.6  # 位置信任度
        self.beta = 0.4   # 速度信任度
        self.max_res_az = 3.5
        self.max_res_el = 1.8
        self.max_vel_az = 40.0
        self.max_vel_el = 20.0
        self.min_dt = 0.03

    def set_dynamic_params(self, params):
        if not params:
            return
        self.max_res_az = float(params.get("MAX_RES_AZ", self.max_res_az))
        self.max_res_el = float(params.get("MAX_RES_EL", self.max_res_el))
        self.max_vel_az = float(params.get("MAX_VEL_AZ", self.max_vel_az))
        self.max_vel_el = float(params.get("MAX_VEL_EL", self.max_vel_el))
        self.min_dt = float(params.get("MIN_DT", self.min_dt))

    def predict(self, dt):
        """盲猜未来状态"""
        # 位置 = 老位置 + 速度 * 时间
        pred_az = (self.state[0] + self.state[2] * dt) % 360.0
        pred_el = self.state[1] + self.state[3] * dt
        self.state[0] = pred_az
        self.state[1] = pred_el
        self.time_since_update += 1 # 默认加1，如果匹配上了会在 update 里清零

    def update(self, meas_az, meas_el, dt):
        """融合真实测量值，修正速度和位置 (残差门控 + 速度限幅)"""
        self.time_since_update = 0
        self.hit_streak += 1

        if dt < self.min_dt:
            dt = self.min_dt

        raw_res_az = angular_diff(meas_az, self.state[0])
        raw_res_el = meas_el - self.state[1]

        res_az = max(min(raw_res_az, self.max_res_az), -self.max_res_az)
        res_el = max(min(raw_res_el, self.max_res_el), -self.max_res_el)

        inst_vx = res_az / dt
        inst_vy = res_el / dt

        self.state[2] = self.state[2] + self.beta * inst_vx
        self.state[3] = self.state[3] + self.beta * inst_vy

        self.state[2] = max(min(self.state[2], self.max_vel_az), -self.max_vel_az)
        self.state[3] = max(min(self.state[3], self.max_vel_el), -self.max_vel_el)

        self.state[0] = (self.state[0] + self.alpha * res_az) % 360.0
        self.state[1] = self.state[1] + self.alpha * res_el

    def get_future_position(self, dt_delay):
        """打提前量：获取未来预测角度"""
        fut_az = (self.state[0] + self.state[2] * dt_delay) % 360.0
        fut_el = self.state[1] + self.state[3] * dt_delay
        return fut_az, fut_el

# ==========================================
# 新增模块 3：多目标调度大脑 (MultiTargetTracker)
# ==========================================
class MultiTargetTracker:
    def __init__(self, max_lost_frames=10, distance_threshold=20.0):
        self.tracks = []
        self.max_lost_frames = max_lost_frames
        self.distance_threshold = distance_threshold # 最大匹配角度差

    def update(self, measurements, dt, params=None):
        """
        measurements: 当前帧所有检测到的绝对角度列表 [[az1, el1], [az2, el2], ...]
        dt: 距离上一帧经过的时间(秒)
        """
        if params and ("DIST_THRESH" in params):
            self.distance_threshold = float(params["DIST_THRESH"])

        # 1. 预测所有已有 Track 的新位置
        for track in self.tracks:
            track.set_dynamic_params(params)
            track.predict(dt)
            
        # 如果当前帧没检测到东西，直接清理丢失目标并返回
        if len(measurements) == 0:
            self.tracks = [t for t in self.tracks if t.time_since_update < self.max_lost_frames]
            return self.tracks

        if len(self.tracks) == 0:
            # 全是新目标
            for meas in measurements:
                t = Track(meas[0], meas[1])  # create kalman track for each measurement
                t.set_dynamic_params(params)
                self.tracks.append(t)
            return self.tracks

        # 2. 计算代价矩阵 (角度距离)
        cost_matrix = np.zeros((len(self.tracks), len(measurements)))
        for t, track in enumerate(self.tracks):
            for m, meas in enumerate(measurements):
                diff_az = angular_diff(meas[0], track.state[0])
                diff_el = meas[1] - track.state[1]
                distance = np.sqrt(diff_az**2 + diff_el**2)
                cost_matrix[t, m] = distance

        # 3. 匈牙利匹配
        track_indices, meas_indices = linear_sum_assignment(cost_matrix)

        # 4. 更新匹配成功的 Track
        unmatched_measurements = set(range(len(measurements)))
        for t_idx, m_idx in zip(track_indices, meas_indices):
            if cost_matrix[t_idx, m_idx] < self.distance_threshold:
                self.tracks[t_idx].update(measurements[m_idx][0], measurements[m_idx][1], dt)
                unmatched_measurements.discard(m_idx)
            else:
                # 距离太远，不认为是同一个目标
                pass

        # 5. 为没匹配上的坐标创建新 Track
        for m_idx in unmatched_measurements:
            meas = measurements[m_idx]
            t = Track(meas[0], meas[1])
            t.set_dynamic_params(params)
            self.tracks.append(t)

        # 6. 删除丢失太久的 Track
        self.tracks = [t for t in self.tracks if t.time_since_update < self.max_lost_frames]

        return self.tracks
# ==========================================
# 4. 核心解算 V8 (修改版：基准水平90度)
# ==========================================
def calculate_angles(cam_key, cx, cy, cfg=None):
    base_az = cfg["theta_horizontal"] 
    base_el = cfg["theta_vertical"]   
    
    # 1. 计算目标在图像中的像素偏移
    diff_x = cx - (IMG_W / 2.0)
    diff_y = cy - (IMG_H / 2.0)
    
    # 2. 像素转换成角度偏移
    offset_az = diff_x * DEG_PER_PIXEL_X
    offset_el = -diff_y * DEG_PER_PIXEL_Y 

    # 3. 计算系统绝对角度 (UI显示用, 保持0~360的罗盘习惯)
    ui_az = (base_az + offset_az) % 360.0
    ui_el = base_el + offset_el

    return ui_az, ui_el
# ==========================================
# 5. 云台到位检测 (针对真实串口优化)
# ==========================================
def wait_gimbal_settle(gimbal, target_el, target_az, threshold=0.5, timeout=0.8):
    start_t = time.time()
    target_az_norm = target_az % 360.0
    loop_cnt = 0 # 用于控制打印频率
    err_az = float("inf")
    
    while (time.time() - start_t) < timeout:
        real_att = gimbal.get_attitude() # 调用驱动 query_angles
        
        if not real_att:
            time.sleep(0.05)
            continue
            
        curr_el, curr_az, _ = real_att 
        curr_az_norm = curr_az % 360.0
        
        err_el = abs(curr_el - target_el)
        err_az = abs(curr_az_norm - target_az_norm)
        if err_az > 180:
            err_az = 360 - err_az
            
        # === 新增：Phase 4 循环追踪打印 (降频打印避免刷屏) ===
        if loop_cnt % 2 == 0:
            print(f"[Phase 4: 云台闭环] 等待到位... 目标[Az:{target_az:.1f}°] | 当前[Az:{curr_az:.1f}°] | 误差:{err_az:.1f}°")
        
        if err_el < threshold and err_az < threshold:
            print(f"[Phase 4: 云台闭环] 已到位! 耗时: {(time.time() - start_t):.3f}s")
            return True
            
        loop_cnt += 1
        time.sleep(0.05) # 降低查询频率，避免堵塞串口 Tx/Rx
        
    print(f"[Phase 4: 云台闭环] 等待超时 ({timeout}s)! 当前误差 Az:{err_az:.1f}° (物理云台还在移动追赶中)")
    return False

# ==========================================
# 6. 主逻辑 V9 (多目标预测与云台调度)
# ==========================================
def main():
    sender = UISender(UI_IP, UI_PORT)
    
    if USE_MOCK_GIMBAL:
        if MockGimbalAdapter is None:
            print("[Error] USE_MOCK_GIMBAL=True, but mock_gimbal.py import failed.")
            return
        print("[Init] Connecting to Mock Gimbal at MOCK_COM...")
        gimbal = MockGimbalAdapter(port="MOCK_COM")
    else:
        print(f"[Init] Connecting to Gimbal at {GIMBAL_PORT}...")
        gimbal = GT06ZAdapter(port=GIMBAL_PORT)
    if not gimbal.connect():
        print("[Error] Failed to connect gimbal.")
        return
    
    if not gimbal.wait_ready():
        print("[Warning] Gimbal not ready instantly, wait...")

    #start background threads for network and laser
    threading.Thread(target=rk3588_thread, daemon=True).start()
    threading.Thread(target=laser_thread_mock, daemon=True).start()
    threading.Thread(target=gimbal_control_thread, args=(gimbal,), daemon=True).start()

    imu = None
    last_imu_print = 0.0
    if ENABLE_IMU:
        try:
            from hwt905_driver import HWT905
            imu = HWT905(IMU_PORT, IMU_BAUDRATE)
            imu.open()
            print(f"[IMU] Enabled on {IMU_PORT}@{IMU_BAUDRATE}")
        except Exception as e:
            print(f"[IMU] Init failed: {e}")
            imu = None
    
    print("=== System V9.0 (Predictive Tracking & Scheduling) Running ===")

    # 初始化追踪大脑
    tracker = MultiTargetTracker(max_lost_frames=10, distance_threshold=4.0)
    range_smoother = RangeSmoother(init_d=50.0, alpha=0.2, max_rate_mps=30.0)
    
    # 状态机与调度变量
    master_id = None
    lock_timer = 0.0
    LOCK_DURATION = 1.5      # 锁定目标的最长驻留时间
    PREDICT_DELAY = 0.4      # 系统与物理响应总延迟 (打提前量)
    CONFIRM_HITS = 3         # 连续追踪多少帧才确认为合法目标
    MAX_DT = 0.5             # Clamp dt to avoid model divergence
    global_cmd_id = 0
    last_sent_ctrl_az = None
    last_sent_ctrl_el = None
    
    last_time = time.time()

    while True:
        try:
            curr_time = time.time()
            if imu and (curr_time - last_imu_print) >= IMU_PRINT_INTERVAL:
                acc, gyro, angle = imu.get_all()
                roll, pitch, yaw = angle
                print(f"ANGLE: {roll:6.2f} {pitch:6.2f} {yaw:6.2f}")
                last_imu_print = curr_time

            # --- 1. 获取 UDP 数据 (如果有) ---
            if not packet_queue:
                time.sleep(0.005) # 稍微让出 CPU
                continue
            dt = curr_time - last_time
            if dt <= 0:
                dt = 1.0 / 15.0
            if dt > MAX_DT:
                dt = MAX_DT
            last_time = curr_time

            with shared_state.lock:
                shared_gimbal_az = shared_state.gimbal_az
                shared_gimbal_el = shared_state.gimbal_el
                valid_laser_dist = shared_state.valid_laser_dist
                laser_ts = shared_state.laser_ts
            laser_for_params = valid_laser_dist if (curr_time - laser_ts) <= LASER_DIST_TTL else 52.5
            smooth_dist = range_smoother.update(laser_for_params, dt)
            dyn_params = get_dynamic_tracking_params(smooth_dist)
            
            while len(packet_queue) > 1: packet_queue.popleft()
            pkg = packet_queue.popleft()
            
            board_str = pkg.get("board", "Unknown") 
            cam_idx = int(pkg.get("cam", 0))
            raw_objs = pkg.get("objs", [])

            logic_id, cfg = get_camera_params(board_str, cam_idx)
            if logic_id is None: continue 
           
            # --- 2. 坐标解析为绝对角度 ---
            current_measurements = []
            for obj_item in raw_objs:
                if isinstance(obj_item, dict): rect = obj_item.get("box", [])
                else: rect = obj_item
                if len(rect) < 4: continue
                
                cx = (rect[0] + rect[2]) / 2.0
                cy = (rect[1] + rect[3]) / 2.0
                
                res = calculate_angles(logic_id, cx, cy, cfg)
                if res:
                    ui_az, ui_el = res
                    current_measurements.append([ui_az, ui_el])
                    # === 新增：Phase 1 打印 ===
                    print(f"\n[Phase 1: 视觉解析] 收到目标 cx={cx:.1f}, cy={cy:.1f} -> 解算绝对角度: Az={ui_az:.2f}°, El={ui_el:.2f}°")
            # --- 3. 喂给 Tracker 更新所有目标轨迹 ---
            active_tracks = tracker.update(current_measurements, dt, params=dyn_params)
            
            # 过滤出合法的、可以被锁定的目标 (连续追踪超过 CONFIRM_HITS 次的)且没有丢失的目标 (time_since_update == 0)
            valid_tracks = [
                t for t in active_tracks
                if t.hit_streak >= CONFIRM_HITS and t.time_since_update == 0 
            ]

            # --- 4. 状态机：调度决策 ---
            # 检查当前跟踪的目标是否已经丢失
            master_track = next((t for t in valid_tracks if t.id == master_id), None)
            
            if master_track is None or lock_timer <= 0:
                # 状态 A：寻找/切换新目标 (SEARCHING)
                master_id = None
                if len(valid_tracks) > 0:
                    # 策略：找当前距离云台物理角度最近的目标
                    curr_gimbal_az = shared_gimbal_az
                    curr_gimbal_el = shared_gimbal_el
                    
                    best_track = None
                    min_dist = float('inf')
                    for t in valid_tracks:
                        dist_az = angular_diff(t.state[0], curr_gimbal_az)
                        dist_el = t.state[1] - curr_gimbal_el
                        dist = np.sqrt(dist_az**2 + dist_el**2)
                        if dist < min_dist:
                            min_dist = dist
                            best_track = t
                            
                    if best_track:
                        master_id = best_track.id
                        lock_timer = LOCK_DURATION
                        master_track = best_track
                        # print(f"[Scheduler] 切换锁定目标 ID: {master_id}, 倒计时重置 {LOCK_DURATION}s")
                        # === 修改：Phase 2 打印 ===
                        print(f"[Phase 2: 目标调度] 成功锁定目标 ID: {master_id} (已连续追踪 {master_track.hit_streak} 帧). 准备交由云台跟踪!")
            # --- 5. 状态机：物理执行与测距 (LOCKED) ---
            if master_track is not None:
                lock_timer -= dt
                
                # A. 提取提前量预测角度
                fut_az, fut_el = master_track.get_future_position(dt_delay=PREDICT_DELAY)
                
                # B. 转换为云台控制角
                ctrl_az, ctrl_el = ui_to_ctrl_angles(fut_az, fut_el)
                # === 新增：Phase 3 打印 ===
                print(f"[Phase 3: 预测控制] 目标当前估算Az={master_track.state[0]:.2f}°, 速度={master_track.state[2]:.2f}°/s")
                print(f"                   -> 打提前量({PREDICT_DELAY}s后)Az={fut_az:.2f}°, El={fut_el:.2f}° | 下发云台指令: Az={ctrl_az:.2f}°, El={ctrl_el:.2f}°")
                # C. 非阻塞下发：只推送最新控制指令给云台线程
                need_send = True
                if (last_sent_ctrl_az is not None) and (last_sent_ctrl_el is not None):
                    d_az = abs(angular_diff(ctrl_az, last_sent_ctrl_az))
                    d_el = abs(ctrl_el - last_sent_ctrl_el)
                    if d_az < GIMBAL_CMD_DEADBAND_AZ and d_el < GIMBAL_CMD_DEADBAND_EL:
                        need_send = False

                if need_send:
                    global_cmd_id += 1
                    push_latest_gimbal_cmd({
                        "cmd_id": global_cmd_id,
                        "az": ctrl_az,
                        "el": ctrl_el,
                        "ts": curr_time,
                    })
                    last_sent_ctrl_az = ctrl_az
                    last_sent_ctrl_el = ctrl_el

                with shared_state.lock:
                    laser_dist_now = shared_state.valid_laser_dist
                    laser_ts_now = shared_state.laser_ts
                master_laser_dist = laser_dist_now if (curr_time - laser_ts_now) <= LASER_DIST_TTL else 52.5
                
                # E. 向 UI 发送数据包 (遍历所有合法的追踪档案)
                # 这样即使云台在打 ID 1，UI 上也能看到 ID 2, 3 的平滑轨迹
                for t in valid_tracks:
                    send_dist = master_laser_dist if (t.id == master_id) else 52.5
                    
                    sender.send_status(
                        board_str, cam_idx, t.id,  # 这里传入的是持续追踪的 ID，而不是一闪而过的数组下标
                        azimuth=t.state[0],        # 发送卡尔曼平滑后的位置
                        elevation=t.state[1], 
                        distance=send_dist
                    )

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Err: {e}")
            import traceback
            traceback.print_exc()

    if imu:
        imu.close()
    if 'gimbal' in locals():
        gimbal.close()

if __name__ == "__main__":
    main()
