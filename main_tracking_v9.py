import socket
import threading
import time
import math
import struct
import json
from collections import deque

# ==========================================
# 0. 驱动引入
# ==========================================
try:
    from gimbal_interface import GT06ZAdapter
except ImportError as e:
    print(f"[System] 驱动接口加载失败: {e}")
    exit(1) # 驱动加载失败直接退出，防止后续报错

# ==========================================
# 配置
# ==========================================
UI_IP = "192.168.1.200"
UI_PORT = 9999
LOCAL_PORT = 8888       
GIMBAL_PORT = "COM3"    # 请确认这是 GT06Z 实际连接的端口
LASER_PORT = "COM4"

IMG_W = 3840.0
IMG_H = 2160.0
FOV_X = 90.0
FOV_Y = 50.0
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
class SharedData:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_dist = 0.0

shared_state = SharedData()
packet_queue = deque(maxlen=20)

def rk3588_thread():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", LOCAL_PORT))
    print(f"[Net] Listening RK3588 on {LOCAL_PORT}...")
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            pkg = json.loads(data.decode('utf-8'))
            if "objs" in pkg:
                packet_queue.append(pkg)
        except:
            pass

def laser_thread_mock():
    # 这里如果您有了真实的激光驱动 sddm_laser.py，也可以替换掉 mock
    while True:
        with shared_state.lock:
            # 模拟激光测距
            shared_state.latest_dist = 52.5 
        time.sleep(0.05)

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

    # ========================================================
    # [修改点] 云台控制角度解算
    # 目标：以水平90度、垂直0度为基准中心
    # ========================================================
    
    # A. 将 0~360 的绝对角度转换为相对于正前方(0度)的 -180~180 角度
    #    例如：350度 -> -10度(左); 10度 -> 10度(右)
    rel_az = ui_az
    if rel_az > 180.0:
        rel_az -= 360.0
        
    # B. 应用云台基准偏移
    #    基准为90度。左转(负值)则减，右转(正值)则加。
    #    公式：Control = 90 + Relative_Angle
    ctrl_az = 90.0 + rel_az
    
    #    垂直基准为0度，直接透传绝对仰角即可 (假设云台0度即平视)
    ctrl_el = ui_el

    # C. 边界与负数保护
    #    云台只能接受正数，且您提到物理上保证了 90-x > 0
    #    这里加个max/min作为双重保险
    if ctrl_az < 0.0: ctrl_az = 0.0
    if ctrl_az > 350.0: ctrl_az = 350.0

    return ui_az, ui_el, ctrl_az, ctrl_el
# ==========================================
# 5. 云台到位检测 (针对真实串口优化)
# ==========================================
def wait_gimbal_settle(gimbal, target_el, target_az, threshold=0.5, timeout=0.8):
    """
    等待云台运动到指定位置。
    注意：真实串口查询(query_angles)是有耗时的(单次约20ms+)，
    因此不要查询得太频繁，这里将 sleep 增加到 0.05
    """
    start_t = time.time()
    
    # 归一化目标 Azimuth 到 0-360 以便比较
    target_az_norm = target_az % 360.0
    
    while (time.time() - start_t) < timeout:
        real_att = gimbal.get_attitude() # 调用驱动 query_angles
        
        if not real_att:
            time.sleep(0.05)
            continue
            
        curr_el, curr_az, _ = real_att 
        
        # 归一化当前 Azimuth
        curr_az_norm = curr_az % 360.0
        
        # 计算偏差
        err_el = abs(curr_el - target_el)
        
        # 水平角度偏差（处理0/360跳变问题）
        err_az = abs(curr_az_norm - target_az_norm)
        if err_az > 180:
            err_az = 360 - err_az
        
        if err_el < threshold and err_az < threshold:
            return True
            
        time.sleep(0.05) # 降低查询频率，避免堵塞串口 Tx/Rx
        
    return False

# ==========================================
# 6. 主逻辑 V8
# ==========================================
def main():
    sender = UISender(UI_IP, UI_PORT)
    
    # --- 替换 Mock，使用真实驱动 ---
    print(f"[Init] Connecting to Gimbal at {GIMBAL_PORT}...")
    gimbal = GT06ZAdapter(port=GIMBAL_PORT)
    
    if not gimbal.connect():
        print("[Error] Failed to connect gimbal.")
        return
    
    if not gimbal.wait_ready():
        print("[Warning] Gimbal not ready instantly, wait...")

    threading.Thread(target=rk3588_thread, daemon=True).start()
    threading.Thread(target=laser_thread_mock, daemon=True).start()
    
    print("=== System V9.0 (GT06Z Real Driver) Running ===")

    master_ctrl_az = None
    master_ctrl_el = None
    master_laser_dist = 0.0

    while True:
        try:
            if not packet_queue:
                time.sleep(0.001)
                continue
            
            while len(packet_queue) > 1: packet_queue.popleft()
            pkg = packet_queue.popleft()
            
            board_str = pkg.get("board", "Unknown") 
            cam_idx = int(pkg.get("cam", 0))
            raw_objs = pkg.get("objs", [])

            logic_id, cfg = get_camera_params(board_str, cam_idx)
            if logic_id is None: continue # 硬件映射失败则跳过
           
            master_ctrl_az = None
            master_ctrl_el = None
            master_laser_dist = 0.0
            
            for t_id, obj_item in enumerate(raw_objs):
                if isinstance(obj_item, dict):
                    rect = obj_item.get("box", [])
                else:
                    rect = obj_item
                
                if len(rect) < 4: continue
                
                cx = (rect[0] + rect[2]) / 2.0
                cy = (rect[1] + rect[3]) / 2.0
                
                res = calculate_angles(logic_id, cx, cy, cfg)
                if not res: continue
                
                ui_az, ui_el, ctrl_az, ctrl_el = res
                send_dist = 0.0
                
                # Case 1: 主目标 (Target 0) -> 驱动云台
                if t_id == 0:
                    # 1. 发送运动指令
                    gimbal.set_attitude(elevation=ctrl_el, azimuth=ctrl_az)
                    
                    # 2. 等待到位 (会阻塞约 0.1~0.8秒，取决于运动幅度)
                    # 如果不需要严格确认到位才测距，可以去掉这个 if check，直接测距
                    is_settled = wait_gimbal_settle(gimbal, target_el=ctrl_el, target_az=ctrl_az, threshold=1.0)
                    
                    if is_settled:
                        with shared_state.lock:
                            # 只有云台稳了，激光的数据才是打在目标上的
                            master_laser_dist = shared_state.latest_dist
                    else:
                        master_laser_dist = 52.5
                    
                        send_dist = master_laser_dist
                        master_ctrl_az = ctrl_az
                        master_ctrl_el = ctrl_el
                    
                    # print(f"[T0] {board_str}-{cam_idx} | Tgt: {ctrl_az:.1f}, {ctrl_el:.1f} | Dist: {send_dist}")

                # Case 2: 从目标 (Ghost Target)
                else:
                    is_ghost = False
                    if master_ctrl_az is not None:
                        diff_az = abs(ctrl_az - master_ctrl_az)
                        diff_el = abs(ctrl_el - master_ctrl_el)
                        # 如果从目标和主目标角度非常接近，认为激光打到的也是它
                        if diff_az < 0.5 and diff_el < 0.5:
                            is_ghost = True
                    
                    if is_ghost:
                        send_dist = master_laser_dist 
                    else:
                        send_dist = 52.5

                sender.send_status(
                    board_str, cam_idx, t_id,
                    azimuth=ui_az, 
                    elevation=ui_el, 
                    distance=send_dist
                )

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Err: {e}")
            import traceback
            traceback.print_exc()

    if 'gimbal' in locals(): gimbal.close()

if __name__ == "__main__":
    main()