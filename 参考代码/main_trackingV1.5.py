
import socket
import threading
import time
import math
import sys
import json 

# ==========================================
# 0. 驱动引入 (已解耦)
# ==========================================
try:
    # ★ 核心修改：引入适配器，而不是具体驱动
    from gimbal_interface import XFGimbalAdapter
    
    # 激光驱动暂时保持原样 (如果需要也可以按同样方式解耦)
    from sddm_laser import SDDMLaser
except ImportError as e:
    print(f"错误: 驱动加载失败 - {e}")
    sys.exit(1)
# ==========================================
# 全局工具函数
# ==========================================
def clamp(val, low, high):
    """通用限幅函数"""
    return max(low, min(val, high))
# ==========================================
# 1. 全局配置参数
# ==========================================

# --- 端口配置 ---
GIMBAL_PORT = "COM3"   # 实际使用请确认
LASER_PORT = "COM4"    # 实际使用请确认

# --- 平滑控制参数 ---
MAX_STEP_PER_LOOP = 0.5 

# --- 网络配置 ---
UDP_IP = "0.0.0.0"
UDP_PORT = 8888
DATA_TIMEOUT = 0.5

# --- 图像参数 (4K) ---
IMG_W = 3840.0
IMG_H = 2160.0
CENTER_X = IMG_W / 2.0
CENTER_Y = IMG_H / 2.0

# --- 镜头参数 ---
FOV_X = 90.0
FOV_Y = 50.0  
DEG_PER_PIXEL_X = FOV_X / IMG_W
DEG_PER_PIXEL_Y = FOV_Y / IMG_H

# --- 物理参数 ---
DEFAULT_DISTANCE_M = 50.0
PRINT_INTERVAL = 2.0   # 打印间隔 (秒)

# --- 摄像头阵列角度偏移表 (ID 0-8) ---
CAMERA_OFFSETS = {
    # 第一排 (抬头, 离云台最近)
    0: ( 16.4, -84.6),   1: (  0.0, -84.6),   2: (-16.4, -84.6),
    # 第二排 (平视, 标准间距)
    3: ( 16.4, -90.0),   4: (  0.0, -90.0),   5: (-16.4, -90.0),
    # 第三排 (低头, 离云台最远)
    6: ( 16.4, -95.4),   7: (  0.0, -95.4),   8: (-16.4, -95.4),
}

# ==========================================
# 2. 共享数据结构 (线程安全)
# ==========================================
class SharedData:
    def __init__(self):
        self.lock = threading.Lock()
        
        # --- 视觉部分 ---
        self.target_valid = False
        self.cam_id = 4
        self.cx = CENTER_X
        self.cy = CENTER_Y
        self.target_ts = 0.0  # 时间戳
        
        # --- 激光部分 ---
        self.dist_m = DEFAULT_DISTANCE_M
        self.dist_ts = 0.0    # 时间戳

shared_state = SharedData()
program_running = True

# ==========================================
# 3. 线程 A：UDP 接收 (JSON协议)
# ==========================================
def udp_server_thread():
    global program_running
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)
    print(f"[Net] UDP 监听启动: {UDP_PORT}")

    while program_running:
        try:
            try:
                data_bytes, addr = sock.recvfrom(4096)
            except BlockingIOError:
                time.sleep(0.002)
                continue

            # JSON 解析
            text = data_bytes.decode('utf-8').strip()
            pkg = json.loads(text)
            
            cid = int(pkg.get("id", -1))
            if cid not in CAMERA_OFFSETS: continue
            
            objs = pkg.get("objs", [])
            if len(objs) > 0:
                rect = objs[0]
                if len(rect) >= 4:
                    # 计算中心点
                    cx = (float(rect[0]) + float(rect[2])) / 2.0
                    cy = (float(rect[1]) + float(rect[3])) / 2.0

                    with shared_state.lock:
                        shared_state.target_valid = True
                        shared_state.cam_id = cid
                        shared_state.cx = cx
                        shared_state.cy = cy
                        shared_state.target_ts = time.time()
        except Exception:
            pass 
    sock.close()

# ==========================================
# 4. 线程 B：激光读取 (防阻塞)
# ==========================================
def laser_reader_thread():
    global program_running
    print(f"[Laser] 线程启动: {LASER_PORT}")
    
    try:
        laser = SDDMLaser(LASER_PORT)
        # 初始化：停止 -> 清缓存 -> 连测
        laser.stop_measurement()
        time.sleep(0.2)
        laser.ser.reset_input_buffer()
        laser.start_measurement(continuous=True)
    except Exception as e:
        print(f"[Laser] 初始化失败: {e}")
        return

    while program_running:
        try:
            # 阻塞式读取，但在独立线程中不影响主程序
            dist = laser.read_distance(debug=False) 
            
            if dist is not None:
                with shared_state.lock:
                    if dist > 0.2:
                        shared_state.dist_m = dist
                        shared_state.dist_ts = time.time()
                    elif dist < 0:
                        # 盲区或无效值，使用安全默认值
                        shared_state.dist_m = 50.66
            
            time.sleep(0.01) # 稍微让出CPU

        except Exception as e:
            print(f"[Laser] 读取错误: {e}")
            time.sleep(1.0) # 出错后暂停

    if 'laser' in locals(): laser.close()
    print("[Laser] 线程停止")

# ==========================================
# 5. 核心解算函数 (动态高度)
# ==========================================
def calculate_destination_angles(cam_id, target_cx, target_cy, distance_m):
    # 1. 获取基准角度
    base_roll, base_pitch = CAMERA_OFFSETS.get(cam_id, CAMERA_OFFSETS[4])

    # 2. 计算视觉相对偏差
    diff_x = target_cx - CENTER_X
    diff_y = target_cy - CENTER_Y
    local_roll = -1.0 * diff_x * DEG_PER_PIXEL_X
    local_pitch = -1.0 * diff_y * DEG_PER_PIXEL_Y

    # --- 3. 动态确定视差高度 ---
    if cam_id < 3:
        current_h_offset = 0.10 # 上排 (ID 0-2)
    elif cam_id > 5:
        current_h_offset = 0.20 # 下排 (ID 6-8)
    else:
        current_h_offset = 0.15 # 中排 (ID 3-5)

    # 4. 计算视差补偿角
    parallax_deg = 0.0
    if distance_m > 0.1: 
        # atan(高度差 / 距离)
        parallax_deg = math.degrees(math.atan(current_h_offset / distance_m))

    # 5. 合成最终指令
    dest_roll = base_roll + local_roll
    dest_pitch = base_pitch + local_pitch - parallax_deg

    return clamp(dest_roll, -45.0, 45.0), clamp(dest_pitch, -115.0, 0.0)

# ==========================================
# 6. 主程序 (控制流)
# ==========================================
def main():
    global program_running
    
    # --- 1. 硬件连接 (使用通用接口) ---
    gimbal = None
    try:
        # ★ 实例化适配器 (解耦点：以后换云台只需改这里为 DJIGimbalAdapter)
        gimbal = XFGimbalAdapter(GIMBAL_PORT)
        gimbal.connect() # 标准连接
    except Exception as e:
        print(f"云台连接失败: {e}")
        return

    print("等待云台就绪...")
    # ★ 标准等待
    if not gimbal.wait_ready(timeout=30): return

    # --- 启动后台线程 ---
    t_udp = threading.Thread(target=udp_server_thread, daemon=True)
    t_udp.start()               
    
    # t_laser = threading.Thread(target=laser_reader_thread, daemon=True)
    # t_laser.start()
    
    # --- 归位 ---
    print("云台归位...")
    current_cmd_roll = 0.0
    current_cmd_pitch = -90.0
    
    # ★ 标准控制
    gimbal.set_attitude(pitch=-90, roll=0)
    time.sleep(2.0)

    print(f"\n=== 系统运行中 (V1.5: 硬件解耦版) ===\n")
    
    LOOP_PERIOD = 1.0 / 50.0 # 50Hz 控制频率
    last_print_time = time.time()

    try:
        next_loop_time = time.perf_counter()

        while True:
            # ========================
            # Step A: 获取最新数据
            # ========================
            has_target = False
            t_id, t_cx, t_cy = 4, CENTER_X, CENTER_Y
            curr_dist = DEFAULT_DISTANCE_M
            
            with shared_state.lock:
                if shared_state.target_valid:
                    if (time.time() - shared_state.target_ts) < DATA_TIMEOUT:
                        has_target = True
                        t_id = shared_state.cam_id
                        t_cx = shared_state.cx
                        t_cy = shared_state.cy
                    else:
                        shared_state.target_valid = False
                curr_dist = shared_state.dist_m

            # ========================
            # Step B: 融合解算
            # ========================
            if has_target:
                dest_roll, dest_pitch = calculate_destination_angles(
                    t_id, t_cx, t_cy, curr_dist
                )
            else:
                dest_roll, dest_pitch = CAMERA_OFFSETS[4]
                curr_dist = DEFAULT_DISTANCE_M

            # ========================
            # Step C: 平滑插值
            # ========================
            diff_roll = dest_roll - current_cmd_roll
            current_cmd_roll += clamp(diff_roll, -MAX_STEP_PER_LOOP, MAX_STEP_PER_LOOP)
            
            diff_pitch = dest_pitch - current_cmd_pitch
            current_cmd_pitch += clamp(diff_pitch, -MAX_STEP_PER_LOOP, MAX_STEP_PER_LOOP)

            # ========================
            # Step D: 执行控制 (使用通用接口)
            # ========================
            # ★ 这里的 set_attitude 在测试模式下会尝试回读状态，read_status=True
            gimbal.set_attitude(pitch=current_cmd_pitch, roll=current_cmd_roll, read_status=True)

            # ========================
            # Step E: 状态监控
            # ========================
            if time.time() - last_print_time > PRINT_INTERVAL:
                state_str = "TRK" if has_target else "IDLE"
                
                # ★ 标准反馈读取
                real_angles = gimbal.get_attitude()
                
                if real_angles:
                    # 标准返回格式 (roll, pitch, yaw)
                    r_real, p_real, y_real = real_angles
                    print(f"[{state_str}] Cam:{t_id} | Dist:{curr_dist:.1f}m | "
                          f"Roll: {current_cmd_roll:.1f}->{r_real:.1f} | "
                          f"Pitch: {current_cmd_pitch:.1f}->{p_real:.1f}")
                else:
                    print(f"[{state_str}] Cam:{t_id} | Dist:{curr_dist:.1f}m | "
                          f"CMD R:{current_cmd_roll:.1f} P:{current_cmd_pitch:.1f} (No FB)")
                
                last_print_time = time.time()

            # ========================
            # Step F: 50Hz 控频
            # ========================
            now = time.perf_counter()
            sleep_time = next_loop_time + LOOP_PERIOD - now
            if sleep_time > 0:
                time.sleep(sleep_time)
                next_loop_time += LOOP_PERIOD
            else:
                next_loop_time = now

    except KeyboardInterrupt:
        print("\n停止...")
    finally:
        program_running = False
        if gimbal: 
            gimbal.close() # ★ 标准关闭

if __name__ == "__main__":
    main()