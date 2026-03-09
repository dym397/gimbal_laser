from dataclasses import dataclass
import struct
from typing import Tuple,Optional
import threading

try:
    import serial  # 可选，用于串口收发
except ImportError:
    serial = None


# -------------------- 目标状态定义 --------------------


@dataclass
class GimbalTarget:
    """云台期望状态"""
    pitch_deg: float = 0.0      # 俯仰角 (Pitch) - 对应 gbc[1]
    roll_deg: float = 0.0       # 滚转角 (Roll)  - 对应 gbc[0]
    
    # C-40D 没有 Yaw 轴，所以不需要 yaw_deg 字段，或者保留但不使用
    
    pitch_go_zero: int = 0      # 俯仰轴回中触发位
    roll_go_zero: int = 0       # 滚转轴回中触发位 


# -------------------- 工具函数 --------------------


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def calculate_crc16_xf(data: bytes) -> int:
    crc_table = (
        0x0000, 0x1021, 0x2042, 0x3063,
        0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B,
        0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    )
    crc = 0
    for b in data:
        da = crc >> 12
        crc = ((crc << 4) & 0xFFFF) ^ crc_table[da ^ (b >> 4)]
        da = crc >> 12
        crc = ((crc << 4) & 0xFFFF) ^ crc_table[da ^ (b & 0x0F)]
    return crc & 0xFFFF


def _pack_axis(go_zero: int, wk_mode: int, op_type: int, op_value: int) -> bytes:
    """
    打包单轴的 gbc 结构：模式字节 + int16 控制量。:contentReference[oaicite:2]{index=2}

    字节布局（高位在左）：
    [7:6] op_type   控制模式
    [5:4] wk_mode   工作模式
    [3]   go_zero   回中触发位
    [2:0] 保留
    """
    header = ((op_type & 0x03) << 6) | ((wk_mode & 0x03) << 4) | ((go_zero & 0x01) << 3)
    return bytes([header]) + struct.pack("<h", int(op_value))

def hex_dump(frame):
    print(' '.join(f'{b:02X}' for b in frame))


# -------------------- 核心：从状态组包 --------------------

def build_packet_from_target(
    target: GimbalTarget,
    *,
    cmd_value: int = 4,
    trig: int = 0,
    fl_sens: int = 0,
) -> bytes:
    """
    根据当前 GimbalTarget 生成一帧完整的 A9 5B 协议数据（含 CRC）。:contentReference[oaicite:3]{index=3}

    约定（跟你现在需求对齐）：
    - cmd.value = 4：手动控制
    - gbc[0] 水平轴/yaw：FPV + 角度控制, 目标角 = yaw_deg
    - gbc[1] 俯仰轴/pitch：FPV + 角度控制，目标角 = pitch_deg
    - gbc[2] 无效
    - uav.valid = 0，姿态角 / 加速度全部置 0（你现在没有载机 IMU，按文档建议置 0）:contentReference[oaicite:4]{index=4}
    - cam 部分全部 0（不做变焦、不做指点平移）:contentReference[oaicite:5]{index=5}
    """

    # ---------- 1. 协议头 ----------
    buf = bytearray()
    buf.extend((0xA9, 0x5B))  # sync[0], sync[1] :contentReference[oaicite:6]{index=6}

    # ---------- 2. 命令字节 cmd ----------
    # [7:3] 命令码 value, [2:0] 触发计数 trig。示例中 0x20 = 0b00100_000，对应 value=4, trig=0。:contentReference[oaicite:7]{index=7}
    cmd_byte = ((cmd_value & 0x1F) << 3) | (trig & 0x07)
    buf.append(cmd_byte)

    # ---------- 3. 辅助字节 aux ----------
    # [7:3] fl_sens（有符号 5bit），[2:0] 保留。我们先用 0。:contentReference[oaicite:8]{index=8}
    fl_sens = int(clamp(fl_sens, -16, 15)) & 0x1F
    aux_byte = fl_sens << 3
    buf.append(aux_byte)

    # ---------- 4. 三轴 gbc[3] ----------
    # 约定：gbc[0]=roll, 1=pitch, 2=不知道是什么。:contentReference[oaicite:9]{index=9}

    # 4.1 滚转轴/roll：锁定 + 角度控制，单位 0.01deg，范围 [-45, 45]。
    # 4.1 gbc[0] -> 滚转轴 (Roll) [红色箭头]
    # 功能: 控制左右歪头。追踪时通常设为 0 以保持水平。
    # 修正: 原代码误以此为 yaw，现更正为 roll
    roll_deg = clamp(target.roll_deg, -45.0, 45.0)
    roll_val = int(round(roll_deg * 100.0))
    buf += _pack_axis(
        go_zero=target.roll_go_zero, 
        wk_mode=2,       # 锁定模式
        op_type=0,       # 角度控制
        op_value=roll_val
    )

    # 4.2 俯仰轴/pitch：锁定 + 角度控制，单位 0.01deg，范围 [-135, 135]。:contentReference[oaicite:10]{index=10}
    pitch_deg = clamp(target.pitch_deg, -135.0, 135.0)
    pitch_val = int(round(pitch_deg * 100.0))  # 0.01deg
    buf += _pack_axis(
        go_zero=0,
        wk_mode=2,  # 锁定模式              
        op_type=0,  # 角度控制
        op_value=pitch_val,
    )

    # 4.3 gbc[2] -> 偏航轴 (Yaw) [不存在]
    # C-40D 为两轴云台，此轴无效，固定发 0
    # yaw_deg = clamp(target.yaw_deg, -180.0, 180.0)
    # yaw_val = int(round(yaw_deg * 100.0))
    buf += _pack_axis(
        go_zero=0,
        wk_mode=2,  # 
        op_type=0,  # 角度控制
        op_value=0,
    )

    # 此时长度应为：2 + 1 + 1 + 3*3 = 13 字节
    assert len(buf) == 13, f"gbc 打包长度异常: {len(buf)}"

    # ---------- 5. 载机数据 uav ----------
    # [7] valid, [6:0] 保留。没载机 IMU，就按文档要求置 0。:contentReference[oaicite:12]{index=12}
    uav_valid = 0
    uav_header = (uav_valid & 0x01) << 7
    buf.append(uav_header)

    # angle[3] + accel[3]，全部 int16 小端，单位 0.01deg / 0.01m/s^2。这里全 0。:contentReference[oaicite:13]{index=13}
    buf += struct.pack("<hhhhhh", 0, 0, 0, 0, 0, 0)

    # 到这里总长度应为：13 + 1 + 12 = 26
    assert len(buf) == 26, f"uav 打包长度异常: {len(buf)}"

    # ---------- 6. 相机 cam ----------
    # 32bit：vert_fov1x(7) + zoom_value(24) + reserved(1)，我们全 0。:contentReference[oaicite:14]{index=14}
    cam_word = 0
    buf += struct.pack("<I", cam_word)

    # target_angle[2]：float，小端，单位 1deg，这里也全 0。:contentReference[oaicite:15]{index=15}
    buf += struct.pack("<ff", 0.0, 0.0)

    # 目前长度应为 26 + 4 + 8 = 38（不含 CRC）
    assert len(buf) == 38, f"数据区长度异常(不含CRC): {len(buf)}"

    # ---------- 7. CRC16（高字节在前） ----------
    crc = calculate_crc16_xf(bytes(buf))
    buf.append((crc >> 8) & 0xFF)  # 高字节
    buf.append(crc & 0xFF)         # 低字节

    # 最终整帧长度应为 40 字节，与文档示例一致。:contentReference[oaicite:16]{index=16}
    assert len(buf) == 40, f"完整数据包长度异常: {len(buf)}"

    return bytes(buf)



def open_gimbal_port(port: str, baudrate: int = 115200):
    """打开云台串口（需要安装 pyserial）"""
    if serial is None:
        raise RuntimeError("需要先安装 pyserial：pip install pyserial")
    return serial.Serial(port=port, baudrate=baudrate, bytesize=8,
                         parity="N", stopbits=1, timeout=0.05)


def send_gimbal_packet(ser, packet: bytes):
    """通过串口发送一帧数据，并打印 hex 用于调试"""
    ser.write(packet)
    ser.flush()
    # hex_str = " ".join(f"{b:02X}" for b in packet)
    # print(f"SEND: {hex_str}")

# -------------------- 云台返回帧解析（B5 9A） --------------------
@dataclass
class GimbalStatus:
    """对应私有协议中的 Gbc2GcuPkt_t 一帧返回数据"""
    fw_ver: int
    hw_err: int
    inv_flag: int
    gbc_stat: int
    tca_flag: int
    cmd_stat: int
    cmd_value: int
    cam_rate: Tuple[int, int, int]
    cam_angle: Tuple[int, int, int]
    mtr_angle: Tuple[int, int, int]


def _parse_gimbal_status_frame(frame: bytes) -> Optional[GimbalStatus]:
    """
    解析一帧完整的 B5 9A 返回包（长度 26 字节）。
    成功返回 GimbalStatus，失败返回 None。
    """
    if len(frame) != 26:
        print(f"[GIMBAL] recv len != 26, got {len(frame)}")
        return None

    if frame[0] != 0xB5 or frame[1] != 0x9A:
        return None

    # CRC 校验（高字节在前）
    crc_recv = (frame[-2] << 8) | frame[-1]
    crc_calc = calculate_crc16_xf(frame[:-2])
    if crc_calc != crc_recv:
        print(f"[GIMBAL] CRC mismatch: calc=0x{crc_calc:04X}, recv=0x{crc_recv:04X}")
        return None

    fw_ver = frame[2]#0x26
    hw_err = frame[3]#0x00

    flags = frame[4] #0x06   
    inv_flag = flags & 0x01 #吊装/立装状态
    gbc_stat = (flags >> 1) & 0x07#云台状态
    tca_flag = (flags >> 4) & 0x01#温控就绪标志

    cmd_byte = frame[5]
    cmd_stat = cmd_byte & 0x07#命令执行状态
    cmd_value = (cmd_byte >> 3) & 0x1F#命令码回报

    # 后面 18 字节 = 9 个 int16：cam_rate[3]角速度 + cam_angle[3]姿态角 + mtr_angle[3] 
    cam_rate0, cam_rate1, cam_rate2, \
        cam_ang0, cam_ang1, cam_ang2, \
        mtr0, mtr1, mtr2 = struct.unpack("<hhhhhhhhh", frame[6:24])

    return GimbalStatus(
        fw_ver=fw_ver,
        hw_err=hw_err,
        inv_flag=inv_flag,
        gbc_stat=gbc_stat,
        tca_flag=tca_flag,
        cmd_stat=cmd_stat,
        cmd_value=cmd_value,
        cam_rate=(cam_rate0, cam_rate1, cam_rate2),
        cam_angle=(cam_ang0, cam_ang1, cam_ang2),
        mtr_angle=(mtr0, mtr1, mtr2),
    )


def read_gimbal_status(ser, timeout: float = 0.05) -> Optional[GimbalStatus]:
    """
    从串口读取一帧云台返回包。
    - 会自动寻找 0xB5 0x9A 作为帧头
    - 读不到完整一帧（超时）返回 None
    """
    if serial is None:
        raise RuntimeError("需要先安装 pyserial：pip install pyserial")

    old_timeout = ser.timeout
    ser.timeout = timeout
    try:
        while True:
            b1 = ser.read(1)
            if not b1:
                # 本次读取超时
                return None
            if b1[0] != 0xB5:
                continue

            b2 = ser.read(1)
            if not b2:
                return None
            if b2[0] != 0x9A:
                # 不是我们的协议头，重新找
                continue

            rest = ser.read(24)
            if len(rest) != 24:
                return None

            frame = b1 + b2 + rest
            # hex_dump(frame)#打印ACK信息
            status = _parse_gimbal_status_frame(frame)
            if status is not None:
                return status
            # 校验失败就继续找下一帧
    finally:
        ser.timeout = old_timeout


_STATE_NAMES = {
    0: "未定义",
    1: "初始化中",
    2: "云台停止",
    3: "云台保护",
    4: "手动控制",
    5: "指点平移",
}


def is_gimbal_ready(status: GimbalStatus, verbose: bool = True) -> bool:
    """
    根据 GimbalStatus 判断云台是否进入“可控”状态。
    这里的策略：
    - hw_err != 0  → 认为不可用
    - gbc_stat == 0/1/2/3 → 未准备好
    - gbc_stat == 4/5 且 tca_flag==1 → 认为已就绪
    """
    if verbose:
        print(
            f"[GIMBAL] state={_STATE_NAMES.get(status.gbc_stat, '未知')} "
            f"tca={status.tca_flag} hw_err={status.hw_err} "
            f"cmd(stat={status.cmd_stat}, val={status.cmd_value})"
        )

    if status.hw_err != 0:
        print("[GIMBAL] 硬件错误，无法控制！")
        return False

    if status.gbc_stat in (0, 1):
        # 未定义 / 初始化中
        return False

    if status.gbc_stat == 2:
        print("[GIMBAL] 当前为“云台停止”状态，如需使用需先发送启动命令（cmd=2）。")
        return False

    if status.gbc_stat == 3:
        print("[GIMBAL] 云台处于“保护模式”，请检查安装姿态、电源、电机等问题。")
        return False

    # 这里要求温控就绪（官方文档：温控就绪后才能校准陀螺仪）
    if status.tca_flag == 0:
        print("[GIMBAL] 温控尚未就绪，继续等待……")
        return False

    # 手动控制 或 指点平移，都认为“姿态已可控”
    if status.gbc_stat in (4, 5):
        return True

    return False

# ============================================================
#                  XFGimbal 高层封装类
# ============================================================

import time


class XFGimbal:
    """
    云台高层控制类：
    - 管理串口打开/关闭
    - 管理 trig 计数
    - 维护当前目标角度
    - 封装：启动/停止/回中/绝对角度/相对角度
    - 提供 wait_ready() 等待云台上电就绪
    """

    def __init__(self, port: str, baudrate: int = 115200,
                 heartbeat_hz: float = 50.0) -> None:
        """
        :param port: 串口号，例如 "COM3" 或 "/dev/ttyUSB0"
        :param baudrate: 波特率，默认 115200
        :param heartbeat_hz: 发送心跳的频率（仅在你外部循环中调用 update 时生效）
        """
        self.ser = open_gimbal_port(port, baudrate)
        self.trig = 0  # 3bit 触发计数
        self.target = GimbalTarget()  # 当前目标角度
        self.cmd_value = 4  # 默认使用“手动控制”命令
        self.fl_sens = 0    # 目前不用，可留作飞控敏感度调整
        self._heartbeat_interval = 1.0 / heartbeat_hz
        self._last_send_time = 0.0
        self.last_status: Optional[GimbalStatus] = None
        # 心跳线程控制
        self._hb_thread: Optional[threading.Thread] = None
        self._hb_stop = threading.Event()


    # -------------------- 内部工具 --------------------

    def _next_trig(self) -> int:
        self.trig = (self.trig + 1) & 0x07
        return self.trig

    def _build_and_send(self, *, cmd_value: Optional[int] = None,
                        target: Optional[GimbalTarget] = None) -> None:
        """
        构造并发送一帧控制包。
        :param cmd_value: 若为 None，则使用 self.cmd_value
        :param target:    若为 None，则使用 self.target
        """
        if target is None:
            target = self.target
        if cmd_value is None:
            cmd_value = self.cmd_value

        pkt = build_packet_from_target(
            target,
            cmd_value=cmd_value,
            trig=self._next_trig(),
            fl_sens=self.fl_sens,
        )
        send_gimbal_packet(self.ser, pkt)
    @staticmethod
    def smoothstep(x: float) -> float:
        """
        S 曲线插值：3x^2 - 2x^3, x ∈ [0,1]
        """
        return 3 * x * x - 2 * x * x * x
    
    """
    S 曲线平滑转动，更适合“摄影运镜效果”（慢速、柔和）。
    快速定位场景优先用 move_to_target_linear。
    """
    def move_smooth_abs(self,
                        end_pitch_deg: float,
                        end_yaw_deg: float,
                        duration: float,
                        dt: float = 0.02,
                        read_status: bool = False):
        """
        在给定时间 duration 内，从“当前目标角度 self.target”
        平滑移动到 (end_pitch_deg, end_yaw_deg)。

        - 只负责更新 target（轨迹生成）
        - 不直接发串口，由心跳线程调用 update() 负责发
        """

        # 1) 目标角限幅
        end_pitch_deg = clamp(end_pitch_deg, -135.0, 135.0)
        end_yaw_deg   = clamp(end_yaw_deg,   -180.0, 180.0)

        # 2) 起始角度从当前 target 读
        start_pitch = self.target.pitch_deg
        start_yaw   = self.target.yaw_deg

        t0 = time.time()
        t_end = t0 + duration

        while True:
            now = time.time()
            if now >= t_end:
                tau = 1.0
            else:
                tau = (now - t0) / duration  # [0,1]

            # S 曲线系数
            s = self.smoothstep(tau)

            # 3) 写入当前时刻应该达到的角度
            self.target.pitch_deg = start_pitch + (end_pitch_deg - start_pitch) * s
            self.target.yaw_deg   = start_yaw   + (end_yaw_deg   - start_yaw)   * s

            # 不在这里发包，等待心跳线程按 50Hz 调用 update()

            if tau >= 1.0:
                break

            # 让 CPU 歇一会儿，避免 while 忙等
            time.sleep(dt)


    # -------------------- 公共 API --------------------
    def close(self) -> None:
        """关闭串口"""
        self.stop_heartbeat()
        if self.ser and self.ser.is_open:
            self.ser.close()

    # 1) 等待云台上电就绪 --------------------------------

    def wait_ready(self, timeout_s: float = 10.0) -> bool:
        """
        上电后调用：循环发送“空控制包”，读取 ACK，并判断状态。
        :param timeout_s: 最长等待时间，超时返回 False
        """
        print("[XFGimbal] 等待云台进入可控状态……")
        start = time.time()

        idle_target = GimbalTarget(pitch_deg=0.0, roll_deg=0.0)

        while True:
            # 超时判断
            if time.time() - start > timeout_s:
                print("[XFGimbal] 等待就绪超时")
                return False

            # 发送一帧空控制包（cmd=4 手动控制，姿态 0）
            self._build_and_send(cmd_value=4, target=idle_target)

            # 读取 ACK
            status = read_gimbal_status(self.ser, timeout=0.05)
            if status is None:
                continue

            self.last_status = status
            if is_gimbal_ready(status, verbose=True):
                print("[XFGimbal] 云台已就绪。")
                return True

            # 控制频率 ~50Hz
            time.sleep(0.02)

    # 2) 主循环中周期调用：发送当前目标 + 读状态 -------------

    def update(self, read_status: bool = True) -> Optional[GimbalStatus]:
        """
        主循环中周期性调用：
        - 发送当前目标到云台
        - 可选：读取一帧 ACK 并返回
        """
        now = time.time()
        if now - self._last_send_time < self._heartbeat_interval:
            # 调用太频繁了，直接返回，避免发包过快
            return self.last_status

        self._last_send_time = now
        self._build_and_send()#组包+发送

        if read_status:
            status = read_gimbal_status(self.ser, timeout=0.0)
            if status is not None:
                self.last_status = status
            return status
        return None
    #用于S曲线平滑处理控制
    def _heartbeat_loop(self):
        """后台线程：固定节奏调用 update()，保持与云台的通信"""
        interval = self._heartbeat_interval
        while not self._hb_stop.is_set():
            # 只负责发当前 target，不用读状态
            self.update(read_status=False)
            time.sleep(interval)

    def start_heartbeat(self):
        """启动 50Hz 心跳线程（只需调一次）"""
        if self._hb_thread is not None and self._hb_thread.is_alive():
            return
        self._hb_stop.clear()
        self._hb_thread = threading.Thread(
            target=self._heartbeat_loop,
            daemon=True,
        )
        self._hb_thread.start()

    def stop_heartbeat(self):
        """停止心跳线程"""
        if self._hb_thread is None:
            return
        self._hb_stop.set()
        self._hb_thread.join(timeout=1.0)
        self._hb_thread = None


    # 3) 绝对角度控制 -------------------------------------

    def set_angle_abs(self, pitch_deg: float, roll_deg: float) -> None:
        """
        设置绝对目标角度。
        注意：C-40D 是 Pitch/Roll 云台。
        :param pitch_deg: 俯仰角 ，云台左右转（底座动）
        :param roll_deg:  滚转角 ，通常设为 0，云台上下动
        """
        # 做一下限幅，避免超出机械范围
        pitch_deg = clamp(pitch_deg, -135.0, 135.0)
        roll_deg = clamp(roll_deg, -45.0, 45.0)

        self.target.pitch_deg = pitch_deg
        self.target.roll_deg = roll_deg

    def center(self) -> None:
        """回中：pitch=0, yaw=0"""
        self.set_angle_abs(0.0, 0.0)

    # 4) 相对角度控制 -------------------------------------

    def move_angle_rel(self, dpitch_deg: float, dyaw_deg: float) -> None:
        """
        在当前角度基础上做相对转动。
        """
        cur_p = getattr(self.target, "pitch_deg", 0.0)
        cur_y = getattr(self.target, "yaw_deg", 0.0)
        self.set_angle_abs(cur_p + dpitch_deg, cur_y + dyaw_deg)

    # 5) 启动 / 停止云台 ----------------------------------

    def start(self) -> None:
        """
        发送一次“启动云台”命令（cmd=2）。
        注意：协议通常要求在停止后再启动。
        """
        print("[XFGimbal] 发送启动云台命令（cmd=2）")
        self._build_and_send(cmd_value=2)

    def stop(self) -> None:
        """
        发送一次“停止云台”命令（cmd=3）。
        """
        print("[XFGimbal] 发送停止云台命令（cmd=3）")
        self._build_and_send(cmd_value=3)

    # 6) 获取当前角度（从最近一次 ACK 中读取） --------------

    # def get_last_angles_deg(self) -> Optional[Tuple[float, float, float]]:
    #     """
    #     返回最近一次 ACK 中的相机姿态角（pitch, roll, yaw），单位 deg。
    #     若还没收到任何 ACK，则返回 None。
    #     """
    #     if self.last_status is None:
    #         return None
    #     p, r, y = self.last_status.cam_angle
    #     # 协议单位为 0.01deg
    #     return p / 100.0, r / 100.0, y / 100.0
    def get_last_angles_deg(self) -> Optional[Tuple[float, float, float]]:
            """
            修正后的角度读取
            """
            if self.last_status is None:
                return None
            
            # 原始数据 (云台视角: 0=平视, 90=天)
            raw_roll, raw_pitch, raw_yaw = self.last_status.cam_angle
            
            # === 核心修正 ===
            # 翻译成您的视角: -90=平视, 0=天
            real_pitch = (raw_pitch / 100.0) - 90.0 
            
            return raw_roll / 100.0, real_pitch, raw_yaw / 100.0