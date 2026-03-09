import serial
import time
import struct

class GT06ZGimbal:
    def __init__(self, port: str):
        self.port = port
        self.ser = None
        self.baudrate = 9600
        self.address = 0x00 
        
        # --- 状态记录 ---
        self.last_sent_el = None
        self.last_sent_az = None
        
        # --- 优化参数 ---
        self.epsilon = 0.2          # 角度变化死区
        self.min_interval = 0.1     # 最小发送间隔(秒)
        self.last_cmd_time = 0.0

    def open(self) -> bool:
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=0.1 
            )
            
            if self.ser.is_open:
                self.ser.reset_input_buffer()
                print("[GT06Z] Driver Ready (Angle Control + Speed Config).")
            return self.ser.is_open
        except Exception as e:
            print(f"[GT06Z] Open fail: {e}")
            return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.stop() # 关闭前确保停止
            self.ser.close()

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def _calc_checksum(self, payload_without_head: bytes) -> int:
        return sum(payload_without_head) & 0xFF

    def _send_frame(self, cmd1, cmd2, data_val):
        """
        发送标准帧
        data_val: 16位整数 (DataH << 8 | DataL)
        """
        if not self.is_connected(): return
        
        data_val = int(data_val) & 0xFFFF
        data_h = (data_val >> 8) & 0xFF
        data_l = data_val & 0xFF
        
        payload = bytes([self.address, cmd1, cmd2, data_h, data_l])
        checksum = self._calc_checksum(payload)
        
        frame = bytes([0xFF]) + payload + bytes([checksum])
        self.ser.write(frame)

    def stop(self):
        """发送停止指令"""
        # Cmd2=00, Data=0000
        self._send_frame(0x00, 0x00, 0x0000)

    # =================================================================
    # 核心修改：速度配置 (Item 1)
    # =================================================================
    def set_speed(self, speed: int = 0x3F):
        """
        通过依次发送各个方向的指令来设置全局速度。
        原理：发送 [方向+速度] -> [停止]，强制更新云台内部速度寄存器。
        
        :param speed: 速度值 1~63 (0x01 ~ 0x3F), 默认最大 63
        """
        if not self.is_connected(): return

        # 1. 限制范围 1 ~ 63
        val = max(1, min(63, int(speed)))
        print(f"[GT06Z] Configuring Speed Registers to: {val} (Max=63)")

        # 构造数据位
        # Data1控制水平速度 (Left/Right) -> 对应 high byte
        # Data2控制垂直速度 (Up/Down)    -> 对应 low byte
        
        data_pan = (val << 8) & 0xFF00  # Data1=Speed, Data2=0
        data_tilt = val & 0xFF          # Data1=0, Data2=Speed

        # --- 步骤 1: 设置水平速度 (右 Right: 0x02) ---
        # 发送: FF 00 00 02 [Speed] 00 [Check]
        self._send_frame(0x00, 0x02, data_pan)
        time.sleep(0.02) # 极短延时让单片机处理
        self.stop()      # 立即停止，防止转动过多
        time.sleep(0.01)

        # --- 步骤 2: 设置水平速度 (左 Left: 0x04) ---
        # 为了保险起见，双向都设置
        self._send_frame(0x00, 0x04, data_pan)
        time.sleep(0.02)
        self.stop()
        time.sleep(0.01)

        # --- 步骤 3: 设置垂直速度 (上 Up: 0x08) ---
        # 发送: FF 00 00 08 00 [Speed] [Check]
        self._send_frame(0x00, 0x08, data_tilt)
        time.sleep(0.02)
        self.stop()
        time.sleep(0.01)

        # --- 步骤 4: 设置垂直速度 (下 Down: 0x10) ---
        self._send_frame(0x00, 0x10, data_tilt)
        time.sleep(0.02)
        self.stop()
        
        print("[GT06Z] Speed Configuration Done.")

    # =================================================================
    # 核心控制：设置角度 (Item 8 & 9)
    # =================================================================
    def set_angles(self, elevation_deg: float, azimuth_deg: float):
        """
        发送绝对角度指令 (云台将使用 set_speed 设定的速度运行)
        """
        if not self.is_connected(): return

        now = time.time()
        if (now - self.last_cmd_time) < self.min_interval:
            return 

        # 变化量死区检查
        need_send_el = False
        if (self.last_sent_el is None) or (abs(elevation_deg - self.last_sent_el) > self.epsilon):
            need_send_el = True

        need_send_az = False
        if self.last_sent_az is None:
            need_send_az = True
        else:
            diff = abs(azimuth_deg - self.last_sent_az)
            if diff > 180: diff = 360 - diff
            if diff > self.epsilon:
                need_send_az = True
            
        if not need_send_el and not need_send_az:
            return

        self.ser.reset_input_buffer()

        # 发送垂直 (Cmd: 00 4D)
        if need_send_el:
            el_cmd_val = 0
            if elevation_deg > 0:
                el_cmd_val = 3600 - int(elevation_deg * 10) 
            else:
                el_cmd_val = int(abs(elevation_deg) * 10)   
            
            el_cmd_val = max(0, min(3600, el_cmd_val))
            self._send_frame(0x00, 0x4D, el_cmd_val)
            self.last_sent_el = elevation_deg
            
            if need_send_az:
                time.sleep(0.04) 

        # 发送水平 (Cmd: 00 4B)
        if need_send_az:
            norm_az = azimuth_deg % 360.0
            az_cmd_val = int(norm_az * 10) 
            az_cmd_val = max(0, min(3600, az_cmd_val))
            self._send_frame(0x00, 0x4B, az_cmd_val)
            self.last_sent_az = azimuth_deg

        self.last_cmd_time = time.time()

    def _read_specific_response(self, expected_cmd_byte, retry=2):
        if not self.is_connected(): return None
        for _ in range(retry):
            if self.ser.in_waiting < 7:
                time.sleep(0.02)
            data = self.ser.read(self.ser.in_waiting or 14) 
            if len(data) < 7: continue 
            
            for i in range(len(data) - 6):
                if data[i] != 0xFF: continue
                frame = data[i : i+7]
                if (sum(frame[1:6]) & 0xFF) != frame[6]: continue 
                
                if frame[3] == expected_cmd_byte:
                    return (frame[4] << 8) | frame[5]
        return None 

    def query_angles(self):
        if not self.is_connected(): return None
        self.ser.reset_input_buffer()

        self._send_frame(0x00, 0x53, 0)
        raw_el = self._read_specific_response(expected_cmd_byte=0x5B)
        if raw_el is not None:
            if raw_el <= 1800:
                self.last_sent_el = -(raw_el / 10.0) 
            else:
                self.last_sent_el = (3600 - raw_el) / 10.0 
        
        self._send_frame(0x00, 0x51, 0)
        raw_az = self._read_specific_response(expected_cmd_byte=0x59)
        if raw_az is not None:
            self.last_sent_az = raw_az / 10.0

        if self.last_sent_el is None: self.last_sent_el = 0.0
        if self.last_sent_az is None: self.last_sent_az = 0.0
            
        return (self.last_sent_el, self.last_sent_az)