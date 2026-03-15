import time
from typing import Tuple, Optional
from gimbal_interface import GimbalBase #

class MockGimbalAdapter(GimbalBase):
    """
    纯软件模拟云台驱动，用于 SITL 闭环测试。
    """
    def __init__(self, port: str = "MOCK_PORT", slew_rate: float = 60.0):
        self.port = port
        self.slew_rate = slew_rate  # 模拟云台最大转速 (度/秒)
        self._is_ready = False
        
        # 初始状态设为系统基准水平（水平90度，俯仰0度）
        self.curr_el = 0.0
        self.curr_az = 90.0
        
        self.target_el = 0.0
        self.target_az = 90.0
        
        self.last_update_time = time.time()

    def connect(self) -> bool:
        print(f"[MockGimbal] 虚拟云台已连接 (端口: {self.port})")
        self._is_ready = True
        self.last_update_time = time.time()
        return True

    def wait_ready(self, timeout: float = 5.0) -> bool:
        return self._is_ready

    def set_attitude(self, elevation: float, azimuth: float, read_status: bool = False):
        self.target_el = elevation
        self.target_az = azimuth % 360.0
        # print(f"[MockGimbal] 收到运动指令 -> 目标 El: {self.target_el:.1f}, Az: {self.target_az:.1f}")

    def get_attitude(self) -> Optional[Tuple[float, float, float]]:
        # 1. 计算距离上次查询经过的时间
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now
        
        # 2. 模拟物理运动 (简单的匀速逼近)
        step = self.slew_rate * dt
        
        # 俯仰角 (El) 逼近
        if abs(self.target_el - self.curr_el) <= step:
            self.curr_el = self.target_el
        else:
            self.curr_el += step if self.target_el > self.curr_el else -step
            
        # 方位角 (Az) 逼近 (需处理 360 度环绕)
        diff_az = (self.target_az - self.curr_az + 180.0) % 360.0 - 180.0
        if abs(diff_az) <= step:
            self.curr_az = self.target_az
        else:
            self.curr_az += step if diff_az > 0 else -step
            
        self.curr_az = self.curr_az % 360.0
        
        # 返回符合接口标准的元组
        return (self.curr_el, self.curr_az, 0.0)

    def close(self):
        print("[MockGimbal] 虚拟云台已关闭")
        self._is_ready = False