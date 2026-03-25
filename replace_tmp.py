import re

def main():
    with open('main_tracking_v9.py', 'r', encoding='utf-8') as f:
        content = f.read()

    # Replace Track class with StandardKalmanTrack
    track_pattern = re.compile(r'class Track:\n(.*?)(?=# ==========================================\n# 新增模块 3：多目标调度大脑 \(MultiTargetTracker\)\n# ==========================================)', re.DOTALL)
    new_track = '''class StandardKalmanTrack:
    _id_count = 0
    def __init__(self, ui_az, ui_el):
        StandardKalmanTrack._id_count += 1
        self.id = StandardKalmanTrack._id_count
        
        # 状态矩阵: X = [[az], [el], [v_az], [v_el]]
        self.state = np.array([[ui_az], [ui_el], [0.0], [0.0]], dtype=float)
        
        # 协方差矩阵 P
        self.P = np.diag([1.0, 1.0, 10.0, 10.0])
        
        # 过程噪声 / 测量噪声默认值
        self.q_pos = 0.1
        self.q_vel = 1.0
        self.r_az = 3.5**2
        self.r_el = 1.8**2
        
        self.hit_streak = 1        # 连续命中次数 (用于建轨确认)
        self.time_since_update = 0 # 连丢次数
        
        # 历史队列
        self.history = deque(maxlen=30)
        self.history.append((self.state.copy(), self.P.copy()))
        
        self.max_vel_az = 40.0
        self.max_vel_el = 20.0
        self.min_dt = 0.03

    def set_dynamic_params(self, params):
        if not params:
            return
        
        max_res_az = float(params.get('MAX_RES_AZ', 3.5))
        max_res_el = float(params.get('MAX_RES_EL', 1.8))
        self.r_az = max_res_az**2
        self.r_el = max_res_el**2
        
        self.max_vel_az = float(params.get('MAX_VEL_AZ', self.max_vel_az))
        self.max_vel_el = float(params.get('MAX_VEL_EL', self.max_vel_el))
        self.min_dt = float(params.get('MIN_DT', self.min_dt))

    def predict(self, dt):
        """标准 Kalman Predict"""
        if dt < self.min_dt:
            dt = self.min_dt
            
        # 状态转移矩阵 F
        F = np.array([
            [1, 0, dt,  0],
            [0, 1,  0, dt],
            [0, 0,  1,  0],
            [0, 0,  0,  1]
        ], dtype=float)
        
        # 过程噪声 Q
        Q = np.diag([self.q_pos, self.q_pos, self.q_vel, self.q_vel])
        
        # 预测状态和协方差
        self.state = np.dot(F, self.state)
        # 防止角度跳变，对 Azimuth 进行取模
        self.state[0, 0] = self.state[0, 0] % 360.0
        self.P = np.dot(np.dot(F, self.P), F.T) + Q
        
        self.time_since_update += 1
        self.history.append((self.state.copy(), self.P.copy()))

    def update(self, meas_az, meas_el, dt):
        """标准 Kalman Update"""
        self.time_since_update = 0
        self.hit_streak += 1

        Z = np.array([[meas_az], [meas_el]], dtype=float)
        
        # 观测矩阵 H
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=float)
        
        # 观测测量噪声 R
        R = np.diag([self.r_az, self.r_el])
        
        # 计算残差 Y = Z - HX
        Y = Z - np.dot(H, self.state)
        
        # 核心：处理 Azimuth 的残差，防止角度回环跳变
        Y[0, 0] = angular_diff(Z[0, 0], self.state[0, 0])
        
        # S = H * P * H^T + R
        S = np.dot(np.dot(H, self.P), H.T) + R
        
        # 卡尔曼增益 K = P * H^T * S^-1
        try:
            K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))
        except np.linalg.LinAlgError:
            print(f"[Tracker] Kalman matrix inversion failed for track {self.id}")
            K = np.zeros((4, 2), dtype=float)

        # X = X + K * Y
        self.state = self.state + np.dot(K, Y)
        # 更新后再次对 Az 取模
        self.state[0, 0] = self.state[0, 0] % 360.0
        
        # 速度限幅保护
        self.state[2, 0] = np.clip(self.state[2, 0], -self.max_vel_az, self.max_vel_az)
        self.state[3, 0] = np.clip(self.state[3, 0], -self.max_vel_el, self.max_vel_el)
        
        # P = (I - K * H) * P
        I = np.eye(4)
        self.P = np.dot((I - np.dot(K, H)), self.P)
        
        # 更新历史
        if len(self.history) > 0:
            self.history[-1] = (self.state.copy(), self.P.copy())
        else:
            self.history.append((self.state.copy(), self.P.copy()))

    def get_future_position(self, dt_delay):
        """打提前量：获取未来预测角度"""
        fut_az = (self.state[0, 0] + self.state[2, 0] * dt_delay) % 360.0
        fut_el = self.state[1, 0] + self.state[3, 0] * dt_delay
        return fut_az, fut_el
        
    def predict_future_n_steps(self, n=10, dt=0.066):
        """多帧预测接口"""
        if dt < self.min_dt:
            dt = self.min_dt
            
        F = np.array([
            [1, 0, dt,  0],
            [0, 1,  0, dt],
            [0, 0,  1,  0],
            [0, 0,  0,  1]
        ], dtype=float)
        Q = np.diag([self.q_pos, self.q_pos, self.q_vel, self.q_vel])
        
        temp_state = self.state.copy()
        temp_P = self.P.copy()
        
        future_states = []
        future_Ps = []
        
        for _ in range(n):
            temp_state = np.dot(F, temp_state)
            temp_state[0, 0] = temp_state[0, 0] % 360.0
            temp_P = np.dot(np.dot(F, temp_P), F.T) + Q
            future_states.append(temp_state.copy())
            future_Ps.append(temp_P.copy())
            
        return future_states, future_Ps

'''
    new_content = track_pattern.sub(new_track, content)

    multi_tracker_pattern = re.compile(r'class MultiTargetTracker:\n(.*?)(?=# ==========================================\n# 4\. 核心解算 V8 \(修改版：基准水平90度\)\n# ==========================================)', re.DOTALL)
    new_multi_tracker = '''class MultiTargetTracker:
    def __init__(self, max_lost_frames=15, base_distance_threshold=4.0):
        self.tracks = []
        self.max_lost_frames = max_lost_frames
        self.base_distance_threshold = base_distance_threshold

    def update(self, measurements, dt, params=None):
        """
        measurements: 当前帧所有检测到的绝对角度列表 [[az1, el1], [az2, el2], ...]
        dt: 距离上一帧经过的时间(秒)
        """
        if params and ("DIST_THRESH" in params):
            self.base_distance_threshold = float(params["DIST_THRESH"])

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
                t = StandardKalmanTrack(meas[0], meas[1])  # create kalman track for each measurement
                t.set_dynamic_params(params)
                self.tracks.append(t)
            return self.tracks

        # 2. 计算代价矩阵 (角度欧氏距离)
        cost_matrix = np.zeros((len(self.tracks), len(measurements)))
        for t, track in enumerate(self.tracks):
            for m, meas in enumerate(measurements):
                diff_az = angular_diff(meas[0], track.state[0, 0])
                diff_el = meas[1] - track.state[1, 0]
                distance = np.sqrt(diff_az**2 + diff_el**2)
                cost_matrix[t, m] = distance

        # 3. 匈牙利匹配
        track_indices, meas_indices = linear_sum_assignment(cost_matrix)

        # 4. 更新匹配成功的 Track (加入协方差动态门限)
        unmatched_measurements = set(range(len(measurements)))
        for t_idx, m_idx in zip(track_indices, meas_indices):
            track = self.tracks[t_idx]
            
            # 使用协方差评估不确定性
            uncertainty = np.sqrt(track.P[0, 0] + track.P[1, 1])
            # 动态欧氏门限：基础残差 + 协方差不确定性 * 膨胀系数(1.5)
            dynamic_thresh = self.base_distance_threshold + (uncertainty * 1.5)
            
            if cost_matrix[t_idx, m_idx] < dynamic_thresh:
                track.update(measurements[m_idx][0], measurements[m_idx][1], dt)
                unmatched_measurements.discard(m_idx)
            else:
                pass

        # 5. 为没匹配上的坐标创建新 Track
        for m_idx in unmatched_measurements:
            meas = measurements[m_idx]
            t = StandardKalmanTrack(meas[0], meas[1])
            t.set_dynamic_params(params)
            self.tracks.append(t)

        # 6. 删除丢失太久的 Track
        self.tracks = [t for t in self.tracks if t.time_since_update < self.max_lost_frames]

        return self.tracks

'''
    new_content = multi_tracker_pattern.sub(new_multi_tracker, new_content)

    with open('main_tracking_v9.py', 'w', encoding='utf-8') as f:
        f.write(new_content)
        
    print("Files replaced successfully.")

if __name__ == '__main__':
    main()
