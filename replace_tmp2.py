import re

def main():
    with open('main_tracking_v9.py', 'r', encoding='utf-8') as f:
        content = f.read()

    # The region to replace is this specific block:
    #             if master_track is None or lock_timer <= 0:
    #                 ...
    #                         print(f"[Phase 2: 目标调度] 成功锁定目标 ID: {master_id} (已连续追踪 {master_track.hit_streak} 帧). 准备交由云台跟踪!")

    replace_pattern = re.compile(
        r'            if master_track is None or lock_timer <= 0:\n'
        r'                # 状态 A：寻找/切换新目标 \(SEARCHING\)\n'
        r'                master_id = None\n'
        r'                if len\(valid_tracks\) > 0:\n'
        r'                    # 策略：找当前距离云台物理角度最近的目标\n'
        r'                    curr_gimbal_az = shared_gimbal_az\n'
        r'                    curr_gimbal_el = shared_gimbal_el\n'
        r'                    \n'
        r'                    best_track = None\n'
        r'                    min_dist = float\(\'inf\'\)\n'
        r'                    for t in valid_tracks:\n'
        r'                        dist_az = angular_diff\(t\.state\[0\], curr_gimbal_az\)\n'
        r'                        dist_el = t\.state\[1\] - curr_gimbal_el\n'
        r'                        dist = np\.sqrt\(dist_az\*\*2 \+ dist_el\*\*2\)\n'
        r'                        if dist < min_dist:\n'
        r'                            min_dist = dist\n'
        r'                            best_track = t\n'
        r'                            \n'
        r'                    if best_track:\n'
        r'                        master_id = best_track\.id\n'
        r'                        lock_timer = LOCK_DURATION\n'
        r'                        master_track = best_track\n'
        r'                        # print\(f"\[Scheduler\] 切换锁定目标 ID: {master_id}, 倒计时重置 {LOCK_DURATION}s"\)\n'
        r'                        # === 修改：Phase 2 打印 ===\n'
        r'                        print\(f"\[Phase 2: 目标调度\] 成功锁定目标 ID: {master_id} \(已连续追踪 {master_track\.hit_streak} 帧\)\. 准备交由云台跟踪!"\)'
    )

    new_scheduler = '''            if master_track is None or lock_timer <= 0:
                # 状态 A：寻找/切换新目标 (SEARCHING)
                best_track = None
                best_score = -float('inf')
                
                curr_gimbal_az = shared_gimbal_az
                curr_gimbal_el = shared_gimbal_el
                
                if len(valid_tracks) > 0:
                    for t in valid_tracks:
                        # 1. 速度因子 (正权 2.0)：提取目标的合成角速度大小
                        v_mag = np.sqrt(t.state[2, 0]**2 + t.state[3, 0]**2)
                        speed_score = v_mag * 2.0
                        
                        # 2. 距离因子 (负权 1.0)：计算目标当前角度与云台物理角度的距离
                        dist_az = angular_diff(t.state[0, 0], curr_gimbal_az)
                        dist_el = t.state[1, 0] - curr_gimbal_el
                        dist = np.sqrt(dist_az**2 + dist_el**2)
                        distance_score = -dist * 1.0
                        
                        # 3. 稳定性因子 (正权 0.5)：连续追踪帧数
                        stability_score = t.hit_streak * 0.5
                        
                        # 综合威胁度得分
                        threat_score = speed_score + distance_score + stability_score
                        
                        # 当前锁定目标 15% 惯性加成防抖动
                        if t.id == master_id:
                            # 给原得分基础上乘以 1.15
                            threat_score *= 1.15
                            
                        if threat_score > best_score:
                            best_score = threat_score
                            best_track = t
                            
                    if best_track:
                        master_id = best_track.id
                        lock_timer = LOCK_DURATION
                        master_track = best_track
                        print(f"[Phase 2: 目标调度] 基于威胁度(v={np.sqrt(master_track.state[2,0]**2+master_track.state[3,0]**2):.1f}°/s, dist={np.sqrt(angular_diff(master_track.state[0,0], curr_gimbal_az)**2 + (master_track.state[1,0]-curr_gimbal_el)**2):.1f}°) 成功锁定目标 ID: {master_id}. 准备交由云台跟踪!")'''

    new_content = replace_pattern.sub(new_scheduler, content)

    # I also need to update the printing and sending logic slightly further down
    # from t.state[0] to t.state[0, 0] because we upgraded to a matrix state.
    
    # Update [Phase 3] print statements
    phase3_pattern = re.compile(
        r'print\(f"\[Phase 3: 预测控制\] 目标当前估算Az={master_track\.state\[0\]:\.2f}°\, 速度={master_track\.state\[2\]:\.2f}°/s"\)'
    )
    new_phase3 = 'print(f"[Phase 3: 预测控制] 目标当前估算Az={master_track.state[0, 0]:.2f}°, 速度={master_track.state[2, 0]:.2f}°/s")'
    new_content = phase3_pattern.sub(new_phase3, new_content)

    # Update UI Sending (Iterating through valid_tracks)
    ui_send_pattern = re.compile(
        r'azimuth=t\.state\[0\],        # 发送卡尔曼平滑后的位置\n'
        r'                        elevation=t\.state\[1\],'
    )
    new_ui_send = '''azimuth=t.state[0, 0],        # 发送卡尔曼平滑后的位置
                        elevation=t.state[1, 0],'''
    new_content = ui_send_pattern.sub(new_ui_send, new_content)

    with open('main_tracking_v9.py', 'w', encoding='utf-8') as f:
        f.write(new_content)
        
    print("Files replaced successfully.")

if __name__ == '__main__':
    main()
