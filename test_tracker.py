import unittest
import numpy as np

# 因为 main_tracking_v9.py 是作为独立脚本，我们需要导入里面的类
from main_tracking_v9 import StandardKalmanTrack, MultiTargetTracker, angular_diff

class TestStandardKalmanTrack(unittest.TestCase):
    def test_predict_straight_line(self):
        # 初始位置 10, 20
        track = StandardKalmanTrack(10.0, 20.0)
        # 手动设置速度为 az=10/s, el=5/s
        track.state[2, 0] = 10.0
        track.state[3, 0] = 5.0
        
        # 预测 0.1s
        track.predict(0.1)
        self.assertAlmostEqual(track.state[0, 0], 11.0)
        self.assertAlmostEqual(track.state[1, 0], 20.5)

    def test_update_converge(self):
        track = StandardKalmanTrack(10.0, 20.0)
        initial_p_trace = np.trace(track.P)
        
        # 多次 update 同一个位置，P 矩阵应该收敛变小
        for _ in range(5):
            track.update(10.0, 20.0, 0.1)
            
        final_p_trace = np.trace(track.P)
        self.assertLess(final_p_trace, initial_p_trace)

    def test_wrap_around_360(self):
        track = StandardKalmanTrack(358.0, 0.0)
        # 先 predict 建立 P 矩阵的跨项关联
        track.predict(0.1)
        # 目标跨越了 0 度，来到 2 度
        track.update(2.0, 0.0, 0.1)
        
        # 内部残差应该是 +4度，不是 -356度
        # 因此速度应该是正向的
        self.assertGreater(track.state[2, 0], 0.0)
        
        # 多次向同一方向移动
        for _ in range(10):
            track.predict(0.1)
            # 因为只有 predict，速度保持，看其是否绕回
        
        # 最终方位角应该在 0~360 之间
        self.assertTrue(0 <= track.state[0, 0] < 360.0)

    def test_history_length(self):
        track = StandardKalmanTrack(0.0, 0.0)
        for i in range(50):
            track.predict(0.1)
            track.update(float(i), 0.0, 0.1)
            
        self.assertEqual(len(track.history), 30)


class TestMultiTargetTracker(unittest.TestCase):
    def test_basic_association(self):
        tracker = MultiTargetTracker(max_lost_frames=15, base_distance_threshold=4.0)
        # Frame 1: 两人出现
        measurements = [[10.0, 20.0], [50.0, 60.0]]
        tracks = tracker.update(measurements, 0.1)
        self.assertEqual(len(tracks), 2)
        
        id1 = tracks[0].id
        id2 = tracks[1].id
        
        # Frame 2: 两人小幅移动
        measurements = [[11.0, 21.0], [51.0, 59.0]]
        tracks = tracker.update(measurements, 0.1)
        self.assertEqual(len(tracks), 2)
        
        # 需保持相同 ID
        self.assertEqual({t.id for t in tracks}, {id1, id2})

    def test_occlusion_recovery(self):
        tracker = MultiTargetTracker(max_lost_frames=15, base_distance_threshold=4.0)
        
        # 目标初始化并稳定追踪几帧，使其积累确定性和速度
        for i in range(5):
            tracks = tracker.update([[10.0 + i, 20.0]], 0.1)
        
        target_id = tracks[0].id
        
        # 遮挡5帧 (利用 predict 瞎猜，不删除)
        for _ in range(5):
            tracks = tracker.update([], 0.1)
        
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0].id, target_id)
        
        # 目标再次出现，因为有协方差膨胀，即使偏差稍大也可以匹配上
        tracks = tracker.update([[18.0, 20.0]], 0.1)
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0].id, target_id)
        
    def test_dynamic_thresholding(self):
        tracker = MultiTargetTracker(max_lost_frames=15, base_distance_threshold=2.0)
        
        # 新目标刚出现，P矩阵较大 (对角线为1.0)
        tracks = tracker.update([[10.0, 20.0]], 0.1)
        
        # predict 会将 P[0,0] 从 1.0 加大到 1.1 左右（Q为0.1）
        # 此时协方差不确定性 uncertainty = sqrt(1.1 + 1.1) ~ 1.48
        # 动态门限 = 2.0 + 1.48 * 1.5 = 4.22
        # 所以相距 3.5 度的目标可以被认为是同一个
        tracks = tracker.update([[13.5, 20.0]], 0.1)
        self.assertEqual(len(tracks), 1) # 成功匹配，没有新建
        
        # 然后稳定追踪多次，协方差会变得很小
        for _ in range(30):
             tracks = tracker.update([[13.5, 20.0]], 0.1)
             
        # 此时 P 变得很小，动态门限接近 4.9，不可能大过 10.0
        # 如果出现距离 10.0 度的测量，将无法匹配，会建新轨
        tracks = tracker.update([[23.5, 20.0]], 0.1)
        self.assertEqual(len(tracks), 2)


class TestThreatScore(unittest.TestCase):
    def test_threat_score(self):
        # 验证我们在 main 函数里写的逻辑是否合理
        # 目标1：静止，距离较近，稳定追踪10帧
        t1 = StandardKalmanTrack(5.0, 0.0)
        t1.state[2, 0] = 0.0 # vaz
        t1.state[3, 0] = 0.0 # vel
        t1.hit_streak = 10
        t1.id = 1
        
        # 目标2：高速，距离略远，稳定追踪10帧
        t2 = StandardKalmanTrack(10.0, 0.0)
        t2.state[2, 0] = 10.0 # vaz
        t2.state[3, 0] = 5.0 # vel
        t2.hit_streak = 10
        t2.id = 2
        
        curr_gimbal_az = 0.0
        curr_gimbal_el = 0.0
        
        def calc_score(t, master_id=None):
            v_mag = np.sqrt(t.state[2, 0]**2 + t.state[3, 0]**2)
            speed_score = v_mag * 2.0
            
            dist_az = angular_diff(t.state[0, 0], curr_gimbal_az)
            dist_el = t.state[1, 0] - curr_gimbal_el
            dist = np.sqrt(dist_az**2 + dist_el**2)
            distance_score = -dist * 1.0
            
            stability_score = t.hit_streak * 0.5
            
            threat_score = speed_score + distance_score + stability_score
            if t.id == master_id:
                threat_score *= 1.15
            return threat_score
            
        score1 = calc_score(t1)
        score2 = calc_score(t2)
        
        # 高速目标应该威胁度更高，即使它距离较远
        self.assertGreater(score2, score1)
        
        # 测试 15% 惯性加成
        t1_as_master_score = calc_score(t1, master_id=1)
        self.assertAlmostEqual(t1_as_master_score, score1 * 1.15)


if __name__ == '__main__':
    unittest.main()
