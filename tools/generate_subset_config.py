import json

# ================= 1. 基础物理参数定义 =================
# 俯仰角 (底层 -> 顶层)
TABLE_ANGLES = [90, 80.5, 61.5, 52, 42.5, 33, 23.5, 14]
# 每层数量 (Layer 1 -> Layer 8)
LAYER_COUNTS = [22, 21, 20, 19, 18, 17, 16, 15]

def get_full_hemisphere_data():
    """生成全量半球数据 (Key为 '层号+序号', 如 '101')"""
    full_data = {}
    for i in range(8):
        layer_idx = i + 1
        count = LAYER_COUNTS[i]
        pitch = float(90 - TABLE_ANGLES[i]) # 底层视为0度
        step_angle = 360.0 / count
        
        for cam_idx in range(1, count + 1):
            # 原始物理ID：层号+序号 (如 101, 201)
            original_id = f"{layer_idx}{cam_idx:02d}"
            
            # 计算角度 (第1个为基准0度)
            horizontal = (cam_idx - 1) * step_angle
            
            full_data[original_id] = {
                "theta_vertical": round(pitch, 2),
                "theta_horizontal": round(horizontal, 3)
            }
    return full_data

# ================= 2. 选型配置 (关键步骤) =================
# 在这里填入你选中的 40 个“原始物理位置 ID”
# 假设你选了包含对齐列(x01)在内的部分扇区
# 例如：第1层选前5个(101-105)，第2层选前5个(201-205)...
# 你需要根据实际选的物理孔位修改这个列表！
SELECTED_SLOTS = [
    # --- 第1层 (假设选了5个) ---
    "103", "102", "101", "122", "121",
    # --- 第2层 (假设选了5个) ---
    "203", "202", "201", "221", "220",
    # --- 第3层 ---
    "303", "302", "301", "320", "319",
    "403", "402", "401", "419", "418",
    "503", "502", "501", "518", "517",
    "603", "602", "601", "617", "616",
    "703", "702", "701", "716", "715",
    "803", "802", "801", "815", "814",
    # ... 请继续填补直到凑齐40个 ...
    # 这里的顺序决定了新编号 1~40 的顺序！
]

if __name__ == "__main__":
    full_map = get_full_hemisphere_data()
    
    new_device_theta = {}
    
    print(f"准备提取 {len(SELECTED_SLOTS)} 个摄像头配置...\n")
    
    for new_id, original_id in enumerate(SELECTED_SLOTS, 1):
        if original_id not in full_map:
            print(f"[错误] 原始ID {original_id} 不存在于全量模型中！")
            continue
            
        # 获取偏差数据
        offsets = full_map[original_id]
        
        # 存入新字典，Key 为 1~40 的整数
        new_device_theta[new_id] = offsets
        
        print(f"新编号 {new_id:02d} (原{original_id}): {offsets}")

    # 生成代码供复制
    print("\n" + "="*30)
    print("# 请复制以下字典到主程序 main_tracking_v8.py")
    print("DEVICE_THETA = " + json.dumps(new_device_theta, indent=4))
    print("="*30)