import json

# ================= 配置参数 =================
# 表格中的原始俯仰角 (从底层 Layer 1 到 顶层 Layer 8)
table_angles = [90, 80.5, 61.5, 52, 42.5, 33, 23.5, 14]

# 每一层的摄像头数量 (Layer 1=22 ... Layer 8=15)
# 逻辑：22 - (layer_index - 1)
layer_counts = [22, 21, 20, 19, 18, 17, 16, 15]

def generate_config():
    device_theta = {}
    
    print("=== 开始生成配置 ===")
    
    for i in range(8):
        layer_idx = i + 1  # 层号 1-8
        count = layer_counts[i]
        raw_angle = table_angles[i]
        
        # 1. 计算该层的统一俯仰角 (Pitch)
        # 需求：底层90度视为0度，向上仰起为正
        pitch = float(90 - raw_angle)
        
        # 2. 计算该层的水平步进角 (Step)
        step_angle = 360.0 / count
        
        print(f"Layer {layer_idx}: Count={count}, Pitch={pitch}°, Step={step_angle:.2f}°")
        
        for cam_idx in range(1, count + 1):
            # 3. 生成ID (格式：层号+2位序号，如 101, 815)
            # 这种格式方便在 UDP 接收时直接解析 int(ID)
            cam_id_str = f"{layer_idx}{cam_idx:02d}"
            
            # 4. 计算水平角
            # 第1个摄像头(cam_idx=1)为基准 0度
            # 顺时针为正
            horizontal = (cam_idx - 1) * step_angle
            
            # 写入字典
            device_theta[cam_id_str] = {
                "theta_vertical": round(pitch, 2),
                "theta_horizontal": round(horizontal, 3)
            }
            
    return device_theta

# ================= 生成并打印 =================
if __name__ == "__main__":
    config = generate_config()
    
    # 转换为格式化字符串，方便复制到 main_tracking_v8.py
    # 也就是生成 device_theta = { ... } 的代码块
    output_str = "device_theta = " + json.dumps(config, indent=4)
    
    # 简单处理一下 key 的引号，使其看起来像 Python 字典 (虽然 JSON 格式 Python 也能认)
    print("\n" + "="*20 + " 请复制以下内容到 main_tracking_v8.py " + "="*20)
    print(output_str)
    print("="*60)
    
    # 保存到文件
    with open("generated_theta.txt", "w", encoding="utf-8") as f:
        f.write(output_str)
        print("\n[提示] 配置已保存到 generated_theta.txt")