import json
import os

# 文件路径
joint_map_file = os.path.join("..", "config", "retargeting_poly_map.json")
poly_models_file = os.path.join("..", "config", "finger_poly_params.json")
output_file = os.path.join("..", "config", "finger_config.json")
# 读取 joint_map.json
with open(joint_map_file, "r") as f:
    joint_map = json.load(f)["joint_map"]

# 读取 poly_models.json
with open(poly_models_file, "r") as f:
    poly_models = json.load(f)

# 构建新的 JSON 结构
combined = {
    "joint_map": joint_map,
    "poly_models": {}
}

for finger, coeff_list in poly_models.items():
    # 收集当前手指所有 joint_type
    joint_types = set()
    for joint_name, info in joint_map.items():
        if info["finger"] == finger or finger.startswith(info["finger"]):
            joint_types.add(info["joint_idx"])
    
    # 按照 MCP - PIP - DIP - CMC 排序
    joint_order = sorted(joint_types, key=lambda x: ["MCP","PIP","DIP","CMC"].index(x))
    
    # 保存到 finger_config
    combined["poly_models"][finger] = {
        "joint_order": joint_order,
        "coeffs": coeff_list
    }

# 保存新的 JSON 文件
with open(output_file, "w") as f:
    json.dump(combined, f, indent=4)

print(f"合并完成，保存为 {output_file}")
