#!/usr/bin/env python3
import numpy as np
from PIL import Image
import yaml
import os

# ----------------------------
# 参数配置（根据你要求设置）
# ----------------------------
map_width_m = 210.0      # 长（X方向，m）
map_height_m = 200.0     # 宽（Y方向，m）
resolution = 0.05         # 分辨率 m/cell，按需调节，越大越省内存

# 原点相对于地图左下角的世界坐标
# 注意：map_server 的 origin 是地图左下角在世界坐标的位置
origin_bottom_distance = 26.0  # 机器人离下边界 26m
origin_right_distance = 33.0   # 机器人离右边界 33m

origin_x = -origin_right_distance           # X方向，负方向表示原点在地图左边界左侧
origin_y = -origin_bottom_distance          # Y方向，负方向表示原点在地图下边界下方

# 输出路径
output_dir = "./maps"
os.makedirs(output_dir, exist_ok=True)
png_file = os.path.join(output_dir, "blank_map.png")
yaml_file = os.path.join(output_dir, "blank_map.yaml")

# ----------------------------
# 生成空白地图
# ----------------------------
width_cells = int(map_width_m / resolution)
height_cells = int(map_height_m / resolution)

# 全白（Free space=0），注意map_server默认0表示FREE_SPACE
data = np.full((height_cells, width_cells),255, dtype=np.uint8)

# 翻转Y轴，使左下角对应世界坐标原点
data = np.flipud(data)

# 保存PNG
im = Image.fromarray(data)
im.save(png_file)
print(f"Saved map image: {png_file}")

# ----------------------------
# 生成YAML
# ----------------------------
yaml_content = {
    "image": os.path.basename(png_file),
    "resolution": resolution,
    "origin": [origin_x, origin_y, 0.0],
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.196
}

with open(yaml_file, 'w') as f:
    yaml.dump(yaml_content, f, default_flow_style=False)

print(f"Saved map yaml: {yaml_file}")
print("地图生成完成！")
