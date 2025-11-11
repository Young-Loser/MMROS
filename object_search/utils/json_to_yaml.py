#!/usr/bin/env python3
import json
import yaml
import os

# === 坐标换算参数 ===
RESOLUTION = 0.04   # [m/pixel]
ORIGIN_X = -16.0    # 左下角 ROS 坐标
ORIGIN_Y = -12.0
MAP_HEIGHT_PX = 600  # 根据你的地图图像高度设定

def px_to_ros(x_px, y_px):
    """像素坐标 -> ROS 世界坐标"""
    x = x_px * RESOLUTION + ORIGIN_X
    y = ORIGIN_Y + (MAP_HEIGHT_PX - y_px) * RESOLUTION
    return round(x, 3), round(y, 3)

def bbox_to_ros(bbox_px):
    """bbox -> ROS坐标（中心点+范围）"""
    x_min, y_min, x_max, y_max = bbox_px
    cx_px = (x_min + x_max) / 2
    cy_px = (y_min + y_max) / 2
    width_px = x_max - x_min
    height_px = y_max - y_min

    cx, cy = px_to_ros(cx_px, cy_px)
    width = round(width_px * RESOLUTION, 3)
    height = round(height_px * RESOLUTION, 3)

    return {
        "center": {"x": cx, "y": cy},
        "size": {"width": width, "height": height}
    }

def convert_json_to_yaml():
    json_file = "src/object_search/utils/apartment1_all.json"
    yaml_file = "src/object_search/utils/apartment1_all.yaml"

    if not os.path.exists(json_file):
        print(f"❌ 未找到 {json_file}")
        return

    with open(json_file, "r") as f:
        data = json.load(f)

    output = {
        "meta": {
            "resolution": RESOLUTION,
            "origin": {"x": ORIGIN_X, "y": ORIGIN_Y},
            "map_height_px": MAP_HEIGHT_PX
        },
        "rooms": [],
        "doors": [],
        "objects": []
    }

    # 转换房间
    for room in data.get("rooms", []):
        entry = {
            "name": room["name"],
            "label": room.get("label", ""),
            "bbox_px": room["bbox_px"],
            "ros": bbox_to_ros(room["bbox_px"])
        }
        output["rooms"].append(entry)

    # 转换门
    for door in data.get("doors", []):
        entry = {
            "id": door["id"],
            "name": door["name"],
            "bbox_px": door["bbox_px"],
            "ros": bbox_to_ros(door["bbox_px"])
        }
        output["doors"].append(entry)

    # 转换物体
    for obj in data.get("objects", []):
        entry = {
            "name": obj["name"],
            "room": obj.get("room", ""),
            "bbox_px": obj["bbox_px"],
            "ros": bbox_to_ros(obj["bbox_px"])
        }
        output["objects"].append(entry)

    with open(yaml_file, "w") as f:
        yaml.dump(output, f, sort_keys=False, allow_unicode=True)

    print(f"✅ 已转换并保存为 {yaml_file}")

if __name__ == "__main__":
    convert_json_to_yaml()

