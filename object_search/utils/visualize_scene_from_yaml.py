#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
根据 apartment1_all.yaml 绘制地图结构：
- 房间边界（灰色虚线框）
- 门（红色矩形）
- 物体（蓝色矩形）
- 名称文字标注
"""

import yaml
import os
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def draw_scene_from_yaml(yaml_path):
    # 读取 YAML 文件
    if not os.path.exists(yaml_path):
        print(f"❌ 文件不存在: {yaml_path}")
        return

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_aspect('equal')
    ax.set_title("🏠 Apartment1 Scene Visualization", fontsize=14)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")

    # ---------------- 绘制房间 ----------------
    if "rooms" in data:
        for room in data["rooms"]:
            if "ros" not in room:
                continue
            c = room["ros"]["center"]
            s = room["ros"]["size"]
            x = c["x"] - s["width"] / 2
            y = c["y"] - s["height"] / 2

            rect = patches.Rectangle(
                (x, y),
                s["width"],
                s["height"],
                linewidth=1.5,
                edgecolor='gray',
                facecolor='none',
                linestyle='--'
            )
            ax.add_patch(rect)
            ax.text(c["x"], c["y"], room["name"], color='gray', fontsize=9, ha='center', va='center')

    # ---------------- 绘制门 ----------------
    if "doors" in data:
        for door in data["doors"]:
            if "ros" not in door:
                continue
            c = door["ros"]["center"]
            s = door["ros"]["size"]
            x = c["x"] - s["width"] / 2
            y = c["y"] - s["height"] / 2

            rect = patches.Rectangle(
                (x, y),
                s["width"],
                s["height"],
                linewidth=1.2,
                edgecolor='red',
                facecolor='mistyrose',
                alpha=0.8
            )
            ax.add_patch(rect)
            ax.text(c["x"], c["y"] + 0.2, door.get("name", "door"), color='red', fontsize=8, ha='center')

    # ---------------- 绘制物体 ----------------
    if "objects" in data:
        label_positions = []  # 用于防止重叠
        offset_vectors = [(0.5, 0.5), (-0.5, 0.5), (-0.5, -0.5), (0.5, -0.5)]  # 四个偏移方向
        offset_index = 0

        for obj in data["objects"]:
            if "ros" not in obj:
                continue
            c = obj["ros"]["center"]
            s = obj["ros"]["size"]
            x = c["x"] - s["width"] / 2
            y = c["y"] - s["height"] / 2

            # 绘制矩形
            rect = patches.Rectangle(
                (x, y),
                s["width"],
                s["height"],
                linewidth=1.5,
                edgecolor='blue',
                facecolor='lightblue',
                alpha=0.6
            )
            ax.add_patch(rect)

            # --- 自动文字偏移 ---
            name = obj.get("name", "unknown")
            ox, oy = offset_vectors[offset_index]
            offset_index = (offset_index + 1) % len(offset_vectors)  # 循环使用四个方向

            text_x = c["x"] + ox * s["width"] * 0.8
            text_y = c["y"] + oy * s["height"] * 0.8

            # 画连线
            ax.plot([c["x"], text_x], [c["y"], text_y], color='gray', linewidth=0.8, linestyle='--')

            # 写文字
            ax.text(
                text_x, text_y, name,
                fontsize=8.5, color='navy',
                ha='center', va='center',
                bbox=dict(facecolor='white', edgecolor='none', alpha=0.6, pad=0.5)
            )


    # ---------------- 坐标轴与显示 ----------------
    ax.set_xlim(-16, 12)
    ax.set_ylim(-12, 12)
    ax.grid(True, linestyle=':', alpha=0.4)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    yaml_path = os.path.join(os.path.dirname(__file__), "../apartment1_all.yaml")
    yaml_path = os.path.abspath(yaml_path)
    draw_scene_from_yaml(yaml_path)
