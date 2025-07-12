#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np

def test_paper_plane_view():
    """测试新的纸飞机视图逻辑"""
    
    print("纸飞机视图逻辑测试")
    print("=" * 50)
    
    # 测试舵面角度计算
    print("舵面角度计算测试：")
    print("90度 = 回中位置")
    print(">90度 = 向下偏转")
    print("<90度 = 向上偏转")
    print()
    
    # 测试右舵面角度
    print("右舵面角度计算：")
    right_angles = [80, 90, 100]
    for angle in right_angles:
        angle_rad = math.radians(angle - 90)
        print(f"  右舵 {angle}° -> 角度差: {angle-90}° -> 弧度: {angle_rad:.3f}")
        if angle > 90:
            print(f"    -> 向下偏转 {angle-90}°")
        elif angle < 90:
            print(f"    -> 向上偏转 {90-angle}°")
        else:
            print(f"    -> 回中位置")
    
    print()
    
    # 测试左舵面角度（反向安装）
    print("左舵面角度计算（反向安装）：")
    left_angles = [80, 90, 100]
    for angle in left_angles:
        # 左舵反向：90度减去实际角度
        reversed_angle = 90 - angle
        angle_rad = math.radians(reversed_angle)
        print(f"  左舵 {angle}° -> 反向角度: {reversed_angle}° -> 弧度: {angle_rad:.3f}")
        if angle > 90:
            print(f"    -> 向上偏转 {angle-90}° (与右舵相反)")
        elif angle < 90:
            print(f"    -> 向下偏转 {90-angle}° (与右舵相反)")
        else:
            print(f"    -> 回中位置")
    
    print()
    
    # 测试舵面协同动作
    print("舵面协同动作测试：")
    test_cases = [
        (80, 100, "同向偏转10度"),
        (100, 80, "同向偏转10度"),
        (90, 90, "回中位置"),
        (85, 95, "同向偏转5度"),
        (95, 85, "同向偏转5度")
    ]
    
    for right_angle, left_angle, description in test_cases:
        right_offset = right_angle - 90
        left_offset = 90 - left_angle  # 左舵反向
        print(f"  右舵{right_angle}° + 左舵{left_angle}°: {description}")
        print(f"    右舵偏移: {right_offset}°, 左舵偏移: {left_offset}°")
        if right_offset == left_offset:
            print(f"    ✓ 舵面协同正确")
        else:
            print(f"    ✗ 舵面协同错误")
        print()
    
    # 测试视角说明
    print("视角说明：")
    print("  从尾部向前看（Tail View）")
    print("  机头被尾部挡住")
    print("  看到的主要是左右大翼（一字型）")
    print("  舵面在尾部，可以清楚看到偏转动作")
    print()
    
    # 测试坐标系统
    print("坐标系统：")
    print("  X轴：左右方向（负值向左，正值向右）")
    print("  Y轴：上下方向（负值向下，正值向上）")
    print("  原点：飞机尾部中心")
    print("  主翼：从(-0.3, 0.1)到(-0.1, 1.2)和(-0.1, -1.2)")
    print("  舵面：从(-0.6, ±0.05)开始偏转")
    
    print("\n" + "=" * 50)
    print("测试完成")

if __name__ == "__main__":
    test_paper_plane_view() 