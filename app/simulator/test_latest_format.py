#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re

def test_latest_receiver_format():
    """测试最新的接收器数据格式解析"""
    
    # 最新的接收器输出格式
    test_lines = [
        "[12345ms] Pitch Error: 2.50 Roll Error: -1.20 Current Pitch: 1.30 Current Roll: -0.80 Right Target: 95.20 Left Target: 84.80",
        "[12350ms] Pitch Error: 1.80 Roll Error: 0.50 Current Pitch: 0.90 Current Roll: 0.20 Right Target: 92.10 Left Target: 87.90",
        "[12355ms] Pitch Error: -0.30 Roll Error: -2.10 Current Pitch: -0.15 Current Roll: -1.05 Right Target: 88.50 Left Target: 91.50",
        "[12360ms] Pitch Error: 0.00 Roll Error: 0.00 Current Pitch: 0.00 Current Roll: 0.00 Right Target: 90.00 Left Target: 90.00"
    ]
    
    def parse_latest_format(line):
        """解析最新的接收器数据格式"""
        try:
            # 提取时间戳
            timestamp_match = re.search(r'\[(\d+)ms\]', line)
            timestamp = int(timestamp_match.group(1)) if timestamp_match else 0
            
            # 提取数值 - 最新格式
            patterns = {
                'timestamp': r'\[(\d+)ms\]',
                'pitch_error': r'Pitch Error: ([-\d.]+)',
                'roll_error': r'Roll Error: ([-\d.]+)',
                'current_pitch': r'Current Pitch: ([-\d.]+)',
                'current_roll': r'Current Roll: ([-\d.]+)',
                'right_target': r'Right Target: ([-\d.]+)',
                'left_target': r'Left Target: ([-\d.]+)'
            }
            
            data = {}
            for key, pattern in patterns.items():
                match = re.search(pattern, line)
                if match:
                    data[key] = float(match.group(1))
            
            return data
            
        except Exception as e:
            print(f"Parse error: {e}")
            return None
    
    print("测试最新的接收器数据格式")
    print("=" * 60)
    print("接收器输出格式:")
    print("[timestamp] Pitch Error: X.XX Roll Error: X.XX Current Pitch: X.XX Current Roll: X.XX Right Target: XXX.X Left Target: XXX.X")
    print("=" * 60)
    
    for i, line in enumerate(test_lines, 1):
        print(f"\n测试数据 {i}:")
        print(f"原始数据: {line}")
        
        data = parse_latest_format(line)
        if data:
            print("解析结果:")
            print(f"  时间戳: {data.get('timestamp', 'N/A')}ms")
            print(f"  俯仰误差: {data.get('pitch_error', 'N/A')}")
            print(f"  横滚误差: {data.get('roll_error', 'N/A')}")
            print(f"  当前俯仰: {data.get('current_pitch', 'N/A')}°")
            print(f"  当前横滚: {data.get('current_roll', 'N/A')}°")
            print(f"  右舵目标: {data.get('right_target', 'N/A')}°")
            print(f"  左舵目标: {data.get('left_target', 'N/A')}°")
            
            # 验证数据完整性
            required_fields = ['timestamp', 'pitch_error', 'roll_error', 'current_pitch', 'current_roll', 'right_target', 'left_target']
            missing_fields = [field for field in required_fields if field not in data]
            if missing_fields:
                print(f"  警告: 缺少字段: {missing_fields}")
            else:
                print("  ✓ 所有字段解析成功")
        else:
            print("  ✗ 解析失败")
    
    print("\n" + "=" * 60)
    print("测试完成")

if __name__ == "__main__":
    test_latest_receiver_format() 