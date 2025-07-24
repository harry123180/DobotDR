# -*- coding: utf-8 -*-
"""
test_CCD3.py - CCD3角度檢測系統獨立測試
"""

import os
import sys

# 添加專案路徑
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, '..', 'CCD3'))

try:
    from CCD3_Headless_Vision_System import CCD3HeadlessDetector
    print("CCD3模組導入成功")
except ImportError as e:
    print(f"CCD3模組導入失敗: {e}")
    exit(1)

def main():
    """CCD3測試主函數"""
    print("=== CCD3獨立測試 ===")
    
    # 創建CCD3實例
    ccd3 = CCD3HeadlessDetector(camera_ip="192.168.1.10")
    print(f"初始化前狀態: {ccd3.is_initialized}")
    
    # 初始化相機
    init_result = ccd3.initialize_camera()
    print(f"初始化結果: {init_result}")
    print(f"初始化後狀態: {ccd3.is_initialized}")
    
    if init_result:
        # 測試檢測
        result = ccd3.capture_and_detect()
        if result and hasattr(result, 'success') and result.success:
            print(f"檢測成功 - 角度: {result.angle}°")
        else:
            print("檢測失敗")

if __name__ == "__main__":
    main()