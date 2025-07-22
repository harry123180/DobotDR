#!/usr/bin/env python3
"""
CCD3無頭檢測系統 - 最簡潔測試程序
執行10000次檢測，輸出角度值和記憶體使用率
"""

import sys
import os
import time
# 導入CCD3模組
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from CCD3_Headless_Vision_System import CCD3HeadlessDetector, DetectionMethod

def main():
    # 創建檢測器
    detector = CCD3HeadlessDetector(camera_ip="192.168.1.10")
    
    try:
        # 初始化相機
        print("初始化相機...")
        if not detector.initialize_camera():
            print("相機初始化失敗")
            return
        
        # 設置DR模式，不儲存圖片
        detector.set_detection_params(detection_method=DetectionMethod.DR_MIN_RECT)
        detector.set_image_save_options(save_original=False, save_processed=False, save_result=False)
        
        print("開始10000次檢測測試...")
        
        # 執行10000次檢測
        for i in range(1, 10001):
            result = detector.capture_and_detect()
            memory = detector.get_memory_status()
            
            if result.success:
                print(f"第{i:5d}次: 角度={result.angle:7.2f}° | 記憶體={memory['rss_mb']:6.1f}MB")
            else:
                print(f"第{i:5d}次: 檢測失敗 | 記憶體={memory['rss_mb']:6.1f}MB")
            
            # 每1000次清理記憶體
            if i % 1000 == 0:
                detector.cleanup_memory()
                print(f"--- 完成{i}次檢測，記憶體已清理 ---")
            time.sleep(2)
    
    finally:
        # 清理資源
        detector.disconnect()
        print("測試完成")

if __name__ == "__main__":
    main()