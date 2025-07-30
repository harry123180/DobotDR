#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CoordinateRecorder.py - 三次拍攝座標記錄工具
用於記錄CCD1三次檢測、Best算法、Find算法的座標結果
"""

import os
import time
from typing import List, Optional
from datetime import datetime


class CoordinateRecorder:
    """輕量級座標記錄器 - 記錄三次拍攝和算法結果"""
    
    def __init__(self, enabled: bool = True, save_directory: str = None):
        """
        初始化座標記錄器
        
        Args:
            enabled: 是否啟用記錄功能
            save_directory: 保存目錄，預設為執行檔同層的coordinate_logs資料夾
        """
        self.enabled = enabled
        
        if save_directory is None:
            # 預設保存到執行檔同層的coordinate_logs資料夾
            self.save_directory = os.path.join(
                os.path.dirname(os.path.abspath(__file__)), 
                'coordinate_logs'
            )
        else:
            self.save_directory = save_directory
        
        # 確保目錄存在
        if self.enabled:
            os.makedirs(self.save_directory, exist_ok=True)
        
        # 記錄資料
        self.reset()
    
    def reset(self):
        """重置記錄資料"""
        self.capture_results = []  # 三次拍攝結果
        self.best_results = []     # Best算法結果
        self.find_results = []     # Find算法結果
        self.tolerance_mm = 0.0    # Best容許範圍
        self.safe_distance = 0.0   # Find安全距離
        self.timestamp = datetime.now()
    
    def record_capture(self, capture_index: int, points: List) -> None:
        """
        記錄單次拍攝結果
        
        Args:
            capture_index: 拍攝次數 (0, 1, 2 對應第1, 2, 3次)
            points: CoordinatePoint列表
        """
        if not self.enabled:
            return
        
        # 確保capture_results有足夠空間
        while len(self.capture_results) <= capture_index:
            self.capture_results.append([])
        
        # 轉換為(x, y)座標列表
        coord_list = []
        for point in points:
            if hasattr(point, 'world_x') and hasattr(point, 'world_y'):
                if point.world_x is not None and point.world_y is not None:
                    coord_list.append((point.world_x, point.world_y))
        
        self.capture_results[capture_index] = coord_list
    
    def record_best_results(self, points: List, tolerance_mm: float) -> None:
        """
        記錄Best算法結果
        
        Args:
            points: Best算法結果點列表
            tolerance_mm: Best算法容許範圍
        """
        if not self.enabled:
            return
        
        self.tolerance_mm = tolerance_mm
        
        # 轉換為(x, y)座標列表
        coord_list = []
        for point in points:
            if hasattr(point, 'world_x') and hasattr(point, 'world_y'):
                if point.world_x is not None and point.world_y is not None:
                    coord_list.append((point.world_x, point.world_y))
        
        self.best_results = coord_list
    
    def record_find_results(self, points: List, safe_distance: float) -> None:
        """
        記錄Find算法結果
        
        Args:
            points: Find算法結果點列表
            safe_distance: Find算法安全距離
        """
        if not self.enabled:
            return
        
        self.safe_distance = safe_distance
        
        # 轉換為(x, y)座標列表
        coord_list = []
        for point in points:
            if hasattr(point, 'world_x') and hasattr(point, 'world_y'):
                if point.world_x is not None and point.world_y is not None:
                    coord_list.append((point.world_x, point.world_y))
        
        self.find_results = coord_list
    
    def save_to_file(self, filename: str = None) -> bool:
        """
        保存記錄到文件
        
        Args:
            filename: 文件名，若為None則自動生成
        
        Returns:
            是否保存成功
        """
        if not self.enabled:
            return True
        
        try:
            # 自動生成文件名
            if filename is None:
                timestamp_str = self.timestamp.strftime("%Y%m%d_%H%M%S")
                filename = f"第{self.timestamp.strftime('%H%M%S')}次CCD1三次檢測結果.txt"
            
            filepath = os.path.join(self.save_directory, filename)
            
            with open(filepath, 'w', encoding='utf-8') as f:
                # 寫入標題
                f.write(f"CCD1三次檢測座標記錄\n")
                f.write(f"記錄時間: {self.timestamp.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write("=" * 50 + "\n\n")
                
                # 寫入三次拍攝結果
                for i, capture_coords in enumerate(self.capture_results):
                    f.write(f"第{i+1}次檢測結果\n")
                    if capture_coords:
                        for x, y in capture_coords:
                            f.write(f"{x:.2f},{y:.2f}\n")
                    else:
                        f.write("無座標\n")
                    f.write("\n")
                
                # 寫入Best算法結果
                f.write("Best算法結果\n")
                if self.best_results:
                    for x, y in self.best_results:
                        f.write(f"{x:.2f},{y:.2f}\n")
                else:
                    f.write("無座標\n")
                f.write("\n")
                
                # 寫入Find算法結果
                f.write("Find算法結果\n")
                if self.find_results:
                    for x, y in self.find_results:
                        f.write(f"{x:.2f},{y:.2f}\n")
                else:
                    f.write("無座標\n")
                f.write("\n")
                
                # 寫入參數
                f.write(f"Best容許範圍: {self.tolerance_mm:.2f} mm\n")
                f.write(f"Find安全距離: {self.safe_distance:.2f} mm\n")
            
            return True
            
        except Exception as e:
            print(f"座標記錄保存失敗: {e}")
            return False
    
    def get_summary(self) -> str:
        """獲取記錄摘要"""
        if not self.enabled:
            return "座標記錄功能已停用"
        
        capture_counts = [len(coords) for coords in self.capture_results]
        best_count = len(self.best_results)
        find_count = len(self.find_results)
        
        summary = f"座標記錄摘要:\n"
        summary += f"  三次拍攝: {capture_counts} 個座標\n"
        summary += f"  Best結果: {best_count} 個座標\n"
        summary += f"  Find結果: {find_count} 個座標\n"
        summary += f"  Best容許範圍: {self.tolerance_mm:.2f} mm\n"
        summary += f"  Find安全距離: {self.safe_distance:.2f} mm"
        
        return summary


# 使用範例
def example_usage():
    """使用範例"""
    
    # 創建記錄器
    recorder = CoordinateRecorder(enabled=True)
    
    # 模擬三次拍攝記錄
    class MockPoint:
        def __init__(self, x, y):
            self.world_x = x
            self.world_y = y
    
    # 記錄第一次拍攝
    capture1_points = [MockPoint(-100.5, 250.3), MockPoint(-95.2, 255.8)]
    recorder.record_capture(0, capture1_points)
    
    # 記錄第二次拍攝
    capture2_points = [MockPoint(-100.8, 250.1), MockPoint(-94.9, 256.2)]
    recorder.record_capture(1, capture2_points)
    
    # 記錄第三次拍攝
    capture3_points = [MockPoint(-100.2, 250.5), MockPoint(-95.5, 255.5)]
    recorder.record_capture(2, capture3_points)
    
    # 記錄Best算法結果
    best_points = [MockPoint(-100.5, 250.3), MockPoint(-95.2, 255.8)]
    recorder.record_best_results(best_points, tolerance_mm=5.0)
    
    # 記錄Find算法結果
    find_points = [MockPoint(-100.5, 250.3)]
    recorder.record_find_results(find_points, safe_distance=20.0)
    
    # 保存到文件
    success = recorder.save_to_file()
    print(f"保存結果: {'成功' if success else '失敗'}")
    
    # 顯示摘要
    print(recorder.get_summary())


if __name__ == "__main__":
    example_usage()