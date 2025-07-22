# -*- coding: utf-8 -*-
"""
enhanced_test.py - CCD1增強崩潰測試
針對真實YOLOv11模型和座標轉換的崩潰測試
"""

import sys
import os
import time
import numpy as np
import cv2
import psutil
import gc
import threading
import signal
import traceback
from typing import Optional, Tuple
from dataclasses import dataclass

# 導入CCD1模組
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from camera_manager import OptimizedCameraManager, CameraConfig, CameraMode, PixelFormat
    CAMERA_AVAILABLE = True
except ImportError as e:
    print(f"❌ 無法導入camera_manager: {e}")
    CAMERA_AVAILABLE = False

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError as e:
    print(f"❌ 無法導入YOLOv11: {e}")
    YOLO_AVAILABLE = False


class CrashDetector:
    """崩潰檢測器"""
    
    def __init__(self):
        self.crash_detected = False
        self.crash_info = None
        self.setup_crash_handlers()
    
    def setup_crash_handlers(self):
        """設置崩潰處理器"""
        def crash_handler(signum, frame):
            self.crash_detected = True
            self.crash_info = {
                'signal': signum,
                'frame': frame,
                'traceback': traceback.format_stack(frame)
            }
            print(f"💥 檢測到程式崩潰信號: {signum}")
            print(f"💥 崩潰位置: {traceback.format_stack(frame)[-2]}")
        
        # 註冊信號處理器
        signal.signal(signal.SIGTERM, crash_handler)
        signal.signal(signal.SIGINT, crash_handler)
        
        # Windows特有
        if hasattr(signal, 'SIGBREAK'):
            signal.signal(signal.SIGBREAK, crash_handler)


class EnhancedCCD1Test:
    """增強版CCD1測試"""
    
    def __init__(self):
        self.camera_manager = None
        self.yolo_model = None
        self.crash_detector = CrashDetector()
        self.test_results = []
        
        # 模擬座標轉換器
        self.coordinate_transformer = None
    
    def setup_real_yolo(self) -> bool:
        """設置真實YOLOv11模型"""
        if not YOLO_AVAILABLE:
            print("⚠️ YOLOv11不可用")
            return False
        
        try:
            # 嘗試載入真實模型
            model_files = ["best.pt", "yolo11n.pt", "yolov8n.pt"]
            
            for model_file in model_files:
                if os.path.exists(model_file):
                    print(f"🔄 載入模型: {model_file}")
                    self.yolo_model = YOLO(model_file)
                    print(f"✅ 成功載入模型: {model_file}")
                    return True
            
            # 如果沒有本地模型，下載預訓練模型
            print("📥 下載YOLOv11nano模型...")
            self.yolo_model = YOLO("yolo11n.pt")  # 自動下載
            print("✅ 成功下載並載入YOLOv11nano模型")
            return True
            
        except Exception as e:
            print(f"❌ 載入YOLOv11模型失敗: {e}")
            return False
    
    def setup_coordinate_transformer(self):
        """設置座標轉換器（模擬CCD1的轉換邏輯）"""
        # 模擬標定數據
        self.camera_matrix = np.array([
            [2500.0, 0.0, 1296.0],
            [0.0, 2500.0, 972.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        
        self.dist_coeffs = np.zeros((1, 5), dtype=np.float64)
        
        # 模擬外參
        self.rvec = np.array([0.1, 0.2, 0.3], dtype=np.float64)
        self.tvec = np.array([0.0, 0.0, 500.0], dtype=np.float64)
        
        print("✅ 座標轉換器設置完成")
    
    def pixel_to_world(self, pixel_coords):
        """模擬像素到世界座標轉換（複製CCD1邏輯）"""
        try:
            if not pixel_coords:
                return []
            
            world_coords = []
            for px, py in pixel_coords:
                # 模擬去畸變
                pixel_point = np.array([[[float(px), float(py)]]], dtype=np.float32)
                undistorted_points = cv2.undistortPoints(
                    pixel_point, 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                
                x_norm, y_norm = undistorted_points[0][0]
                normalized_coords = np.array([x_norm, y_norm, 1.0])
                
                # 模擬深度計算
                rotation_matrix, _ = cv2.Rodrigues(self.rvec)
                R3 = rotation_matrix[2, :]
                denominator = np.dot(R3, normalized_coords)
                
                if abs(denominator) < 1e-6:
                    continue
                
                depth_scale = (0 - self.tvec[2]) / denominator
                camera_point = depth_scale * normalized_coords
                
                # 轉換到世界座標
                tvec_3d = self.tvec.reshape(3)
                translated_point = camera_point - tvec_3d
                world_point_3d = np.dot(rotation_matrix.T, translated_point)
                
                world_x = float(world_point_3d[0])
                world_y = float(world_point_3d[1])
                
                world_coords.append((world_x, world_y))
            
            return world_coords
            
        except Exception as e:
            print(f"❌ 座標轉換失敗: {e}")
            raise e  # 重新拋出異常以檢測崩潰
    
    def test_yolo_detection_intensive(self, image: np.ndarray) -> dict:
        """測試真實YOLOv11檢測"""
        try:
            if self.yolo_model is None:
                return {'success': False, 'error': 'No model loaded'}
            
            print(f"🔍 執行YOLOv11檢測: 圖像形狀={image.shape}")
            
            # 真實YOLOv11檢測
            results = self.yolo_model(image, conf=0.8, verbose=False)
            
            # 處理檢測結果
            detections = []
            if results and len(results) > 0:
                result = results[0]
                if result.boxes is not None and len(result.boxes) > 0:
                    boxes = result.boxes.cpu().numpy()
                    
                    for box in boxes:
                        class_id = int(box.cls[0])
                        confidence = float(box.conf[0])
                        x1, y1, x2, y2 = box.xyxy[0]
                        center_x = float((x1 + x2) / 2)
                        center_y = float((y1 + y2) / 2)
                        
                        detections.append({
                            'class_id': class_id,
                            'confidence': confidence,
                            'center': (center_x, center_y)
                        })
            
            print(f"✅ 檢測完成: 發現{len(detections)}個目標")
            
            # 🔥 關鍵測試：座標轉換（CCD1崩潰點）
            if detections:
                pixel_coords = [det['center'] for det in detections]
                print(f"🌍 開始座標轉換: {len(pixel_coords)}個點")
                
                world_coords = self.pixel_to_world(pixel_coords)
                print(f"✅ 座標轉換完成: {len(world_coords)}個點")
                
                return {
                    'success': True,
                    'detections': len(detections),
                    'pixel_coords': pixel_coords,
                    'world_coords': world_coords,
                    'real_model': True
                }
            else:
                return {
                    'success': True,
                    'detections': 0,
                    'pixel_coords': [],
                    'world_coords': [],
                    'real_model': True
                }
            
        except Exception as e:
            print(f"💥 YOLOv11檢測異常: {type(e).__name__}: {e}")
            print(f"💥 詳細錯誤: {traceback.format_exc()}")
            
            # 記錄崩潰信息
            self.crash_detector.crash_detected = True
            self.crash_detector.crash_info = {
                'exception_type': type(e).__name__,
                'exception_message': str(e),
                'traceback': traceback.format_exc()
            }
            
            raise e  # 重新拋出以模擬真實崩潰
    
    def test_crash_scenarios(self) -> dict:
        """測試各種崩潰場景"""
        print("\n🧨 開始崩潰場景測試")
        print("=" * 60)
        
        if not CAMERA_AVAILABLE:
            print("❌ 相機不可用，跳過測試")
            return {'success': False, 'error': 'Camera not available'}
        
        # 設置相機
        config = CameraConfig(
            name="crash_test_camera",
            ip="192.168.1.8",
            bandwidth_limit_mbps=200,
            frame_rate=5.0,
            exposure_time=50000.0,
            use_latest_frame_only=True
        )
        
        self.camera_manager = OptimizedCameraManager()
        self.camera_manager.add_camera("crash_test_camera", config)
        
        if not self.camera_manager.connect_camera("crash_test_camera"):
            print("❌ 相機連接失敗")
            return {'success': False, 'error': 'Camera connection failed'}
        
        self.camera_manager.start_streaming(["crash_test_camera"])
        
        # 設置YOLOv11
        if not self.setup_real_yolo():
            print("❌ YOLOv11設置失敗")
            return {'success': False, 'error': 'YOLO setup failed'}
        
        # 設置座標轉換器
        self.setup_coordinate_transformer()
        
        # 開始高強度測試
        crash_results = []
        
        for i in range(50):  # 50次高強度測試
            try:
                print(f"\n🔄 崩潰測試 {i+1}/50")
                
                # 檢查記憶體狀態
                memory_percent = psutil.virtual_memory().percent
                print(f"📊 記憶體使用率: {memory_percent:.1f}%")
                
                # 拍照
                frame_data = self.camera_manager.capture_new_frame("crash_test_camera", timeout=2000)
                if frame_data is None:
                    print("❌ 拍照失敗")
                    continue
                
                # 圖像處理
                image = frame_data.data
                if len(image.shape) == 2:
                    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                
                # 🔥 關鍵測試：YOLOv11檢測 + 座標轉換
                detection_result = self.test_yolo_detection_intensive(image)
                
                crash_results.append({
                    'iteration': i + 1,
                    'success': True,
                    'detections': detection_result.get('detections', 0),
                    'memory_percent': psutil.virtual_memory().percent
                })
                
                print(f"   ✅ 測試 {i+1} 成功")
                
                # 檢查崩潰
                if self.crash_detector.crash_detected:
                    print(f"💥 檢測到崩潰在第 {i+1} 次迭代！")
                    break
                
                # 高頻測試，減少延遲
                time.sleep(0.1)
                
            except KeyboardInterrupt:
                print(f"⏹️ 用戶中斷測試")
                break
                
            except Exception as e:
                print(f"💥 第 {i+1} 次測試崩潰: {type(e).__name__}: {e}")
                crash_results.append({
                    'iteration': i + 1,
                    'success': False,
                    'error': str(e),
                    'error_type': type(e).__name__
                })
                
                # 模擬CCD1的崩潰行為
                if isinstance(e, (MemoryError, RuntimeError, SystemError)):
                    print(f"💥 致命錯誤，模擬程式崩潰")
                    break
        
        return {
            'success': True,
            'crash_detected': self.crash_detector.crash_detected,
            'crash_info': self.crash_detector.crash_info,
            'completed_iterations': len([r for r in crash_results if r.get('success', False)]),
            'total_iterations': len(crash_results),
            'results': crash_results
        }
    
    def cleanup(self):
        """清理資源"""
        try:
            if self.camera_manager:
                self.camera_manager.shutdown()
            print("✅ 資源清理完成")
        except Exception as e:
            print(f"❌ 清理失敗: {e}")


def main():
    """主測試函數"""
    print("🚀 CCD1增強崩潰測試開始")
    print("=" * 60)
    
    test = EnhancedCCD1Test()
    
    try:
        # 執行崩潰場景測試
        result = test.test_crash_scenarios()
        
        # 輸出結果
        print("\n" + "=" * 60)
        print("📋 崩潰測試結果總結")
        print("=" * 60)
        
        if result['success']:
            print(f"崩潰檢測: {'✅ 是' if result['crash_detected'] else '❌ 否'}")
            print(f"完成迭代: {result['completed_iterations']}/{result['total_iterations']}")
            
            if result['crash_detected']:
                print(f"\n💥 崩潰詳情:")
                crash_info = result.get('crash_info', {})
                print(f"   錯誤類型: {crash_info.get('exception_type', 'Unknown')}")
                print(f"   錯誤訊息: {crash_info.get('exception_message', 'Unknown')}")
                
                print(f"\n🔍 可能的CCD1崩潰原因:")
                print(f"   1. YOLOv11模型載入/推論問題")
                print(f"   2. 座標轉換計算異常")
                print(f"   3. 海康SDK底層問題")
                print(f"   4. OpenCV函數異常")
            else:
                print(f"\n✅ 未檢測到崩潰，CCD1問題可能是:")
                print(f"   1. 硬體問題（相機、網路）")
                print(f"   2. 作業系統層級問題")
                print(f"   3. 其他環境因素")
        else:
            print(f"測試失敗: {result.get('error', 'Unknown error')}")
    
    except KeyboardInterrupt:
        print(f"\n⏹️ 測試被用戶中斷")
    
    except Exception as e:
        print(f"\n❌ 測試異常: {e}")
        print(f"詳細錯誤: {traceback.format_exc()}")
    
    finally:
        test.cleanup()


if __name__ == "__main__":
    main()