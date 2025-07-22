from Robot import Robot, RobotConfig
import sys
import os

class System:
    def __init__(self, ccd1_ip="192.168.1.8"):
        # 設置完全靜默模式
        import logging
        
        # 關閉根日誌記錄器
        logging.getLogger().setLevel(logging.CRITICAL)
        
        # 關閉所有可能的日誌記錄器
        silent_loggers = [
            'CameraManager', 'CCD1VisionSystem', 'RobotSystem', 'ultralytics', 
            'PIL', 'camera_manager', 'OptimizedCameraManager'
        ]
        for logger_name in silent_loggers:
            logging.getLogger(logger_name).setLevel(logging.CRITICAL)
            logging.getLogger(logger_name).disabled = True
        
        # 禁用所有處理器
        for handler in logging.root.handlers[:]:
            logging.root.removeHandler(handler)
        
        config = RobotConfig(
            ccd1_ip=ccd1_ip, 
            auto_initialize=False, 
            enable_logging=False
        )
        
        self.robot = Robot(config)
        
        # 靜默初始化CCD1
        with open(os.devnull, 'w') as devnull:
            old_stdout = sys.stdout
            sys.stdout = devnull
            try:
                success = self.robot.init_ccd1()
                if not success:
                    sys.stdout = old_stdout
                    raise RuntimeError("CCD1初始化失敗")
                
                # 調整記憶體閾值防止模型被清理
                if self.robot.CCD1 and hasattr(self.robot.CCD1, 'config'):
                    self.robot.CCD1.config.memory_warning_threshold = 300.0
                    self.robot.CCD1.config.force_gc_frequency = 20
                    self.robot.CCD1.config.memory_cleanup_frequency = 50
                
                if (hasattr(self.robot.CCD1, 'yolo_detector') and 
                    self.robot.CCD1.yolo_detector and 
                    hasattr(self.robot.CCD1.yolo_detector, 'config')):
                    self.robot.CCD1.yolo_detector.config.memory_warning_threshold = 300.0
                
                # 首次檢測完成模型加載
                first_result = self.robot.CCD1.detect_objects()
                if not first_result.success:
                    sys.stdout = old_stdout
                    raise RuntimeError("首次檢測失敗")
                
                # 重置記憶體基準
                self.robot.CCD1.reset_memory_baseline()
                
            finally:
                sys.stdout = old_stdout
    
    def get_detections(self):
        """獲取DR_F和STACK的數量與座標"""
        try:
            # 完全靜默執行檢測
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                sys.stdout = devnull
                try:
                    # 檢查模型是否被清理，如果是則重新載入
                    if (hasattr(self.robot.CCD1, 'yolo_detector') and
                        hasattr(self.robot.CCD1.yolo_detector, 'model_manager') and 
                        not self.robot.CCD1.yolo_detector.model_manager.is_model_loaded()):
                        self.robot.CCD1.yolo_detector.model_manager.load_model(1)
                    
                    result = self.robot.CCD1.detect_objects()
                    
                finally:
                    sys.stdout = old_stdout
            
            if result.success:
                detections = result.detections_by_class
                coords = result.coordinates_by_class if hasattr(result, 'coordinates_by_class') else {}
                
                return {
                    'dr_f_count': detections.get(0, 0),
                    'stack_count': detections.get(1, 0), 
                    'dr_f_coords': coords.get(0, []),
                    'stack_coords': coords.get(1, [])
                }
            else:
                return {
                    'dr_f_count': 0,
                    'stack_count': 0,
                    'dr_f_coords': [],
                    'stack_coords': []
                }
        except Exception:
            return {
                'dr_f_count': 0,
                'stack_count': 0,
                'dr_f_coords': [],
                'stack_coords': []
            }
    
    def close(self):
        """關閉系統"""
        try:
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                sys.stdout = devnull
                try:
                    self.robot.close_ccd1()
                finally:
                    sys.stdout = old_stdout
        except:
            pass

# 使用範例
if __name__ == "__main__":
    system = System()
    try:
        # 獲取檢測結果
        result = system.get_detections()
        print(f"DR_F: {result['dr_f_count']} 個")
        print(f"STACK: {result['stack_count']} 個")
        print(f"DR_F座標: {len(result['dr_f_coords'])} 個")
        print(f"STACK座標: {len(result['stack_coords'])} 個")
        
        # 顯示前3個座標
        for i, (x, y) in enumerate(result['dr_f_coords'][:3]):
            print(f"DR_F {i+1}: ({x:.1f}, {y:.1f})")
            
    finally:
        system.close()