from Robot import Robot, RobotConfig
import sys
import os
import logging

class System:
    def __init__(self, 
                 ccd1_ip="192.168.1.8",
                 ccd3_ip="192.168.1.10", 
                 vp_ip="192.168.1.7",
                 dobot_ip="192.168.1.6"):
        """
        初始化系統所有子模組 - 完全靜默模式
        
        Args:
            ccd1_ip: CCD1視覺系統IP
            ccd3_ip: CCD3角度檢測系統IP  
            vp_ip: 震動盤系統IP
            dobot_ip: Dobot機械臂IP
        """
        # 設置完全靜默模式
        self._setup_silent_logging()
        
        # 創建機器人配置
        config = RobotConfig(
            ccd1_ip=ccd1_ip,
            ccd3_ip=ccd3_ip,
            vp_ip=vp_ip,
            dobot_ip=dobot_ip,
            auto_initialize=False,
            enable_logging=True
        )
        
        # 創建機器人實例
        self.robot = Robot(config)
        
        # 子系統初始化狀態追蹤
        self.initialized_systems = {
            'CCD1': False,
            'CCD3': False,
            'VP': False,
            'Gripper': False,
            'Dobot': False
        }
        
        # 靜默初始化所有可用子系統
        self._silent_initialize_all()
    
    def _setup_silent_logging(self):
        """設置完全靜默日誌模式"""
        # 關閉根日誌記錄器
        logging.getLogger().setLevel(logging.CRITICAL)
        
        # 關閉所有可能的日誌記錄器
        silent_loggers = [
            'CameraManager', 'CCD1VisionSystem', 'RobotSystem', 'ultralytics',
            'PIL', 'camera_manager', 'OptimizedCameraManager', 'CCD3HeadlessDetector',
            'ProductionVibrationPlate', 'GripperHighLevelAPI', 'DobotController',
            'DoBot', 'ModbusClient', 'pymodbus'
        ]
        
        for logger_name in silent_loggers:
            logger = logging.getLogger(logger_name)
            logger.setLevel(logging.CRITICAL)
            logger.disabled = True
        
        # 禁用所有處理器
        for handler in logging.root.handlers[:]:
            logging.root.removeHandler(handler)
    
    def _silent_initialize_all(self):
        """靜默初始化所有子系統"""
        with open(os.devnull, 'w') as devnull:
            old_stdout = sys.stdout
            old_stderr = sys.stderr
            sys.stdout = devnull
            sys.stderr = devnull
            
            try:
                # 初始化CCD1視覺系統
                self._init_ccd1()
                
                # 初始化CCD3角度檢測系統
                self._init_ccd3()
                
                # 初始化震動盤系統
                self._init_vp()
                
                # 初始化夾爪系統
                self._init_gripper()
                
                # 初始化Dobot機械臂
                self._init_dobot()
                
            finally:
                sys.stdout = old_stdout
                sys.stderr = old_stderr
    
    def _init_ccd1(self):
        """靜默初始化CCD1視覺系統"""
        try:
            success = self.robot.init_ccd1()
            if success and self.robot.CCD1:
                # 調整記憶體閾值防止模型被清理
                if hasattr(self.robot.CCD1, 'config'):
                    self.robot.CCD1.config.memory_warning_threshold = 300.0
                    self.robot.CCD1.config.force_gc_frequency = 20
                    self.robot.CCD1.config.memory_cleanup_frequency = 50
                
                if (hasattr(self.robot.CCD1, 'yolo_detector') and 
                    self.robot.CCD1.yolo_detector and 
                    hasattr(self.robot.CCD1.yolo_detector, 'config')):
                    self.robot.CCD1.yolo_detector.config.memory_warning_threshold = 300.0
                
                # 首次檢測完成模型加載
                first_result = self.robot.CCD1.detect_objects()
                if first_result.success:
                    # 重置記憶體基準
                    self.robot.CCD1.reset_memory_baseline()
                    self.initialized_systems['CCD1'] = True
                    
        except Exception:
            self.initialized_systems['CCD1'] = False
    
    def _init_ccd3(self):
        """靜默初始化CCD3角度檢測系統"""
        try:
            success = self.robot.init_ccd3()
            if success:
                self.initialized_systems['CCD3'] = True
        except Exception:
            self.initialized_systems['CCD3'] = False
    
    def _init_vp(self):
        """靜默初始化震動盤系統"""
        try:
            success = self.robot.init_vp()
            if success:
                self.initialized_systems['VP'] = True
        except Exception:
            self.initialized_systems['VP'] = False
    
    def _init_gripper(self):
        """靜默初始化夾爪系統"""
        try:
            success = self.robot.init_gripper()
            if success:
                self.initialized_systems['Gripper'] = True
        except Exception:
            self.initialized_systems['Gripper'] = False
    
    def _init_dobot(self):
        """靜默初始化Dobot機械臂"""
        try:
            success = self.robot.init_dobot()
            if success:
                self.initialized_systems['Dobot'] = True
        except Exception:
            self.initialized_systems['Dobot'] = False
    
    # ==================== CCD1視覺檢測接口 ====================
    
    def get_detections(self):
        """獲取DR_F和STACK的數量與座標"""
        if not self.initialized_systems['CCD1']:
            return {
                'dr_f_count': 0,
                'stack_count': 0,
                'dr_f_coords': [],
                'stack_coords': []
            }
        
        try:
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
    
    def switch_ccd1_model(self, model_id: int) -> bool:
        """切換CCD1檢測模型"""
        if not self.initialized_systems['CCD1']:
            return False
        
        try:
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                sys.stdout = devnull
                try:
                    return self.robot.CCD1_切換模型(model_id)
                finally:
                    sys.stdout = old_stdout
        except Exception:
            return False
    
    # ==================== CCD3角度檢測接口 ====================
    
    def get_angle(self) -> float:
        """獲取CCD3檢測到的角度"""
        if not self.initialized_systems['CCD3']:
            return 0.0
        
        try:
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                sys.stdout = devnull
                try:
                    angle = self.robot.CCD3_獲得角度()
                    return angle if angle is not None else 0.0
                finally:
                    sys.stdout = old_stdout
        except Exception:
            return 0.0
    
    def get_center_point(self) -> tuple:
        """獲取CCD3檢測到的中心點座標"""
        if not self.initialized_systems['CCD3']:
            return (0, 0)
        
        try:
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                sys.stdout = devnull
                try:
                    center = self.robot.CCD3_獲得中心點()
                    return center if center is not None else (0, 0)
                finally:
                    sys.stdout = old_stdout
        except Exception:
            return (0, 0)
    
    # ==================== 震動盤控制接口 ====================
    
    def vp_start_vibration(self, mode: str, intensity: int, frequency: int = 100, duration: float = 0) -> bool:
        """開始震動盤震動"""
        if not self.initialized_systems['VP']:
            return False
        
        try:
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                sys.stdout = devnull
                try:
                    return self.robot.VP_開始震動(mode, intensity, frequency, duration)
                finally:
                    sys.stdout = old_stdout
        except Exception:
            return False
    
    def vp_stop_vibration(self) -> bool:
        """停止震動盤震動"""
        if not self.initialized_systems['VP']:
            return False
        
        try:
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                sys.stdout = devnull
                try:
                    return self.robot.VP_停止震動()
                finally:
                    sys.stdout = old_stdout
        except Exception:
            return False
    
    def vp_set_backlight(self, enabled: bool, brightness: int = 50) -> bool:
        """設定震動盤背光"""
        if not self.initialized_systems['VP']:
            return False
        
        try:
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                sys.stdout = devnull
                try:
                    return self.robot.VP_設定背光(enabled, brightness)
                finally:
                    sys.stdout = old_stdout
        except Exception:
            return False
    
    # ==================== 夾爪控制接口 ====================
    
    def gripper_quick_close(self) -> bool:
        """夾爪快速關閉"""
        if not self.initialized_systems['Gripper']:
            return False
        
        try:
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                sys.stdout = devnull
                try:
                    return self.robot.Gripper_快速關閉()
                finally:
                    sys.stdout = old_stdout
        except Exception:
            return False
    
    def gripper_quick_open(self) -> bool:
        """夾爪快速開啟"""
        if not self.initialized_systems['Gripper']:
            return False
        
        try:
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                sys.stdout = devnull
                try:
                    return self.robot.Gripper_快速開啟()
                finally:
                    sys.stdout = old_stdout
        except Exception:
            return False
    
    def gripper_smart_grip(self, target_position: int = 420) -> bool:
        """夾爪智能夾取"""
        if not self.initialized_systems['Gripper']:
            return False
        
        try:
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                sys.stdout = devnull
                try:
                    return self.robot.Gripper_智能夾取(target_position)
                finally:
                    sys.stdout = old_stdout
        except Exception:
            return False
    
    def gripper_smart_release(self, release_position: int = 470) -> bool:
        """夾爪智能釋放"""
        if not self.initialized_systems['Gripper']:
            return False
        
        try:
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                sys.stdout = devnull
                try:
                    return self.robot.Gripper_智能釋放(release_position)
                finally:
                    sys.stdout = old_stdout
        except Exception:
            return False
    
    # ==================== Dobot機械臂控制接口 ====================
    
    def dobot_enable(self) -> bool:
        """使能機械臂"""
        if not self.initialized_systems['Dobot']:
            return False
        
        try:
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                sys.stdout = devnull
                try:
                    return self.robot.Dobot.使能機械臂()
                finally:
                    sys.stdout = old_stdout
        except Exception:
            return False
    
    def dobot_disable(self) -> bool:
        """下使能機械臂"""
        if not self.initialized_systems['Dobot']:
            return False
        
        try:
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                sys.stdout = devnull
                try:
                    return self.robot.Dobot.下使能機械臂()
                finally:
                    sys.stdout = old_stdout
        except Exception:
            return False
    
    def dobot_movj_point(self, point_name: str) -> bool:
        """關節運動到指定點位"""
        if not self.initialized_systems['Dobot']:
            return False
        
        try:
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                sys.stdout = devnull
                try:
                    return self.robot.Dobot.JointMovJ(point_name)
                finally:
                    sys.stdout = old_stdout
        except Exception:
            return False
    
    def dobot_movl_point(self, point_name: str) -> bool:
        """直線運動到指定點位"""
        if not self.initialized_systems['Dobot']:
            return False
        
        try:
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                sys.stdout = devnull
                try:
                    return self.robot.Dobot.MovL(point_name)
                finally:
                    sys.stdout = old_stdout
        except Exception:
            return False
    
    def dobot_movj_coord(self, x: float, y: float, z: float, r: float) -> bool:
        """關節運動到指定座標"""
        if not self.initialized_systems['Dobot']:
            return False
        
        try:
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                sys.stdout = devnull
                try:
                    return self.robot.Dobot.MovJ_座標(x, y, z, r)
                finally:
                    sys.stdout = old_stdout
        except Exception:
            return False
    
    def dobot_sync(self) -> bool:
        """等待運動完成"""
        if not self.initialized_systems['Dobot']:
            return False
        
        try:
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                sys.stdout = devnull
                try:
                    return self.robot.Dobot.Sync()
                finally:
                    sys.stdout = old_stdout
        except Exception:
            return False
    
    def vibration_feed(self, feed_duration: float = 2.0, pulse_delay_percent: int = 30,
                      pulse_high_time: float = 0.3, pulse_low_time: float = 0.3, 
                      pulse_count: int = 1) -> bool:
        """震動進料控制"""
        if not self.initialized_systems['Dobot']:
            return False
        
        try:
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                sys.stdout = devnull
                try:
                    return self.robot.震動進料(feed_duration, pulse_delay_percent, 
                                           pulse_high_time, pulse_low_time, pulse_count)
                finally:
                    sys.stdout = old_stdout
        except Exception:
            return False
    
    # ==================== 系統狀態和控制接口 ====================
    
    def get_system_status(self) -> dict:
        """獲取系統狀態"""
        return {
            'initialized_systems': self.initialized_systems.copy(),
            'total_initialized': sum(self.initialized_systems.values()),
            'available_systems': {
                'CCD1': self.robot.CCD1 is not None,
                'CCD3': self.robot.CCD3 is not None,
                'VP': self.robot.VP is not None,
                'Gripper': self.robot.Gripper is not None,
                'Dobot': self.robot.Dobot is not None
            }
        }
    
    def emergency_stop(self) -> bool:
        """緊急停止所有系統"""
        try:
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                sys.stdout = devnull
                try:
                    return self.robot.緊急停止()
                finally:
                    sys.stdout = old_stdout
        except Exception:
            return False
    
    def clear_emergency_stop(self) -> bool:
        """清除緊急停止狀態"""
        try:
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                sys.stdout = devnull
                try:
                    return self.robot.清除緊急停止()
                finally:
                    sys.stdout = old_stdout
        except Exception:
            return False
    
    def close(self):
        """關閉系統所有子模組"""
        try:
            with open(os.devnull, 'w') as devnull:
                old_stdout = sys.stdout
                old_stderr = sys.stderr
                sys.stdout = devnull
                sys.stderr = devnull
                
                try:
                    # 按順序關閉各子系統
                    if self.initialized_systems['CCD1']:
                        self.robot.close_ccd1()
                        self.initialized_systems['CCD1'] = False
                    
                    if self.initialized_systems['CCD3']:
                        self.robot.close_ccd3()
                        self.initialized_systems['CCD3'] = False
                    
                    if self.initialized_systems['VP']:
                        self.robot.close_vp()
                        self.initialized_systems['VP'] = False
                    
                    if self.initialized_systems['Gripper']:
                        self.robot.close_gripper()
                        self.initialized_systems['Gripper'] = False
                    
                    if self.initialized_systems['Dobot']:
                        self.robot.close_dobot()
                        self.initialized_systems['Dobot'] = False
                    
                    # 最後斷開機器人連接
                    self.robot.斷開連接()
                    
                finally:
                    sys.stdout = old_stdout
                    sys.stderr = old_stderr
        except:
            pass


# 使用範例
if __name__ == "__main__":
    system = System()
    try:
        # 顯示系統狀態
        status = system.get_system_status()
        print("=== 系統初始化狀態 ===")
        for name, initialized in status['initialized_systems'].items():
            print(f"{name}: {'✓ 成功' if initialized else '✗ 失敗'}")
        print(f"總計: {status['total_initialized']}/5 個系統初始化成功")
        
        # CCD1檢測範例
        if status['initialized_systems']['CCD1']:
            print("\n=== CCD1視覺檢測 ===")
            result = system.get_detections()
            print(f"DR_F: {result['dr_f_count']} 個")
            print(f"STACK: {result['stack_count']} 個")
            print(f"DR_F座標: {len(result['dr_f_coords'])} 個")
            print(f"STACK座標: {len(result['stack_coords'])} 個")
            
            # 顯示前3個座標
            for i, (x, y) in enumerate(result['dr_f_coords'][:3]):
                print(f"DR_F {i+1}: ({x:.1f}, {y:.1f})")
        
        # CCD3角度檢測範例
        if status['initialized_systems']['CCD3']:
            print("\n=== CCD3角度檢測 ===")
            angle = system.get_angle()
            center = system.get_center_point()
            print(f"檢測角度: {angle:.2f}°")
            print(f"中心點: {center}")
        
        # 震動盤控制範例
        if status['initialized_systems']['VP']:
            print("\n=== 震動盤控制 ===")
            system.vp_set_backlight(True, 50)
            system.vp_start_vibration("vertical", 60, 100, 1)
            system.vp_stop_vibration()
            print("震動盤測試完成")
        
        # 夾爪控制範例
        if status['initialized_systems']['Gripper']:
            print("\n=== 夾爪控制 ===")
            system.gripper_quick_open()
            system.gripper_smart_grip(420)
            system.gripper_smart_release(470)
            print("夾爪測試完成")
        
        # 機械臂控制範例
        if status['initialized_systems']['Dobot']:
            print("\n=== 機械臂控制 ===")
            system.dobot_enable()
            print("機械臂已使能")
            
    except KeyboardInterrupt:
        print("\n收到中斷信號，正在關閉系統...")
    except Exception as e:
        print(f"執行異常: {e}")
    finally:
        system.close()
        print("系統已安全關閉")