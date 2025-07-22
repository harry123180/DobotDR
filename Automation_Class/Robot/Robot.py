import os
import sys
import time
import threading
import json
import logging
from typing import Dict, Any, Optional, Tuple, List, Union
from dataclasses import dataclass
from datetime import datetime
from logging.handlers import RotatingFileHandler

# 設定路徑
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)

# 添加各模組路徑
sys.path.append(os.path.join(project_root, 'API'))
sys.path.append(os.path.join(project_root, 'CCD1'))
sys.path.append(os.path.join(project_root, 'CCD3'))
sys.path.append(os.path.join(project_root, 'VP'))
sys.path.append(os.path.join(project_root, 'Gripper'))
sys.path.append(current_dir)

# 導入各子系統
try:
    from CCD1_Headless_Vision_System import CCD1VisionSystem, CCD1VisionConfig
    CCD1_AVAILABLE = True
except ImportError as e:
    print(f"CCD1模組導入失敗: {e}")
    CCD1_AVAILABLE = False

try:
    from CCD3_Headless_Vision_System import CCD3HeadlessDetector, DetectionMethod
    CCD3_AVAILABLE = True
except ImportError as e:
    print(f"CCD3模組導入失敗: {e}")
    CCD3_AVAILABLE = False

try:
    from vibration_plate_controller import ProductionVibrationPlate
    VP_AVAILABLE = True
except ImportError as e:
    print(f"VP模組導入失敗: {e}")
    VP_AVAILABLE = False

try:
    from GripperHighLevel_DirectSerial import GripperHighLevelAPI
    GRIPPER_AVAILABLE = True
except ImportError as e:
    print(f"Gripper模組導入失敗: {e}")
    GRIPPER_AVAILABLE = False

try:
    from dobot_api import DobotApiDashboard, DobotApiMove
    DOBOT_AVAILABLE = True
except ImportError as e:
    print(f"Dobot API模組導入失敗: {e}")
    DOBOT_AVAILABLE = False


@dataclass
class RobotConfig:
    """機器人配置結構"""
    # 網路配置
    dobot_ip: str = "192.168.1.6"
    ccd1_ip: str = "192.168.1.8"
    ccd3_ip: str = "192.168.1.10"
    vp_ip: str = "192.168.1.7"
    
    # 系統配置
    auto_initialize: bool = True
    enable_logging: bool = True
    log_level: str = "INFO"
    
    # 錯誤處理配置
    auto_retry_on_error: bool = True
    max_retry_count: int = 3
    retry_delay: float = 1.0
    
    # 安全配置
    enable_collision_detection: bool = True
    collision_level: int = 3
    emergency_stop_timeout: float = 5.0


class RobotPointManager:
    """機器人點位管理器"""
    
    def __init__(self, points_file: str = "saved_points/robot_points.json"):
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.points_file = os.path.join(self.current_dir, points_file)
        self.points = {}
        self.load_points()
    
    def load_points(self):
        """載入點位數據"""
        try:
            if os.path.exists(self.points_file):
                with open(self.points_file, 'r', encoding='utf-8') as f:
                    points_data = json.load(f)
                    
                # 轉換為以名稱為鍵的字典
                for point in points_data:
                    self.points[point['name']] = point
                    
                print(f"已載入 {len(self.points)} 個點位")
            else:
                print(f"點位檔案不存在: {self.points_file}")
                
        except Exception as e:
            print(f"載入點位失敗: {e}")
    
    def get_point(self, name: str) -> Optional[Dict[str, Any]]:
        """根據名稱獲取點位"""
        return self.points.get(name)
    
    def get_all_points(self) -> Dict[str, Dict[str, Any]]:
        """獲取所有點位"""
        return self.points.copy()
    
    def get_point_names(self) -> List[str]:
        """獲取所有點位名稱"""
        return list(self.points.keys())


class Robot:
    """生產線機器人整合控制類別"""
    
    def __init__(self, config: Optional[RobotConfig] = None):
        """
        初始化機器人控制系統
        
        Args:
            config: 機器人配置，None使用預設配置
        """
        # 配置管理
        self.config = config or RobotConfig()
        
        # 設置日誌
        self.logger = self._setup_logging() if self.config.enable_logging else None
        
        # 點位管理器
        self.point_manager = RobotPointManager()
        
        # 子系統實例
        self.CCD1: Optional[CCD1VisionSystem] = None
        self.CCD3: Optional[CCD3HeadlessDetector] = None
        self.VP: Optional[ProductionVibrationPlate] = None
        self.Gripper: Optional[GripperHighLevelAPI] = None
        self.Dobot: Optional[DobotController] = None
        
        # 系統狀態
        self.initialized = False
        self.emergency_stopped = False
        self._operation_lock = threading.RLock()
        self._disconnected = False  # 防止重複斷開連接
        
        # 統計資訊
        self.stats = {
            'start_time': time.time(),
            'operation_count': 0,
            'error_count': 0,
            'emergency_stops': 0,
            'successful_operations': 0
        }
        
        # 震動進料控制狀態
        self._vibration_feed_active = False
        self._vibration_feed_error = False
        
        self._log_info("Robot類別初始化開始")
        
        # 自動初始化
        if self.config.auto_initialize:
            self.initialize_all_systems()
    
    def _setup_logging(self) -> logging.Logger:
        """設置日誌系統"""
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
        os.makedirs(log_dir, exist_ok=True)
        
        formatter = logging.Formatter(
            '%(asctime)s [%(levelname)s] Robot:%(funcName)s:%(lineno)d - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        file_handler = RotatingFileHandler(
            os.path.join(log_dir, 'robot_system.log'),
            maxBytes=10*1024*1024,
            backupCount=5,
            encoding='utf-8'
        )
        file_handler.setFormatter(formatter)
        
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        
        logger = logging.getLogger("RobotSystem")
        logger.setLevel(getattr(logging, self.config.log_level.upper()))
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)
        
        return logger
    
    def initialize_all_systems(self) -> bool:
        """初始化所有子系統"""
        self._log_info("開始初始化所有子系統")
        
        success_count = 0
        total_systems = 5
        
        try:
            # 初始化CCD1視覺系統
            if CCD1_AVAILABLE:
                try:
                    self._log_info("正在初始化CCD1視覺系統...")
                    ccd1_config = CCD1VisionConfig(
                        camera_ip=self.config.ccd1_ip,
                        auto_retry_on_error=self.config.auto_retry_on_error,
                        max_retry_count=self.config.max_retry_count
                    )
                    self.CCD1 = CCD1VisionSystem(ccd1_config)
                    if self.CCD1.initialized:
                        success_count += 1
                        self._log_info("CCD1視覺系統初始化成功")
                    else:
                        self._log_warning("CCD1視覺系統初始化失敗")
                except Exception as e:
                    self._log_error(f"CCD1初始化異常: {e}")
            else:
                self._log_warning("CCD1模組不可用")
            
            # 初始化CCD3角度檢測系統
            if CCD3_AVAILABLE:
                try:
                    self._log_info("正在初始化CCD3角度檢測系統...")
                    self.CCD3 = CCD3HeadlessDetector(camera_ip=self.config.ccd3_ip)
                    if self.CCD3.initialize_camera():
                        success_count += 1
                        self._log_info("CCD3角度檢測系統初始化成功")
                    else:
                        self._log_warning("CCD3角度檢測系統初始化失敗")
                except Exception as e:
                    self._log_error(f"CCD3初始化異常: {e}")
            else:
                self._log_warning("CCD3模組不可用")
            
            # 初始化震動盤系統
            if VP_AVAILABLE:
                try:
                    self._log_info("正在初始化震動盤系統...")
                    self.VP = ProductionVibrationPlate()
                    if self.VP.connect():
                        success_count += 1
                        self._log_info("震動盤系統初始化成功")
                    else:
                        self._log_warning("震動盤系統初始化失敗")
                except Exception as e:
                    self._log_error(f"震動盤初始化異常: {e}")
            else:
                self._log_warning("VP模組不可用")
            
            # 初始化夾爪系統
            if GRIPPER_AVAILABLE:
                try:
                    self._log_info("正在初始化夾爪系統...")
                    self.Gripper = GripperHighLevelAPI(auto_initialize=True)
                    if self.Gripper.is_connected():
                        success_count += 1
                        self._log_info("夾爪系統初始化成功")
                    else:
                        self._log_warning("夾爪系統初始化失敗")
                except Exception as e:
                    self._log_error(f"夾爪初始化異常: {e}")
            else:
                self._log_warning("Gripper模組不可用")
            
            # 初始化Dobot機械臂
            if DOBOT_AVAILABLE:
                try:
                    self._log_info("正在初始化Dobot機械臂...")
                    self.Dobot = DobotController(self.config.dobot_ip, self.point_manager, self.logger)
                    if self.Dobot.connect():
                        success_count += 1
                        self._log_info("Dobot機械臂初始化成功")
                    else:
                        self._log_warning("Dobot機械臂初始化失敗")
                except Exception as e:
                    self._log_error(f"Dobot初始化異常: {e}")
            else:
                self._log_warning("Dobot API模組不可用")
            
            # 系統安全設置
            if self.config.enable_collision_detection and self.Dobot and self.Dobot.is_connected():
                try:
                    self.Dobot.設定碰撞檢測等級(self.config.collision_level)
                    self._log_info(f"碰撞檢測等級設置為: {self.config.collision_level}")
                except Exception as e:
                    self._log_error(f"設置碰撞檢測失敗: {e}")
            
            self.initialized = (success_count >= 3)  # 至少3個系統成功才算初始化成功
            
            self._log_info(f"系統初始化完成: {success_count}/{total_systems} 個子系統成功")
            return self.initialized
            
        except Exception as e:
            self._log_error(f"系統初始化失敗: {e}")
            return False
    import os
import sys
import time
import threading
import json
import logging
from typing import Dict, Any, Optional, Tuple, List, Union
from dataclasses import dataclass
from datetime import datetime
from logging.handlers import RotatingFileHandler

# 設定路徑
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)

# 添加各模組路徑
sys.path.append(os.path.join(project_root, 'API'))
sys.path.append(os.path.join(project_root, 'CCD1'))
sys.path.append(os.path.join(project_root, 'CCD3'))
sys.path.append(os.path.join(project_root, 'VP'))
sys.path.append(os.path.join(project_root, 'Gripper'))
sys.path.append(current_dir)

# 導入各子系統
try:
    from CCD1_Headless_Vision_System import CCD1VisionSystem, CCD1VisionConfig
    CCD1_AVAILABLE = True
except ImportError as e:
    print(f"CCD1模組導入失敗: {e}")
    CCD1_AVAILABLE = False

try:
    from CCD3_Headless_Vision_System import CCD3HeadlessDetector, DetectionMethod
    CCD3_AVAILABLE = True
except ImportError as e:
    print(f"CCD3模組導入失敗: {e}")
    CCD3_AVAILABLE = False

try:
    from vibration_plate_controller import ProductionVibrationPlate
    VP_AVAILABLE = True
except ImportError as e:
    print(f"VP模組導入失敗: {e}")
    VP_AVAILABLE = False

try:
    from GripperHighLevel_DirectSerial import GripperHighLevelAPI
    GRIPPER_AVAILABLE = True
except ImportError as e:
    print(f"Gripper模組導入失敗: {e}")
    GRIPPER_AVAILABLE = False

try:
    from dobot_api import DobotApiDashboard, DobotApiMove
    DOBOT_AVAILABLE = True
except ImportError as e:
    print(f"Dobot API模組導入失敗: {e}")
    DOBOT_AVAILABLE = False


@dataclass
class RobotConfig:
    """機器人配置結構"""
    # 網路配置
    dobot_ip: str = "192.168.1.6"
    ccd1_ip: str = "192.168.1.8"
    ccd3_ip: str = "192.168.1.10"
    vp_ip: str = "192.168.1.7"
    
    # 系統配置 - 修改為手動初始化
    auto_initialize: bool = False  # 改為False
    enable_logging: bool = True
    log_level: str = "INFO"
    
    # 錯誤處理配置
    auto_retry_on_error: bool = True
    max_retry_count: int = 3
    retry_delay: float = 1.0
    
    # 安全配置
    enable_collision_detection: bool = True
    collision_level: int = 3
    emergency_stop_timeout: float = 5.0


class RobotPointManager:
    """機器人點位管理器"""
    
    def __init__(self, points_file: str = "saved_points/robot_points.json"):
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.points_file = os.path.join(self.current_dir, points_file)
        self.points = {}
        self.load_points()
    
    def load_points(self):
        """載入點位數據"""
        try:
            if os.path.exists(self.points_file):
                with open(self.points_file, 'r', encoding='utf-8') as f:
                    points_data = json.load(f)
                    
                # 轉換為以名稱為鍵的字典
                for point in points_data:
                    self.points[point['name']] = point
                    
                self._log_info(f"已載入 {len(self.points)} 個點位")
            else:
                self._log_warning(f"點位檔案不存在: {self.points_file}")
                
        except Exception as e:
            self._log_error(f"載入點位失敗: {e}")
    
    def get_point(self, name: str) -> Optional[Dict[str, Any]]:
        """根據名稱獲取點位"""
        return self.points.get(name)
    
    def get_all_points(self) -> Dict[str, Dict[str, Any]]:
        """獲取所有點位"""
        return self.points.copy()
    
    def get_point_names(self) -> List[str]:
        """獲取所有點位名稱"""
        return list(self.points.keys())
    
    def _log_info(self, message: str):
        print(f"[PointManager INFO] {message}")
    
    def _log_warning(self, message: str):
        print(f"[PointManager WARNING] {message}")
    
    def _log_error(self, message: str, exc_info: bool = False):
        """記錄錯誤"""
        self.stats['error_count'] += 1
        if self.logger:
            self.logger.error(message, exc_info=exc_info)
        else:
            print(f"[Robot ERROR] {message}")


class Robot:
    """生產線機器人整合控制類別 - 支持個別子系統控制"""
    
    def __init__(self, config: Optional[RobotConfig] = None):
        """
        初始化機器人控制系統
        
        Args:
            config: 機器人配置，None使用預設配置
        """
        # 配置管理
        self.config = config or RobotConfig()
        
        # 設置日誌
        self.logger = self._setup_logging() if self.config.enable_logging else None
        
        # 點位管理器
        self.point_manager = RobotPointManager()
        
        # 子系統實例
        self.CCD1: Optional[CCD1VisionSystem] = None
        self.CCD3: Optional[CCD3HeadlessDetector] = None
        self.VP: Optional[ProductionVibrationPlate] = None
        self.Gripper: Optional[GripperHighLevelAPI] = None
        self.Dobot: Optional[DobotController] = None
        
        # 子系統初始化狀態
        self.subsystem_status = {
            'CCD1': False,
            'CCD3': False,
            'VP': False,
            'Gripper': False,
            'Dobot': False
        }
        
        # 系統狀態
        self.initialized = False
        self.emergency_stopped = False
        self._operation_lock = threading.RLock()
        self._disconnected = False
        
        # 統計資訊
        self.stats = {
            'start_time': time.time(),
            'operation_count': 0,
            'error_count': 0,
            'emergency_stops': 0,
            'successful_operations': 0
        }
        
        # 震動進料控制狀態
        self._vibration_feed_active = False
        self._vibration_feed_error = False
        
        self._log_info("Robot類別初始化開始")
        
        # 自動初始化（如果啟用）
        if self.config.auto_initialize:
            self.initialize_all_systems()
    
    def _setup_logging(self) -> logging.Logger:
        """設置日誌系統"""
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
        os.makedirs(log_dir, exist_ok=True)
        
        formatter = logging.Formatter(
            '%(asctime)s [%(levelname)s] Robot:%(funcName)s:%(lineno)d - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        file_handler = RotatingFileHandler(
            os.path.join(log_dir, 'robot_system.log'),
            maxBytes=10*1024*1024,
            backupCount=5,
            encoding='utf-8'
        )
        file_handler.setFormatter(formatter)
        
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        
        logger = logging.getLogger("RobotSystem")
        logger.setLevel(getattr(logging, self.config.log_level.upper()))
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)
        
        return logger
    
    # ==================== 個別子系統初始化方法 ====================
    
    def init_ccd1(self) -> bool:
        """初始化CCD1視覺系統"""
        if not CCD1_AVAILABLE:
            self._log_error("CCD1模組不可用")
            return False
        
        if self.subsystem_status['CCD1']:
            self._log_warning("CCD1已經初始化")
            return True
        
        try:
            self._log_info("正在初始化CCD1視覺系統...")
            ccd1_config = CCD1VisionConfig(
                camera_ip=self.config.ccd1_ip,
                auto_retry_on_error=self.config.auto_retry_on_error,
                max_retry_count=self.config.max_retry_count
            )
            self.CCD1 = CCD1VisionSystem(ccd1_config)
            if self.CCD1.initialized:
                self.subsystem_status['CCD1'] = True
                self._log_info("CCD1視覺系統初始化成功")
                return True
            else:
                self._log_error("CCD1視覺系統初始化失敗")
                return False
        except Exception as e:
            self._log_error(f"CCD1初始化異常: {e}", exc_info=True)
            return False
    
    def init_ccd3(self) -> bool:
        """初始化CCD3角度檢測系統"""
        if not CCD3_AVAILABLE:
            self._log_error("CCD3模組不可用")
            return False
        
        if self.subsystem_status['CCD3']:
            self._log_warning("CCD3已經初始化")
            return True
        
        try:
            self._log_info("正在初始化CCD3角度檢測系統...")
            self.CCD3 = CCD3HeadlessDetector(camera_ip=self.config.ccd3_ip)
            if self.CCD3.initialize_camera():
                self.subsystem_status['CCD3'] = True
                self._log_info("CCD3角度檢測系統初始化成功")
                return True
            else:
                self._log_error("CCD3角度檢測系統初始化失敗")
                return False
        except Exception as e:
            self._log_error(f"CCD3初始化異常: {e}", exc_info=True)
            return False
    
    def init_vp(self) -> bool:
        """初始化震動盤系統"""
        if not VP_AVAILABLE:
            self._log_error("VP模組不可用")
            return False
        
        if self.subsystem_status['VP']:
            self._log_warning("VP已經初始化")
            return True
        
        try:
            self._log_info("正在初始化震動盤系統...")
            self.VP = ProductionVibrationPlate()
            if self.VP.connect():
                self.subsystem_status['VP'] = True
                self._log_info("震動盤系統初始化成功")
                return True
            else:
                self._log_error("震動盤系統初始化失敗")
                return False
        except Exception as e:
            self._log_error(f"VP初始化異常: {e}", exc_info=True)
            return False
    
    def init_gripper(self) -> bool:
        """初始化夾爪系統"""
        if not GRIPPER_AVAILABLE:
            self._log_error("Gripper模組不可用")
            return False
        
        if self.subsystem_status['Gripper']:
            self._log_warning("Gripper已經初始化")
            return True
        
        try:
            self._log_info("正在初始化夾爪系統...")
            self.Gripper = GripperHighLevelAPI(auto_initialize=True)
            if self.Gripper.is_connected():
                self.subsystem_status['Gripper'] = True
                self._log_info("夾爪系統初始化成功")
                return True
            else:
                self._log_error("夾爪系統初始化失敗")
                return False
        except Exception as e:
            self._log_error(f"Gripper初始化異常: {e}", exc_info=True)
            return False
    
    def init_dobot(self) -> bool:
        """初始化Dobot機械臂"""
        if not DOBOT_AVAILABLE:
            self._log_error("Dobot API模組不可用")
            return False
        
        if self.subsystem_status['Dobot']:
            self._log_warning("Dobot已經初始化")
            return True
        
        try:
            self._log_info("正在初始化Dobot機械臂...")
            self.Dobot = DobotController(self.config.dobot_ip, self.point_manager, self.logger)
            if self.Dobot.connect():
                self.subsystem_status['Dobot'] = True
                self._log_info("Dobot機械臂初始化成功")
                
                # 設置碰撞檢測
                if self.config.enable_collision_detection:
                    try:
                        self.Dobot.設定碰撞檢測等級(self.config.collision_level)
                        self._log_info(f"碰撞檢測等級設置為: {self.config.collision_level}")
                    except Exception as e:
                        self._log_error(f"設置碰撞檢測失敗: {e}")
                
                return True
            else:
                self._log_error("Dobot機械臂初始化失敗")
                return False
        except Exception as e:
            self._log_error(f"Dobot初始化異常: {e}", exc_info=True)
            return False
    
    # ==================== 個別子系統關閉方法 ====================
    
    def close_ccd1(self):
        """關閉CCD1視覺系統"""
        try:
            if self.CCD1:
                self.CCD1.disconnect()
                self.CCD1 = None
                self.subsystem_status['CCD1'] = False
                self._log_info("CCD1視覺系統已關閉")
        except Exception as e:
            self._log_error(f"CCD1關閉異常: {e}", exc_info=True)
    
    def close_ccd3(self):
        """關閉CCD3角度檢測系統"""
        try:
            if self.CCD3:
                self.CCD3.disconnect()
                self.CCD3 = None
                self.subsystem_status['CCD3'] = False
                self._log_info("CCD3角度檢測系統已關閉")
        except Exception as e:
            self._log_error(f"CCD3關閉異常: {e}", exc_info=True)
    
    def close_vp(self):
        """關閉震動盤系統"""
        try:
            if self.VP:
                self.VP.disconnect()
                self.VP = None
                self.subsystem_status['VP'] = False
                self._log_info("震動盤系統已關閉")
        except Exception as e:
            self._log_error(f"VP關閉異常: {e}", exc_info=True)
    
    def close_gripper(self):
        """關閉夾爪系統"""
        try:
            if self.Gripper:
                self.Gripper.disconnect()
                self.Gripper = None
                self.subsystem_status['Gripper'] = False
                self._log_info("夾爪系統已關閉")
        except Exception as e:
            self._log_error(f"Gripper關閉異常: {e}", exc_info=True)
    
    def close_dobot(self):
        """關閉Dobot機械臂"""
        try:
            if self.Dobot:
                self.Dobot.disconnect()
                self.Dobot = None
                self.subsystem_status['Dobot'] = False
                self._log_info("Dobot機械臂已關閉")
        except Exception as e:
            self._log_error(f"Dobot關閉異常: {e}", exc_info=True)
    
    # ==================== 批量初始化和關閉方法 ====================
    
    def initialize_all_systems(self) -> bool:
        """初始化所有子系統"""
        self._log_info("開始初始化所有子系統")
        
        success_count = 0
        total_systems = 5
        
        if self.init_ccd1():
            success_count += 1
        if self.init_ccd3():
            success_count += 1
        if self.init_vp():
            success_count += 1
        if self.init_gripper():
            success_count += 1
        if self.init_dobot():
            success_count += 1
        
        self.initialized = (success_count >= 3)
        self._log_info(f"系統初始化完成: {success_count}/{total_systems} 個子系統成功")
        return self.initialized
    
    def 斷開連接(self):
        """斷開所有連接並清理資源"""
        if self._disconnected:
            return
            
        try:
            self._log_info("開始斷開所有系統連接")
            self._disconnected = True
            
            self.close_ccd1()
            self.close_ccd3()
            self.close_vp()
            self.close_gripper()
            self.close_dobot()
            
            self.initialized = False
            self._log_info("所有系統連接已斷開")
            
        except Exception as e:
            self._log_error(f"斷開連接失敗: {e}")
    # ==================== CCD1視覺系統方法 ====================
    
    def CCD1_獲得可用數量(self) -> Dict[int, int]:
        """獲取CCD1檢測到的物件數量"""
        try:
            if not self.CCD1:
                self._log_error("CCD1系統未初始化")
                return {}
            
            result = self.CCD1.detect_objects()
            if result.success:
                self._log_info(f"CCD1檢測成功: {result.detections_by_class}")
                return result.detections_by_class
            else:
                self._log_error(f"CCD1檢測失敗: {result.error_message}")
                return {}
                
        except Exception as e:
            self._log_error(f"CCD1獲得數量失敗: {e}")
            return {}
    
    def CCD1_獲得可用物件座標(self, use_last_result: bool = True) -> Dict[int, List[Tuple[float, float]]]:
        """
        獲取CCD1檢測到的物件座標
        
        Args:
            use_last_result: 是否使用上次檢測結果，避免重複檢測
        """
        try:
            if not self.CCD1:
                self._log_error("CCD1系統未初始化")
                return {}
            
            # 如果使用上次結果且結果可用
            if use_last_result and self.CCD1.last_result and self.CCD1.last_result.success:
                result = self.CCD1.last_result
                self._log_info("使用上次CCD1檢測結果獲取座標")
            else:
                # 重新檢測
                result = self.CCD1.detect_objects()
            
            if result.success:
                coords = {}
                # 像素座標
                if result.coordinates_by_class:
                    coords['pixel'] = result.coordinates_by_class
                # 世界座標（如果可用）
                if result.world_coordinates_by_class:
                    coords['world'] = result.world_coordinates_by_class
                
                self._log_info(f"CCD1座標獲取成功: 像素座標{len(result.coordinates_by_class)}類")
                return coords
            else:
                self._log_error(f"CCD1座標獲取失敗: {result.error_message}")
                return {}
                
        except Exception as e:
            self._log_error(f"CCD1獲得座標失敗: {e}")
            return {}
    
    def CCD1_切換模型(self, model_id: int) -> bool:
        """切換CCD1檢測模型"""
        try:
            if not self.CCD1:
                self._log_error("CCD1系統未初始化")
                return False
            
            success = self.CCD1.switch_yolo_model(model_id)
            if success:
                self._log_info(f"CCD1模型切換成功: {model_id}")
            else:
                self._log_error(f"CCD1模型切換失敗: {model_id}")
            
            return success
            
        except Exception as e:
            self._log_error(f"CCD1切換模型失敗: {e}")
            return False
    
    # ==================== CCD3角度檢測方法 ====================
    
    def CCD3_獲得角度(self) -> Optional[float]:
        """獲取CCD3檢測到的角度"""
        try:
            if not self.CCD3:
                self._log_error("CCD3系統未初始化")
                return None
            
            result = self.CCD3.capture_and_detect()
            if result.success:
                self._log_info(f"CCD3角度檢測成功: {result.angle:.2f}°")
                return result.angle
            else:
                self._log_error(f"CCD3角度檢測失敗: {result.error_message}")
                return None
                
        except Exception as e:
            self._log_error(f"CCD3獲得角度失敗: {e}")
            return None
    
    def CCD3_獲得中心點(self) -> Optional[Tuple[int, int]]:
        """獲取CCD3檢測到的中心點座標"""
        try:
            if not self.CCD3:
                self._log_error("CCD3系統未初始化")
                return None
            
            result = self.CCD3.capture_and_detect()
            if result.success:
                self._log_info(f"CCD3中心點檢測成功: {result.center}")
                return result.center
            else:
                self._log_error(f"CCD3中心點檢測失敗: {result.error_message}")
                return None
                
        except Exception as e:
            self._log_error(f"CCD3獲得中心點失敗: {e}")
            return None
    
    def CCD3_設定檢測方法(self, method: DetectionMethod) -> bool:
        """設定CCD3檢測方法"""
        try:
            if not self.CCD3:
                self._log_error("CCD3系統未初始化")
                return False
            
            success = self.CCD3.set_detection_params(detection_method=method)
            if success:
                self._log_info(f"CCD3檢測方法設定成功: {method}")
            else:
                self._log_error(f"CCD3檢測方法設定失敗: {method}")
            
            return success
            
        except Exception as e:
            self._log_error(f"CCD3設定檢測方法失敗: {e}")
            return False
    
    # ==================== 震動盤控制方法 ====================
    
    def VP_開始震動(self, mode: str, intensity: int, frequency: int = 100, duration: float = 0) -> bool:
        """開始震動盤震動"""
        try:
            if not self.VP:
                self._log_error("VP系統未初始化")
                return False
            
            success = self.VP.vibrate(mode, intensity, frequency, duration)
            if success:
                self._log_info(f"震動盤啟動成功: {mode}, 強度{intensity}, 頻率{frequency}Hz")
            else:
                self._log_error(f"震動盤啟動失敗")
            
            return success
            
        except Exception as e:
            self._log_error(f"VP開始震動失敗: {e}")
            return False
    
    def VP_停止震動(self) -> bool:
        """停止震動盤震動"""
        try:
            if not self.VP:
                self._log_error("VP系統未初始化")
                return False
            
            success = self.VP.stop()
            if success:
                self._log_info("震動盤停止成功")
            else:
                self._log_error("震動盤停止失敗")
            
            return success
            
        except Exception as e:
            self._log_error(f"VP停止震動失敗: {e}")
            return False
    
    def VP_設定背光(self, enabled: bool, brightness: int = 50) -> bool:
        """設定震動盤背光"""
        try:
            if not self.VP:
                self._log_error("VP系統未初始化")
                return False
            
            success = True
            if brightness != 50:  # 只有亮度不是預設值才設定
                success = self.VP.set_backlight_brightness(brightness)
            
            if success:
                success = self.VP.set_backlight(enabled)
            
            if success:
                self._log_info(f"震動盤背光設定成功: {'開啟' if enabled else '關閉'}, 亮度{brightness}")
            else:
                self._log_error("震動盤背光設定失敗")
            
            return success
            
        except Exception as e:
            self._log_error(f"VP設定背光失敗: {e}")
            return False
    
    # ==================== 夾爪控制方法 ====================
    
    def Gripper_快速關閉(self) -> bool:
        """夾爪快速關閉"""
        try:
            if not self.Gripper:
                self._log_error("Gripper系統未初始化")
                return False
            
            success = self.Gripper.quick_close()
            if success:
                self._log_info("夾爪快速關閉成功")
            else:
                self._log_error("夾爪快速關閉失敗")
            
            return success
            
        except Exception as e:
            self._log_error(f"Gripper快速關閉失敗: {e}")
            return False
    
    def Gripper_快速開啟(self) -> bool:
        """夾爪快速開啟"""
        try:
            if not self.Gripper:
                self._log_error("Gripper系統未初始化")
                return False
            
            success = self.Gripper.quick_open()
            if success:
                self._log_info("夾爪快速開啟成功")
            else:
                self._log_error("夾爪快速開啟失敗")
            
            return success
            
        except Exception as e:
            self._log_error(f"Gripper快速開啟失敗: {e}")
            return False
    
    def Gripper_智能夾取(self, target_position: int = 420) -> bool:
        """夾爪智能夾取"""
        try:
            if not self.Gripper:
                self._log_error("Gripper系統未初始化")
                return False
            
            success = self.Gripper.smart_grip(target_position)
            if success:
                self._log_info(f"夾爪智能夾取成功: {target_position}")
            else:
                self._log_error(f"夾爪智能夾取失敗: {target_position}")
            
            return success
            
        except Exception as e:
            self._log_error(f"Gripper智能夾取失敗: {e}")
            return False
    
    def Gripper_智能釋放(self, release_position: int = 470) -> bool:
        """夾爪智能釋放"""
        try:
            if not self.Gripper:
                self._log_error("Gripper系統未初始化")
                return False
            
            success = self.Gripper.smart_release(release_position)
            if success:
                self._log_info(f"夾爪智能釋放成功: {release_position}")
            else:
                self._log_error(f"夾爪智能釋放失敗: {release_position}")
            
            return success
            
        except Exception as e:
            self._log_error(f"Gripper智能釋放失敗: {e}")
            return False
    
    # ==================== 震動進料控制方法 ====================
    
    def 震動進料(self, 
                feed_duration: float = 2.0,
                pulse_delay_percent: int = 30, 
                pulse_high_time: float = 0.3,
                pulse_low_time: float = 0.3,
                pulse_count: int = 1) -> bool:
        """
        DO4+DO1震動進料控制
        
        Args:
            feed_duration: DO4持續時間 (秒)
            pulse_delay_percent: DO1延遲百分比 (0-100)
            pulse_high_time: DO1 HIGH持續時間 (秒)
            pulse_low_time: DO1 LOW持續時間 (秒)
            pulse_count: DO1脈衝次數
            
        Returns:
            bool: 執行是否成功
        """
        try:
            self._log_info(f"開始震動進料控制")
            
            if not self.Dobot or not self.Dobot.is_connected():
                self._log_error("機械臂未連接，無法執行震動進料")
                return False
            
            # 計算實際延遲時間
            calculated_delay = (feed_duration * pulse_delay_percent) / 100.0
            
            self._log_info(f"震動進料參數:")
            self._log_info(f"  DO4持續時間: {feed_duration}秒")
            self._log_info(f"  DO1延遲百分比: {pulse_delay_percent}%")
            self._log_info(f"  DO1實際延遲: {calculated_delay:.2f}秒")
            self._log_info(f"  DO1脈衝次數: {pulse_count}次")
            self._log_info(f"  DO1脈衝週期: HIGH={pulse_high_time}s, LOW={pulse_low_time}s")
            
            # 設置完成標記
            self._vibration_feed_active = True
            self._vibration_feed_error = False
            
            # 啟動DO4 (投料使能)
            self._log_info("啟動DO4投料使能")
            result = self.Dobot.dashboard.DOExecute(4, 1)
            self._log_info(f"DO4啟動結果: {result}")
            
            # 創建DO4持續時間控制執行緒
            do4_thread = threading.Thread(
                target=self._do4_duration_control,
                args=(feed_duration,),
                daemon=True
            )
            do4_thread.start()
            
            # 創建DO1延遲脈衝執行緒
            do1_thread = threading.Thread(
                target=self._do1_delayed_pulse_control,
                args=(calculated_delay, pulse_high_time, pulse_low_time, pulse_count),
                daemon=True
            )
            do1_thread.start()
            
            # 等待執行完成
            total_duration = max(feed_duration, calculated_delay + (pulse_high_time + pulse_low_time) * pulse_count)
            wait_time = total_duration + 0.5  # 額外0.5秒緩衝
            
            self._log_info(f"等待震動進料完成 (預計{wait_time:.1f}秒)")
            
            start_time = time.time()
            while time.time() - start_time < wait_time:
                if not self._vibration_feed_active:
                    break
                if self._vibration_feed_error:
                    self._log_error("震動進料過程中發生錯誤")
                    return False
                time.sleep(0.1)
            
            # 等待執行緒結束
            do4_thread.join(timeout=2.0)
            do1_thread.join(timeout=2.0)
            
            # 確保所有輸出關閉
            self._log_info("確保所有DO輸出關閉")
            self.Dobot.dashboard.DOExecute(4, 0)  # 關閉DO4
            self.Dobot.dashboard.DOExecute(1, 0)  # 關閉DO1
            
            self._vibration_feed_active = False
            
            elapsed_time = time.time() - start_time
            self._log_info(f"震動進料完成，實際耗時: {elapsed_time:.2f}秒")
            
            return True
            
        except Exception as e:
            self._log_error(f"震動進料執行失敗: {e}")
            self._vibration_feed_active = False
            self._vibration_feed_error = True
            
            # 緊急關閉所有DO
            try:
                if self.Dobot and self.Dobot.is_connected():
                    self.Dobot.dashboard.DOExecute(4, 0)
                    self.Dobot.dashboard.DOExecute(1, 0)
            except:
                pass
            
            return False
    
    def _do4_duration_control(self, feed_duration: float):
        """DO4持續時間控制執行緒"""
        try:
            self._log_info(f"DO4將持續{feed_duration}秒")
            time.sleep(feed_duration)
            
            # 關閉DO4
            result = self.Dobot.dashboard.DOExecute(4, 0)
            self._log_info(f"DO4自動關閉結果: {result}")
            
        except Exception as e:
            self._log_error(f"DO4持續時間控制失敗: {e}")
            self._vibration_feed_error = True
    
    def _do1_delayed_pulse_control(self, delay_time: float, high_time: float, 
                                   low_time: float, pulse_count: int):
        """DO1延遲脈衝控制執行緒"""
        try:
            # 等待延遲時間
            self._log_info(f"DO1等待延遲{delay_time:.2f}秒")
            time.sleep(delay_time)
            
            # 執行脈衝
            self._log_info(f"DO1開始執行{pulse_count}次脈衝")
            for i in range(pulse_count):
                if not self._vibration_feed_active:
                    break
                
                # DO1 HIGH
                result = self.Dobot.dashboard.DOExecute(1, 1)
                self._log_info(f"DO1脈衝{i+1} HIGH: {result}")
                time.sleep(high_time)
                
                # DO1 LOW
                result = self.Dobot.dashboard.DOExecute(1, 0)
                self._log_info(f"DO1脈衝{i+1} LOW: {result}")
                time.sleep(low_time)
            
            self._log_info(f"DO1脈衝控制完成")
            
        except Exception as e:
            self._log_error(f"DO1延遲脈衝控制失敗: {e}")
            self._vibration_feed_error = True
    
    def 停止震動進料(self) -> bool:
        """緊急停止震動進料"""
        try:
            self._log_warning("緊急停止震動進料")
            self._vibration_feed_active = False
            
            if self.Dobot and self.Dobot.is_connected():
                self.Dobot.dashboard.DOExecute(4, 0)  # 關閉DO4
                self.Dobot.dashboard.DOExecute(1, 0)  # 關閉DO1
                self._log_info("震動進料已緊急停止")
                return True
            else:
                self._log_error("機械臂未連接，無法停止震動進料")
                return False
                
        except Exception as e:
            self._log_error(f"停止震動進料失敗: {e}")
            return False

    # ==================== 系統狀態和控制方法 ====================
    
    def 緊急停止(self) -> bool:
        """執行緊急停止"""
        try:
            self._log_warning("執行緊急停止")
            self.emergency_stopped = True
            self.stats['emergency_stops'] += 1
            
            success_count = 0
            
            # 停止機械臂
            if self.Dobot:
                try:
                    if self.Dobot.緊急停止():
                        success_count += 1
                except Exception as e:
                    self._log_error(f"機械臂緊急停止失敗: {e}")
            
            # 停止震動盤
            if self.VP:
                try:
                    if self.VP.emergency_stop():
                        success_count += 1
                except Exception as e:
                    self._log_error(f"震動盤緊急停止失敗: {e}")
            
            # 停止夾爪（如果有運動）
            if self.Gripper:
                try:
                    if self.Gripper.stop():
                        success_count += 1
                except Exception as e:
                    self._log_error(f"夾爪停止失敗: {e}")
            
            self._log_warning(f"緊急停止完成: {success_count} 個系統成功停止")
            return success_count > 0
            
        except Exception as e:
            self._log_error(f"緊急停止失敗: {e}")
            return False
    
    def 清除緊急停止(self) -> bool:
        """清除緊急停止狀態"""
        try:
            if not self.emergency_stopped:
                self._log_info("系統未處於緊急停止狀態")
                return True
            
            # 清除機械臂錯誤
            if self.Dobot:
                self.Dobot.清除錯誤()
            
            self.emergency_stopped = False
            self._log_info("緊急停止狀態已清除")
            return True
            
        except Exception as e:
            self._log_error(f"清除緊急停止失敗: {e}")
            return False
    def get_subsystem_status(self) -> Dict[str, bool]:
        """獲取子系統初始化狀態"""
        return self.subsystem_status.copy()
    
    def is_subsystem_initialized(self, subsystem: str) -> bool:
        """檢查特定子系統是否已初始化"""
        return self.subsystem_status.get(subsystem, False)
    
    def get_initialized_count(self) -> int:
        """獲取已初始化的子系統數量"""
        return sum(self.subsystem_status.values())
    def 獲得系統狀態(self) -> Dict[str, Any]:
        """獲取系統整體狀態"""
        try:
            status = {
                'initialized': self.initialized,
                'emergency_stopped': self.emergency_stopped,
                'subsystem_status': self.subsystem_status.copy(),  # 新增
                'initialized_count': self.get_initialized_count(),  # 新增
                'statistics': self.stats.copy(),
                'subsystems': {}
            }
            
            # CCD1狀態
            if self.CCD1:
                try:
                    status['subsystems']['CCD1'] = self.CCD1.get_status()
                except Exception as e:
                    status['subsystems']['CCD1'] = {'error': str(e)}
            
            # CCD3狀態
            if self.CCD3:
                try:
                    status['subsystems']['CCD3'] = self.CCD3.get_statistics()
                except Exception as e:
                    status['subsystems']['CCD3'] = {'error': str(e)}
            
            # VP狀態
            if self.VP:
                try:
                    status['subsystems']['VP'] = self.VP.get_status()
                except Exception as e:
                    status['subsystems']['VP'] = {'error': str(e)}
            
            # Gripper狀態
            if self.Gripper:
                try:
                    status['subsystems']['Gripper'] = self.Gripper.get_complete_status()
                except Exception as e:
                    status['subsystems']['Gripper'] = {'error': str(e)}
            
            # Dobot狀態
            if self.Dobot:
                try:
                    status['subsystems']['Dobot'] = self.Dobot.獲得狀態()
                except Exception as e:
                    status['subsystems']['Dobot'] = {'error': str(e)}
            
            return status
            
        except Exception as e:
            self._log_error(f"獲取系統狀態失敗: {e}")
            return {'error': str(e)}
    
    def 重新連接所有系統(self) -> bool:
        """重新連接所有子系統"""
        self._log_info("開始重新連接所有系統")
        return self.initialize_all_systems()
    
    def 斷開連接(self):
        """斷開所有連接並清理資源"""
        if self._disconnected:
            return
            
        try:
            self._log_info("開始斷開所有系統連接")
            self._disconnected = True
            
            # 斷開各子系統
            if self.CCD1:
                try:
                    self.CCD1.disconnect()
                except Exception as e:
                    self._log_error(f"CCD1斷開失敗: {e}")
            
            if self.CCD3:
                try:
                    self.CCD3.disconnect()
                except Exception as e:
                    self._log_error(f"CCD3斷開失敗: {e}")
            
            if self.VP:
                try:
                    self.VP.disconnect()
                except Exception as e:
                    self._log_error(f"VP斷開失敗: {e}")
            
            if self.Gripper:
                try:
                    self.Gripper.disconnect()
                except Exception as e:
                    self._log_error(f"Gripper斷開失敗: {e}")
            
            if self.Dobot:
                try:
                    self.Dobot.disconnect()
                except Exception as e:
                    self._log_error(f"Dobot斷開失敗: {e}")
            
            self.initialized = False
            self._log_info("所有系統連接已斷開")
            
        except Exception as e:
            self._log_error(f"斷開連接失敗: {e}")
    
    def _log_info(self, message: str):
        """記錄資訊"""
        if self.logger:
            self.logger.info(message)
        else:
            print(f"[Robot INFO] {message}")
    
    def _log_warning(self, message: str):
        """記錄警告"""
        if self.logger:
            self.logger.warning(message)
        else:
            print(f"[Robot WARNING] {message}")
    
    def _log_error(self, message: str):
        """記錄錯誤"""
        self.stats['error_count'] += 1
        if self.logger:
            self.logger.error(message)
        else:
            print(f"[Robot ERROR] {message}")
    
    def __del__(self):
        """析構函數"""
        try:
            if not self._disconnected:
                self.斷開連接()
        except:
            pass


# ==================== Dobot控制器包裝類 ====================

class DobotController:
    """Dobot機械臂控制器包裝類"""
    
    def __init__(self, ip: str, point_manager: RobotPointManager, logger: Optional[logging.Logger] = None):
        self.ip = ip
        self.point_manager = point_manager
        self.logger = logger
        
        # Dobot API實例
        self.dashboard: Optional[DobotApiDashboard] = None
        self.move: Optional[DobotApiMove] = None
        
        # 連接狀態
        self.connected = False
        self.enabled = False
        
        # 運動參數
        self.current_speed_j = 50
        self.current_speed_l = 50
        self.current_acc_j = 50
        self.current_acc_l = 50
    
    def connect(self) -> bool:
        """連接到Dobot機械臂"""
        try:
            # Dashboard連接 (29999端口)
            self.dashboard = DobotApiDashboard(self.ip, 29999)
            
            # Move連接 (30003端口) 
            self.move = DobotApiMove(self.ip, 30003)
            
            # 測試連接
            mode = self.dashboard.RobotMode()
            if mode:
                self.connected = True
                self._log_info(f"Dobot連接成功: {self.ip}")
                
                # 自動使能
                if self.使能機械臂():
                    self._log_info("機械臂自動使能成功")
                
                return True
            else:
                self._log_error("Dobot連接測試失敗")
                return False
                
        except Exception as e:
            self._log_error(f"Dobot連接失敗: {e}")
            return False
    
    def disconnect(self):
        """斷開Dobot連接"""
        try:
            if self.enabled:
                self.下使能機械臂()
            
            if self.dashboard:
                self.dashboard.close()
                self.dashboard = None
            
            if self.move:
                self.move.close()
                self.move = None
            
            self.connected = False
            self._log_info("Dobot連接已斷開")
            
        except Exception as e:
            self._log_error(f"Dobot斷開失敗: {e}")
    
    def is_connected(self) -> bool:
        """檢查連接狀態"""
        return self.connected
    
    # ==================== 基本控制方法 ====================
    
    def 使能機械臂(self) -> bool:
        """使能機械臂"""
        try:
            if not self.dashboard:
                return False
            
            result = self.dashboard.EnableRobot()
            if "EnableRobot" in result:
                self.enabled = True
                self._log_info("機械臂使能成功")
                return True
            else:
                self._log_error(f"機械臂使能失敗: {result}")
                return False
                
        except Exception as e:
            self._log_error(f"使能機械臂異常: {e}")
            return False
    
    def 下使能機械臂(self) -> bool:
        """下使能機械臂"""
        try:
            if not self.dashboard:
                return False
            
            result = self.dashboard.DisableRobot()
            if "DisableRobot" in result:
                self.enabled = False
                self._log_info("機械臂下使能成功")
                return True
            else:
                self._log_error(f"機械臂下使能失敗: {result}")
                return False
                
        except Exception as e:
            self._log_error(f"下使能機械臂異常: {e}")
            return False
    
    def 清除錯誤(self) -> bool:
        """清除機械臂錯誤"""
        try:
            if not self.dashboard:
                return False
            
            result = self.dashboard.ClearError()
            self._log_info("機械臂錯誤已清除")
            return True
            
        except Exception as e:
            self._log_error(f"清除錯誤異常: {e}")
            return False
    
    def 緊急停止(self) -> bool:
        """緊急停止機械臂"""
        try:
            if not self.dashboard:
                return False
            
            result = self.dashboard.EmergencyStop()
            self._log_warning("機械臂緊急停止")
            return True
            
        except Exception as e:
            self._log_error(f"緊急停止異常: {e}")
            return False
    
    def 復位機械臂(self) -> bool:
        """復位機械臂"""
        try:
            if not self.dashboard:
                return False
            
            result = self.dashboard.ResetRobot()
            self._log_info("機械臂已復位")
            return True
            
        except Exception as e:
            self._log_error(f"復位機械臂異常: {e}")
            return False
    
    # ==================== 參數設定方法 ====================
    
    def 設定關節速度(self, speed: int) -> bool:
        """設定關節速度 (1-100)"""
        try:
            if not self.dashboard:
                return False
            
            speed = max(1, min(speed, 100))
            result = self.dashboard.SpeedJ(speed)
            self.current_speed_j = speed
            self._log_info(f"關節速度設定為: {speed}")
            return True
            
        except Exception as e:
            self._log_error(f"設定關節速度異常: {e}")
            return False
    
    def 設定直線速度(self, speed: int) -> bool:
        """設定直線速度 (1-100)"""
        try:
            if not self.dashboard:
                return False
            
            speed = max(1, min(speed, 100))
            result = self.dashboard.SpeedL(speed)
            self.current_speed_l = speed
            self._log_info(f"直線速度設定為: {speed}")
            return True
            
        except Exception as e:
            self._log_error(f"設定直線速度異常: {e}")
            return False
    
    def 設定關節加速度(self, acc: int) -> bool:
        """設定關節加速度 (1-100)"""
        try:
            if not self.dashboard:
                return False
            
            acc = max(1, min(acc, 100))
            result = self.dashboard.AccJ(acc)
            self.current_acc_j = acc
            self._log_info(f"關節加速度設定為: {acc}")
            return True
            
        except Exception as e:
            self._log_error(f"設定關節加速度異常: {e}")
            return False
    
    def 設定直線加速度(self, acc: int) -> bool:
        """設定直線加速度 (1-100)"""
        try:
            if not self.dashboard:
                return False
            
            acc = max(1, min(acc, 100))
            result = self.dashboard.AccL(acc)
            self.current_acc_l = acc
            self._log_info(f"直線加速度設定為: {acc}")
            return True
            
        except Exception as e:
            self._log_error(f"設定直線加速度異常: {e}")
            return False
    
    def 設定碰撞檢測等級(self, level: int) -> bool:
        """設定碰撞檢測等級 (0-5)"""
        try:
            if not self.dashboard:
                return False
            
            level = max(0, min(level, 5))
            result = self.dashboard.SetCollisionLevel(level)
            self._log_info(f"碰撞檢測等級設定為: {level}")
            return True
            
        except Exception as e:
            self._log_error(f"設定碰撞檢測等級異常: {e}")
            return False
    
    # ==================== 運動控制方法 ====================
    
    def JointMovJ(self, point_name: str) -> bool:
        """關節運動到指定點位"""
        try:
            if not self.move:
                self._log_error("Move連接未建立")
                return False
            
            point = self.point_manager.get_point(point_name)
            if not point:
                self._log_error(f"找不到點位: {point_name}")
                return False
            
            cartesian = point['cartesian']
            result = self.move.MovJ(
                cartesian['x'], cartesian['y'], cartesian['z'], cartesian['r']
            )
            
            self._log_info(f"關節運動到點位 {point_name}: {cartesian}")
            return True
            
        except Exception as e:
            self._log_error(f"關節運動異常: {e}")
            return False
    
    def MovL(self, point_name: str) -> bool:
        """直線運動到指定點位"""
        try:
            if not self.move:
                self._log_error("Move連接未建立")
                return False
            
            point = self.point_manager.get_point(point_name)
            if not point:
                self._log_error(f"找不到點位: {point_name}")
                return False
            
            cartesian = point['cartesian']
            result = self.move.MovL(
                cartesian['x'], cartesian['y'], cartesian['z'], cartesian['r']
            )
            
            self._log_info(f"直線運動到點位 {point_name}: {cartesian}")
            return True
            
        except Exception as e:
            self._log_error(f"直線運動異常: {e}")
            return False
    
    def MovJ_座標(self, x: float, y: float, z: float, r: float) -> bool:
        """關節運動到指定座標"""
        try:
            if not self.move:
                return False
            
            result = self.move.MovJ(x, y, z, r)
            self._log_info(f"關節運動到座標: ({x}, {y}, {z}, {r})")
            return True
            
        except Exception as e:
            self._log_error(f"關節運動到座標異常: {e}")
            return False
    
    def MovL_座標(self, x: float, y: float, z: float, r: float) -> bool:
        """直線運動到指定座標"""
        try:
            if not self.move:
                return False
            
            result = self.move.MovL(x, y, z, r)
            self._log_info(f"直線運動到座標: ({x}, {y}, {z}, {r})")
            return True
            
        except Exception as e:
            self._log_error(f"直線運動到座標異常: {e}")
            return False
    
    def RelMovJ(self, offset_x: float, offset_y: float, offset_z: float, offset_r: float) -> bool:
        """相對關節運動"""
        try:
            if not self.move:
                return False
            
            result = self.move.RelMovJ(offset_x, offset_y, offset_z, offset_r)
            self._log_info(f"相對關節運動: ({offset_x}, {offset_y}, {offset_z}, {offset_r})")
            return True
            
        except Exception as e:
            self._log_error(f"相對關節運動異常: {e}")
            return False
    
    def RelMovL(self, offset_x: float, offset_y: float, offset_z: float, offset_r: float) -> bool:
        """相對直線運動"""
        try:
            if not self.move:
                return False
            
            result = self.move.RelMovL(offset_x, offset_y, offset_z, offset_r)
            self._log_info(f"相對直線運動: ({offset_x}, {offset_y}, {offset_z}, {offset_r})")
            return True
            
        except Exception as e:
            self._log_error(f"相對直線運動異常: {e}")
            return False
    
    def Sync(self) -> bool:
        """等待運動完成"""
        try:
            if not self.move:
                return False
            
            result = self.move.Sync()
            self._log_info("運動同步完成")
            return True
            
        except Exception as e:
            self._log_error(f"運動同步異常: {e}")
            return False
    
    # ==================== 狀態查詢方法 ====================
    
    def 獲得當前位置(self) -> Optional[Dict[str, float]]:
        """獲得當前位置"""
        try:
            if not self.dashboard:
                return None
            
            pose_result = self.dashboard.GetPose()
            angle_result = self.dashboard.GetAngle()
            
            return {
                'pose': pose_result,
                'angle': angle_result
            }
            
        except Exception as e:
            self._log_error(f"獲得當前位置異常: {e}")
            return None
    
    def 獲得機械臂模式(self) -> Optional[str]:
        """獲得機械臂模式"""
        try:
            if not self.dashboard:
                return None
            
            mode = self.dashboard.RobotMode()
            return mode
            
        except Exception as e:
            self._log_error(f"獲得機械臂模式異常: {e}")
            return None
    
    def 獲得錯誤ID(self) -> Optional[str]:
        """獲得錯誤ID"""
        try:
            if not self.dashboard:
                return None
            
            error_id = self.dashboard.GetErrorID()
            return error_id
            
        except Exception as e:
            self._log_error(f"獲得錯誤ID異常: {e}")
            return None
    
    def 獲得狀態(self) -> Dict[str, Any]:
        """獲得機械臂完整狀態"""
        try:
            status = {
                'connected': self.connected,
                'enabled': self.enabled,
                'current_position': self.獲得當前位置(),
                'robot_mode': self.獲得機械臂模式(),
                'error_id': self.獲得錯誤ID(),
                'parameters': {
                    'speed_j': self.current_speed_j,
                    'speed_l': self.current_speed_l,
                    'acc_j': self.current_acc_j,
                    'acc_l': self.current_acc_l
                },
                'available_points': self.point_manager.get_point_names()
            }
            
            return status
            
        except Exception as e:
            self._log_error(f"獲得狀態異常: {e}")
            return {'error': str(e)}
    
    def _log_info(self, message: str):
        """記錄資訊"""
        if self.logger:
            self.logger.info(message)
        else:
            print(f"[Dobot INFO] {message}")
    
    def _log_warning(self, message: str):
        """記錄警告"""
        if self.logger:
            self.logger.warning(message)
        else:
            print(f"[Dobot WARNING] {message}")
    
    def _log_error(self, message: str):
        """記錄錯誤"""
        if self.logger:
            self.logger.error(message)
        else:
            print(f"[Dobot ERROR] {message}")


# ==================== 使用範例 ====================

def optimized_example_usage():
    """優化的使用範例 - 解決檢測到的問題"""
    
    # 創建機器人實例 - 修正夾爪端口配置
    config = RobotConfig(
        dobot_ip="192.168.1.6",
        ccd1_ip="192.168.1.8", 
        ccd3_ip="192.168.1.10",
        vp_ip="192.168.1.7",
        auto_initialize=True
    )
    
    robot = Robot(config)
    
    try:
        if robot.initialized:
            print("機器人系統初始化成功")
            
            # 使用範例 - 優化版本
            print("\n=== CCD1視覺檢測 (優化版本) ===")
            
            # 先檢測數量
            detections = robot.CCD1_獲得可用數量()
            print(f"檢測數量: {detections}")
            
            # 使用上次檢測結果獲取座標，避免重複檢測導致模型被清理
            coordinates = robot.CCD1_獲得可用物件座標(use_last_result=True)
            print(f"物件座標類別數: {len(coordinates)}")
            
            # 如果需要重新載入模型
            if not robot.CCD1.yolo_detector.model_manager.is_model_loaded():
                print("重新載入YOLO模型...")
                robot.CCD1_切換模型(1)
            
            print("\n=== CCD3角度檢測 ===")
            angle = robot.CCD3_獲得角度()
            print(f"檢測角度: {angle}")
            
            center = robot.CCD3_獲得中心點()
            print(f"中心點: {center}")
            
            print("\n=== 震動盤控制 ===")
            robot.VP_設定背光(True, 50)
            robot.VP_開始震動("vertical", 60, 100, 2)
            time.sleep(3)
            robot.VP_停止震動()
            
            print("\n=== 夾爪控制 (跳過COM端口問題) ===")
            if robot.Gripper and robot.Gripper.is_connected():
                robot.Gripper_快速開啟()
                time.sleep(1)
                robot.Gripper_智能夾取(420)
                time.sleep(1)
                robot.Gripper_智能釋放(470)
            else:
                print("夾爪未連接，跳過夾爪測試")
            
            print("\n=== 機械臂控制 ===")
            robot.Dobot.設定關節速度(50)  # 降低速度以確保安全
            robot.Dobot.JointMovJ("standby")
            robot.Dobot.Sync()
            robot.震動進料()
            robot.Dobot.JointMovJ("VP_TOPSIDE")
            robot.Dobot.Sync()
            robot.Dobot.JointMovJ("standby")
            robot.Dobot.Sync()
            # 簡化的系統狀態檢查
            print("\n=== 系統狀態摘要 ===")
            status = robot.獲得系統狀態()
            print(f"系統初始化: {status['initialized']}")
            print(f"緊急停止狀態: {status['emergency_stopped']}")
            print(f"錯誤計數: {status['statistics']['error_count']}")
            
            # 各子系統狀態
            for name, subsystem in status['subsystems'].items():
                if 'error' not in subsystem:
                    if name == 'CCD1':
                        print(f"CCD1: 相機連接={subsystem.get('camera_connected', False)}, "
                              f"模型載入={subsystem.get('yolo_model_loaded', False)}")
                    elif name == 'CCD3':
                        print(f"CCD3: 初始化={subsystem.get('is_initialized', False)}, "
                              f"成功率={subsystem.get('success_rate', 0):.1f}%")
                    elif name == 'VP':
                        print(f"VP: 連接={subsystem.get('connected', False)}, "
                              f"震動中={subsystem.get('vibration_state', {}).get('is_vibrating', False)}")
                    elif name == 'Gripper':
                        print(f"Gripper: 連接={subsystem.get('connected', False)}")
                    elif name == 'Dobot':
                        print(f"Dobot: 連接={subsystem.get('connected', False)}, "
                              f"使能={subsystem.get('enabled', False)}")
                else:
                    print(f"{name}: 錯誤 - {subsystem['error']}")
            
        else:
            print("機器人系統初始化失敗")
            
    except KeyboardInterrupt:
        print("\n收到中斷信號，正在安全關閉系統...")
        robot.緊急停止()
        
    except Exception as e:
        print(f"執行異常: {e}")
        # 緊急停止
        try:
            robot.緊急停止()
        except:
            pass
        
    finally:
        # 清理資源
        print("\n正在安全關閉系統...")
        robot.斷開連接()
        print("程序結束")


if __name__ == "__main__":
    optimized_example_usage()
