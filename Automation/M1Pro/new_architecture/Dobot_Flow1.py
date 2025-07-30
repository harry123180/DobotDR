
"""
Dobot_Flow1.py - Flow1 VP視覺抓取流程執行器 (DR專案版 - 支援DR參數傳入)
- 升級motion_steps支援速度、加速度、運動類型、sync調用、切換手勢等參數
- 從AutoProgram模組讀取座標 (地址1350-1354)  
- 支援DR專案格式的參數傳入：speed_j, acc_j, speed_l, acc_l, tool, sync等
- 支援MovJ、MovL、JointMovJ運動類型
- 整合快速角度檢測功能
"""

import time
import os
import json
import threading
import queue
import logging
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum

# 導入Modbus TCP Client (適配pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError:
    MODBUS_AVAILABLE = False


@dataclass
class RobotPoint:
    """機械臂點位數據結構"""
    name: str
    x: float
    y: float
    z: float
    r: float
    j1: float
    j2: float
    j3: float
    j4: float


@dataclass
class FlowResult:
    """流程執行結果"""
    success: bool
    error_message: str = ""
    execution_time: float = 0.0
    steps_completed: int = 0
    total_steps: int = 0
    detected_position: Optional[Dict[str, float]] = None
    target_angle: Optional[float] = None
    command_angle: Optional[float] = None
    angle_acquisition_success: bool = False
    extra_data: Dict[str, Any] = None

    def __post_init__(self):
        if self.extra_data is None:
            self.extra_data = {}


class FlowStatus(Enum):
    """流程狀態"""
    READY = "ready"
    RUNNING = "running" 
    PAUSED = "paused"
    COMPLETED = "completed"
    ERROR = "error"


def setup_logging(module_name):
    """統一設置logging配置"""
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
    os.makedirs(log_dir, exist_ok=True)
    
    formatter = logging.Formatter(
        '%(asctime)s [%(levelname)s] %(name)s:%(funcName)s:%(lineno)d - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    file_handler = logging.handlers.RotatingFileHandler(
        os.path.join(log_dir, f'{module_name}.log'),
        maxBytes=10*1024*1024,
        backupCount=7,
        encoding='utf-8'
    )
    file_handler.setFormatter(formatter)
    
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    
    logger = logging.getLogger(module_name)
    logger.setLevel(logging.DEBUG)
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger


class AutoProgramInterface:
    """AutoProgram座標接口 - 統一交握版本"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        self._connection_retries = 0
        self._max_retries = 3
        
        # AutoProgram交握寄存器映射
        self.REGISTERS = {
            # AutoProgram系統狀態寄存器 (1300-1319)
            'SYSTEM_STATUS': 1300,           # 系統狀態 (0=停止, 1=運行, 2=Flow1觸發, 3=Flow2完成, 4=錯誤)
            'PREPARE_DONE': 1301,            # prepare_done狀態 (0=需要取料, 1=已完成取料)
            'AUTO_PROGRAM_ENABLED': 1302,    # 自動程序啟用狀態
            'AF_DR_F_STATUS': 1303,        # AutoFeeding DR_F狀態
            'FLOW2_STATUS': 1304,            # Flow2完成狀態
            
            # AutoProgram控制寄存器 (1320-1339)  
            'SYSTEM_CONTROL': 1320,          # 系統控制指令
            'AUTO_PROGRAM_CONTROL': 1321,    # 自動程序啟用控制
            
            # AutoProgram座標寄存器 (1340-1349)
            'TARGET_X_HIGH': 1341,           # 目標座標X高位
            'TARGET_X_LOW': 1342,            # 目標座標X低位
            'TARGET_Y_HIGH': 1343,           # 目標座標Y高位  
            'TARGET_Y_LOW': 1344,            # 目標座標Y低位
            
            # 原始AutoFeeding寄存器 (向下兼容)
            'AF_DR_F_AVAILABLE': 940,      # DR_F可用標誌
            'AF_COORDS_TAKEN': 945,          # 座標已讀取標誌確認
        }
        
        # 預先建立連接
        self.ensure_connection()
    
    def ensure_connection(self) -> bool:
        """確保連接已建立，包含重連邏輯"""
        if self.connected and self.modbus_client:
            try:
                # 快速連接測試
                test_result = self.modbus_client.read_holding_registers(
                    self.REGISTERS['SYSTEM_STATUS'], 1, slave=1
                )
                if not test_result.isError():
                    return True
            except:
                pass
        
        # 需要重新連接
        return self._establish_connection()
    
    def _establish_connection(self) -> bool:
        """建立連接的內部方法"""
        try:
            if self.modbus_client:
                try:
                    self.modbus_client.close()
                except:
                    pass
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=2.0
            )
            
            if self.modbus_client.connect():
                self.connected = True
                self._connection_retries = 0
                print("✓ AutoProgram連接已建立")
                return True
            else:
                self.connected = False
                self._connection_retries += 1
                print(f"✗ AutoProgram連接失敗 (嘗試 {self._connection_retries}/{self._max_retries})")
                return False
                
        except Exception as e:
            self.connected = False
            self._connection_retries += 1
            print(f"AutoProgram連接異常: {e}")
            return False
    
    def disconnect(self):
        """斷開連接"""
        if self.modbus_client and self.connected:
            try:
                self.modbus_client.close()
            except:
                pass
        self.connected = False
        self.modbus_client = None
    
    def read_register(self, register_name: str) -> Optional[int]:
        """讀取寄存器 - 包含自動重連"""
        if not self.ensure_connection() or register_name not in self.REGISTERS:
            return None
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.read_holding_registers(address, count=1, slave=1)
            
            if not result.isError():
                return result.registers[0]
            else:
                return None
                
        except Exception:
            # 連接可能中斷，標記為未連接
            self.connected = False
            return None
    
    def write_register(self, register_name: str, value: int) -> bool:
        """寫入寄存器 - 包含自動重連"""
        if not self.ensure_connection() or register_name not in self.REGISTERS:
            return False
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.write_register(address, value, slave=1)
            return not result.isError()
        except Exception:
            self.connected = False
            return False
    
    def check_autoprogram_ready_and_coordinates(self) -> bool:
        """檢查AutoProgram狀態並確認座標可用 - 修正版接受狀態3"""
        try:
            # 1. 檢查AutoProgram系統狀態(1300) 
            # 接受 1(運行中), 2(Flow1觸發狀態), 3(Flow1等待狀態)
            system_status = self.read_register('SYSTEM_STATUS')
            if system_status not in [1, 2, 3]:
                return False
            
            # 2. 檢查prepare_done狀態(1301) = 0(需要取料)
            prepare_done = self.read_register('PREPARE_DONE') 
            if prepare_done != 0:
                return False
                
            # 3. 檢查座標是否已準備在AutoProgram中
            # 讀取座標寄存器檢查是否有有效座標
            x_high = self.read_register('TARGET_X_HIGH') or 0
            x_low = self.read_register('TARGET_X_LOW') or 0
            y_high = self.read_register('TARGET_Y_HIGH') or 0
            y_low = self.read_register('TARGET_Y_LOW') or 0
            
            # 檢查座標是否非零(有效)
            if x_high == 0 and x_low == 0 and y_high == 0 and y_low == 0:
                return False
                
            return True
            
        except Exception as e:
            print(f"檢查AutoProgram狀態異常: {e}")
            return False
    
    def read_target_coordinates_from_autoprogram(self) -> Optional[Dict[str, float]]:
        """從AutoProgram讀取目標座標"""
        try:
            # 批量讀取座標寄存器 (1340-1343)
            try:
                result = self.modbus_client.read_holding_registers(
                    self.REGISTERS['TARGET_X_HIGH'], 4, slave=1
                )
                if result.isError():
                    return None
                
                registers = result.registers
                x_high, x_low, y_high, y_low = registers[0], registers[1], registers[2], registers[3]
                
            except Exception:
                # 批量讀取失敗，回退到單個讀取
                x_high = self.read_register('TARGET_X_HIGH') or 0
                x_low = self.read_register('TARGET_X_LOW') or 0
                y_high = self.read_register('TARGET_Y_HIGH') or 0
                y_low = self.read_register('TARGET_Y_LOW') or 0
            
            # 32位合併並轉換精度
            world_x_int = (x_high << 16) | x_low
            world_y_int = (y_high << 16) | y_low
            
            # 處理負數 (補碼轉換)
            if world_x_int >= 2**31:
                world_x_int -= 2**32
            if world_y_int >= 2**31:
                world_y_int -= 2**32
            
            # 恢復精度 (÷100)
            world_x = world_x_int / 100.0
            world_y = world_y_int / 100.0
            
            return {
                'x': world_x,
                'y': world_y,
                'source': 'autoprogram_interface'
            }
            
        except Exception as e:
            print(f"讀取AutoProgram目標座標異常: {e}")
            return None
    
    def confirm_coordinate_read(self) -> bool:
        """確認座標已讀取"""
        return self.write_register('AF_COORDS_TAKEN', 1)
    
    def get_autoprogram_status_info(self) -> Dict[str, Any]:
        """獲取AutoProgram狀態資訊"""
        try:
            return {
                'system_status': self.read_register('SYSTEM_STATUS'),
                'prepare_done': self.read_register('PREPARE_DONE'),
                'auto_program_enabled': self.read_register('AUTO_PROGRAM_ENABLED'),
                'af_dr_f_status': self.read_register('AF_DR_F_STATUS'),
                'flow2_status': self.read_register('FLOW2_STATUS'),
                'af_dr_f_available': self.read_register('AF_DR_F_AVAILABLE'),
                'connected': self.connected
            }
        except Exception as e:
            print(f"獲取AutoProgram狀態資訊異常: {e}")
            return {'connected': False, 'error': str(e)}


class PointsManager:
    """點位管理器 - 支援cartesian和pose格式"""
    
    def __init__(self, points_file: str = "saved_points/robot_points.json"):
        self.logger = setup_logging("PointsManager")
        if not os.path.isabs(points_file):
            current_dir = os.path.dirname(os.path.abspath(__file__))
            self.points_file = os.path.join(current_dir, points_file)
        else:
            self.points_file = points_file
        self.points: Dict[str, RobotPoint] = {}
        
    def load_points(self) -> bool:
        """載入點位數據 - 支援cartesian和pose格式"""
        try:
            self.logger.info(f"嘗試載入點位檔案: {self.points_file}")
            
            if not os.path.exists(self.points_file):
                self.logger.error(f"點位檔案不存在: {self.points_file}")
                return False
                
            with open(self.points_file, "r", encoding="utf-8") as f:
                points_list = json.load(f)
            
            self.points.clear()
            for point_data in points_list:
                try:
                    # 支援兩種格式：pose 或 cartesian
                    if "pose" in point_data:
                        pose_data = point_data["pose"]
                    elif "cartesian" in point_data:
                        pose_data = point_data["cartesian"]
                    else:
                        self.logger.warning(f"點位 {point_data.get('name', 'unknown')} 缺少座標數據")
                        continue
                    
                    if "joint" not in point_data:
                        self.logger.warning(f"點位 {point_data.get('name', 'unknown')} 缺少關節數據")
                        continue
                    
                    joint_data = point_data["joint"]
                    
                    point = RobotPoint(
                        name=point_data["name"],
                        x=float(pose_data["x"]),
                        y=float(pose_data["y"]),
                        z=float(pose_data["z"]),
                        r=float(pose_data["r"]),
                        j1=float(joint_data["j1"]),
                        j2=float(joint_data["j2"]),
                        j3=float(joint_data["j3"]),
                        j4=float(joint_data["j4"])
                    )
                    
                    # 處理點位名稱的拼寫錯誤
                    point_name = point.name
                    if point_name == "stanby":
                        point_name = "standby"
                        self.logger.info("自動修正點位名稱: stanby -> standby")
                    
                    self.points[point_name] = point
                    
                except Exception as e:
                    self.logger.error(f"處理點位 {point_data.get('name', 'unknown')} 時發生錯誤: {e}")
                    continue
                
            self.logger.info(f"載入點位數據成功，共{len(self.points)}個點位: {list(self.points.keys())}")
            return True
            
        except Exception as e:
            self.logger.error(f"載入點位數據失敗: {e}", exc_info=True)
            return False
    
    def get_point(self, name: str) -> Optional[RobotPoint]:
        """獲取指定點位"""
        return self.points.get(name)
    
    def list_points(self) -> List[str]:
        """列出所有點位名稱"""
        return list(self.points.keys())
    
    def has_point(self, name: str) -> bool:
        """檢查是否存在指定點位"""
        return name in self.points


class DrFlow1VisionPickExecutor:
    """Flow1: VP視覺抓取流程執行器 - DR專案整合DR參數傳入版"""
    
    def __init__(self, sync_enable: bool = False):
        self.logger = setup_logging("DrFlow1VisionPickExecutor")
        
        # 核心組件 (通過initialize方法設置)
        self.robot = None
        self.motion_state_machine = None
        self.external_modules = {}
        
        # 流程配置
        self.flow_id = 1
        self.flow_name = "VP視覺抓取流程(支援DR參數)"
        self.status = FlowStatus.READY
        self.current_step = 0
        self.start_time = 0.0
        self.last_error = ""
        
        # 同步控制參數
        self.sync_enable = sync_enable
        
        # 流程參數
        self.PICKUP_HEIGHT = 147.52
        
        # 角度檢測參數
        self.target_angle = None
        self.command_angle = None
        self.angle_acquisition_success = False
        self.ANGLE_OFFSET = 40.0
        
        # 優化參數
        self.angle_detection_timeout = 3.0
        self.use_fast_angle_detection = True
        
        # 初始化點位管理器
        self.points_manager = PointsManager()
        self.points_loaded = False
        
        # 初始化AutoProgram接口
        self.autoprogram_interface = AutoProgramInterface()
        
        # Flow1完成狀態管理
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.FLOW1_COMPLETE_REGISTER = 1204
        
        # Flow1需要的點位名稱
        self.REQUIRED_POINTS = [
            "standby",
            "VP_TOPSIDE",
            "Rotate_top",
            "Rotate_down",
            "Rotate_V2",
            "put_asm_top"
        ]
        
        # 建構流程步驟
        self.motion_steps = []
        self.total_steps = 0
        
        # 嘗試載入點位檔案
        self._load_and_validate_points()
        
        # 只有點位載入成功才建構流程步驟
        if self.points_loaded:
            self.build_flow_steps()
        
        self.logger.info(f"DrFlow1VisionPickExecutor初始化完成 (支援DR參數版)")
        self.logger.info(f"  sync控制: {'啟用' if sync_enable else '停用'}")
        self.logger.info(f"  座標來源: AutoProgram模組 (地址1350-1354)")
        self.logger.info(f"  角度檢測: 超時{self.angle_detection_timeout}秒")
        
    def initialize(self, robot, motion_state_machine, external_modules):
        """初始化Flow執行器"""
        self.robot = robot
        self.motion_state_machine = motion_state_machine
        self.external_modules = external_modules
        
        self.logger.info("Flow1執行器初始化完成")
        self.logger.info(f"  可用模組: Gripper={self.external_modules.get('gripper') is not None}, "
                        f"Angle={self.external_modules.get('angle') is not None}")
        
    def _load_and_validate_points(self):
        """載入並驗證點位檔案"""
        self.logger.info("Flow1正在載入外部點位檔案...")
        
        if not self.points_manager.load_points():
            self.logger.error("無法載入點位檔案，Flow1無法執行")
            self.points_loaded = False
            return
        
        missing_points = []
        for point_name in self.REQUIRED_POINTS:
            if not self.points_manager.has_point(point_name):
                missing_points.append(point_name)
        
        if missing_points:
            self.logger.error(f"缺少必要點位: {missing_points}")
            self.logger.debug(f"可用點位: {self.points_manager.list_points()}")
            self.points_loaded = False
            return
        
        self.logger.info("所有必要點位載入成功")
        self.points_loaded = True
        
    def build_flow_steps(self):
        """建構Flow1步驟 - 支援DR參數版"""
        if not self.points_loaded:
            self.logger.warning("點位未載入，無法建構流程步驟")
            self.motion_steps = []
            self.total_steps = 0
            return
            
        # 定義新的流程步驟 - 加入DR風格參數
        self.motion_steps = [
            # 1. AutoProgram座標讀取
            {'type': 'autoprogram_coordinates_read', 'params': {}},
            {'type': 'arm_orientation_change', 'params': {'orientation': 1}},  # 切換到左手手勢
            # 2. 初始準備 - 加入速度控制
            {'type': 'move_to_point', 'params': {
                'point_name': 'standby', 
                'move_type': 'J', 
                'speed_j': 100, 
                'acc_j': 100, 
                'sync': False
            }},
            {'type': 'gripper_close', 'params': {}},
            
            # 3. 移動到VP上方檢測位置 - 高速運動
            {'type': 'move_to_point', 'params': {
                'point_name': 'VP_TOPSIDE', 
                'move_type': 'J', 
                'speed_j': 100, 
                'acc_j': 100, 
                'sync': False
            }},
            
            # 4. 移動到檢測物件位置 - 精準控制
            {'type': 'move_to_detected_position_high', 'params': {
                'speed_l': 100, 
                'acc_l': 100, 
                'high': 180,
                'sync': False
            }},
            {'type': 'move_to_detected_position_low', 'params': {
                'speed_l': 100, 
                'acc_l': 100, 
                'sync': True
            }},
            {'type': 'gripper_smart_release', 'params': {'position': 360}},
            
            {'type': 'move_to_point', 'params': {
                'point_name': 'VP_TOPSIDE', 
                'move_type': 'L', 
                'speed_l': 100, 
                'acc_l': 100, 
                'sync': False
            }},
            {'type': 'move_to_point', 'params': {
                'point_name': 'standby', 
                'move_type': 'J', 
                'speed_j': 100, 
                'acc_j': 100, 
                'sync': False
            }},
            
            # 5. 翻轉站序列 - 標準速度
            {'type': 'move_to_point', 'params': {
                'point_name': 'Rotate_top', 
                'move_type': 'J', 
                'speed_j': 100, 
                'acc_j': 100, 
                'sync': False
            }},
            {'type': 'move_to_point', 'params': {
                'point_name': 'Rotate_down', 
                'move_type': 'J', 
                'speed_j': 100, 
                'acc_j': 100, 
                'sync': True
            }},
            {'type': 'gripper_close', 'params': {}},
            {'type': 'move_to_point', 'params': {
                'point_name': 'Rotate_top', 
                'move_type': 'J', 
                'speed_j': 100, 
                'acc_j': 100, 
                'sync': False
            }},
            {'type': 'move_to_point', 'params': {
                'point_name': 'standby', 
                'move_type': 'J', 
                'speed_j':100, 
                'acc_j': 100, 
                'sync': True
            }},
            
            # 6. 快速角度檢測
            {'type': 'angle_detection_fast', 'params': {}},
            
            # 7. 移動到組裝位置 - 精準控制
            {'type': 'move_to_point', 'params': {
                'point_name': 'Rotate_top', 
                'move_type': 'J', 
                'speed_j': 100, 
                'acc_j': 100, 
                'sync': False
            }},
            {'type': 'move_to_point', 'params': {
                'point_name': 'Rotate_down', 
                'move_type': 'J', 
                'speed_j': 100, 
                'acc_j': 100, 
                'sync': True
            }},
            {'type': 'gripper_smart_release', 'params': {'position': 360}},
            {'type': 'move_to_point', 'params': {
                'point_name': 'Rotate_top', 
                'move_type': 'J', 
                'speed_j': 100, 
                'acc_j': 100, 
                'sync': False
            }},
            
            # 8. 帶J4角度控制的組裝位置 - 超精準控制 + 切換手勢範例
            
            {'type': 'move_to_point_with_j4', 'params': {
                'point_name': 'put_asm_top', 
                'move_type': 'J', 
                'speed_j': 100, 
                'acc_j': 100, 
                'sync': False
            }},
            
         
        ]
        
        self.total_steps = len(self.motion_steps)
        self.logger.info(f"Flow1支援DR參數流程步驟建構完成，共{self.total_steps}步")
        self.logger.info("新增功能: 支援速度/加速度控制 + 工具座標系切換")
    
    def execute(self) -> FlowResult:
        """執行Flow1主邏輯 - 支援DR參數版"""
        if not self.points_loaded:
            return FlowResult(
                success=False,
                error_message="點位檔案載入失敗，無法執行Flow1",
                execution_time=0.0,
                steps_completed=0,
                total_steps=0
            )
        
        self.status = FlowStatus.RUNNING
        self.start_time = time.time()
        self.current_step = 0
        
        # 重置角度檢測參數
        self.target_angle = None
        self.command_angle = None
        self.angle_acquisition_success = False
        
        # 檢查初始化
        if not self.robot or not self.robot.is_connected:
            return FlowResult(
                success=False,
                error_message="機械臂未連接或未初始化",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        # 檢查AutoProgram連接
        if not self.autoprogram_interface.ensure_connection():
            return FlowResult(
                success=False,
                error_message="AutoProgram連接失敗",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        detected_position = None
        
        try:
            for step in self.motion_steps:
                if self.status == FlowStatus.PAUSED:
                    time.sleep(0.1)
                    continue
                    
                if self.status == FlowStatus.ERROR:
                    break
                
                self.logger.info(f"Flow1 步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
                # 更新進度到motion_state_machine
                if self.motion_state_machine:
                    progress = int((self.current_step / self.total_steps) * 100)
                    self.motion_state_machine.set_progress(progress)
                
                # 執行步驟
                success = False
                
                if step['type'] == 'move_to_point':
                    success = self._execute_move_to_point_with_parameters(step['params'])
                elif step['type'] == 'move_to_point_with_j4':
                    success = self._execute_move_to_point_with_j4(step['params'])
                elif step['type'] == 'gripper_close':
                    success = self._execute_gripper_close()
                elif step['type'] == 'gripper_smart_release':
                    success = self._execute_gripper_smart_release(step['params'])
                elif step['type'] == 'autoprogram_coordinates_read':
                    detected_position = self._execute_read_autoprogram_coordinates()
                    success = detected_position is not None
                elif step['type'] == 'move_to_detected_position_high':
                    success = self._execute_move_to_detected_position_with_parameters(detected_position, step['params'])
                elif step['type'] == 'move_to_detected_position_low':
                    success = self._execute_move_to_detected_position_with_parameters(detected_position, step['params'])
                elif step['type'] == 'arm_orientation_change':
                    success = self._execute_arm_orientation_change(step['params'])
                elif step['type'] == 'angle_detection_fast':
                    success = self._execute_angle_detection_fast()
                else:
                    self.logger.warning(f"未知步驟類型: {step['type']}")
                    success = False
                
                if not success:
                    self.status = FlowStatus.ERROR
                    return FlowResult(
                        success=False,
                        error_message=f"步驟 {step['type']} 執行失敗",
                        execution_time=time.time() - self.start_time,
                        steps_completed=self.current_step,
                        total_steps=self.total_steps,
                        target_angle=self.target_angle,
                        command_angle=self.command_angle,
                        angle_acquisition_success=self.angle_acquisition_success
                    )
                
                self.current_step += 1
            
            # 流程成功完成
            self.status = FlowStatus.COMPLETED
            execution_time = time.time() - self.start_time
            
            # 設置Flow1完成狀態
            self._safe_set_flow1_complete_status(True)
            
            # 最終進度設置
            if self.motion_state_machine:
                self.motion_state_machine.set_progress(100)
            
            self.logger.info(f"Flow1執行完成！總耗時: {execution_time:.2f}秒")
            if self.angle_acquisition_success:
                self.logger.info(f"角度控制: 目標角度={self.target_angle:.2f}°, 指令角度={self.command_angle:.2f}°")
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps,
                detected_position=detected_position,
                target_angle=self.target_angle,
                command_angle=self.command_angle,
                angle_acquisition_success=self.angle_acquisition_success
            )
            
        except Exception as e:
            self.status = FlowStatus.ERROR
            self.logger.error(f"Flow1執行異常: {e}", exc_info=True)
            return FlowResult(
                success=False,
                error_message=f"Flow1執行異常: {str(e)}",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps,
                target_angle=self.target_angle,
                command_angle=self.command_angle,
                angle_acquisition_success=self.angle_acquisition_success
            )
    def _execute_arm_orientation_change(self, params: Dict[str, Any]) -> bool:
        """執行機械臂手勢切換 (SetArmOrientation)"""
        try:
            orientation = params.get('orientation', 0)
            
            # 手勢定義
            orientation_names = {
                0: "右手手勢 (Right)",
                1: "左手手勢 (Left)" 
            }
            
            orientation_name = orientation_names.get(orientation, f"未知手勢({orientation})")
            self.logger.info(f"切換機械臂手勢到: {orientation_name}")
            
            # 方法1: 直接調用機械臂API的SetArmOrientation()方法
            if hasattr(self.robot, 'dashboard_api') and self.robot.dashboard_api:
                try:
                    # 檢查是否有SetArmOrientation方法
                    if hasattr(self.robot.dashboard_api, 'SetArmOrientation'):
                        result = self.robot.dashboard_api.SetArmOrientation(orientation)
                        self.logger.info(f"機械臂手勢切換成功: SetArmOrientation({orientation})")
                        self.logger.debug(f"API回應: {result}")
                        return True
                    else:
                        self.logger.warning("機械臂API不支援SetArmOrientation方法")
                        return True  # 降級處理，不影響流程
                except Exception as e:
                    self.logger.error(f"機械臂手勢切換失敗: {e}")
                    return False
            
            # 方法2: 如果機械臂有set_arm_orientation方法
            elif hasattr(self.robot, 'set_arm_orientation'):
                try:
                    success = self.robot.set_arm_orientation(orientation)
                    if success:
                        self.logger.info(f"機械臂手勢切換成功: set_arm_orientation({orientation})")
                    else:
                        self.logger.error(f"機械臂手勢切換失敗: set_arm_orientation({orientation})")
                    return success
                except Exception as e:
                    self.logger.error(f"機械臂手勢切換異常: {e}")
                    return False
            
            # 方法3: 如果有move_api可以發送SetArmOrientation指令
            elif hasattr(self.robot, 'move_api') and self.robot.move_api:
                try:
                    # 直接發送SetArmOrientation指令字符串
                    command = f"SetArmOrientation({orientation})"
                    result = self.robot.move_api.sendRecvMsg(command)
                    self.logger.info(f"機械臂手勢切換指令發送: {command}")
                    self.logger.debug(f"API回應: {result}")
                    return True
                except Exception as e:
                    self.logger.error(f"機械臂手勢切換指令發送失敗: {e}")
                    return False
            
            else:
                self.logger.warning("機械臂API不支援手勢切換，跳過此步驟")
                return True  # 降級處理，不影響整個流程
                
        except Exception as e:
            self.logger.error(f"機械臂手勢切換異常: {e}", exc_info=True)
            return False#!/usr/bin/env python3
    def _execute_move_to_point_with_parameters(self, params: Dict[str, Any]) -> bool:
        try:
            point_name = params['point_name']
            move_type = params['move_type']
            
            # 獲取點位
            point = self.points_manager.get_point(point_name)
            if not point:
                self.logger.error(f"點位管理器中找不到點位: {point_name}")
                return False
            
            # 提取運動參數
            speed_j = params.get('speed_j',100)
            acc_j = params.get('acc_j',100)
            speed_l = params.get('speed_l', 100)
            acc_l = params.get('acc_l', 100)
            tool = params.get('tool', 0)
            sync_enabled = params.get('sync', self.sync_enable)
            
            self.logger.info(f"移動到點位 {point_name} ({move_type})")
            #self.logger.debug(f"  關節角度: (j1:{point.j1:.1f}, j2:{point.j2:.1f}, j3:{point.j3:.1f}, j4:{point.j4:.1f})")
            #self.logger.debug(f"  笛卡爾座標: ({point.x:.2f}, {point.y:.2f}, {point.z:.2f}, {point.r:.2f})")
            self.logger.debug(f"  運動參數: speed_j={speed_j}, acc_j={acc_j}, speed_l={speed_l}, acc_l={acc_l}, tool={tool}")
            
            # 根據運動類型設置速度參數
            success = False
            if move_type in ['J', 'JointMovJ']:
                # 設置關節運動參數
                if hasattr(self.robot, 'set_speed_j'):
                    self.robot.set_speed_j(speed_j)
                if hasattr(self.robot, 'set_acc_j'):
                    self.robot.set_acc_j(acc_j)
                if hasattr(self.robot, 'set_tool'):
                    self.robot.set_tool(tool)
                
                # 執行關節運動
                success = self.robot.joint_move_j(point.j1, point.j2, point.j3, point.j4)
                
            elif move_type in ['L', 'MovL']:
                # 設置直線運動參數
                if hasattr(self.robot, 'set_speed_l'):
                    self.robot.set_speed_l(speed_l)
                if hasattr(self.robot, 'set_acc_l'):
                    self.robot.set_acc_l(acc_l)
                if hasattr(self.robot, 'set_tool'):
                    self.robot.set_tool(tool)
                
                # 執行直線運動
                success = self.robot.move_l(point.x, point.y, point.z, point.r)
                
            elif move_type == 'MovJ':
                # MovJ使用笛卡爾座標但關節插值
                if hasattr(self.robot, 'set_speed_j'):
                    self.robot.set_speed_j(speed_j)
                if hasattr(self.robot, 'set_acc_j'):
                    self.robot.set_acc_j(acc_j)
                if hasattr(self.robot, 'set_tool'):
                    self.robot.set_tool(tool)
                
                # 如果機械臂API支援MovJ
                if hasattr(self.robot, 'move_j'):
                    success = self.robot.move_j(point.x, point.y, point.z, point.r)
                else:
                    # 降級為JointMovJ
                    success = self.robot.joint_move_j(point.j1, point.j2, point.j3, point.j4)
                    
            else:
                self.logger.error(f"未支援的移動類型: {move_type}")
                return False
            
            # Sync控制
            if success and sync_enabled:
                self.robot.sync()
                self.logger.info(f"  移動到 {point_name} 成功 ({move_type}) (含Sync)")
            elif success:
                self.logger.info(f"  移動到 {point_name} 成功 ({move_type})")
            else:
                self.logger.error(f"  移動到 {point_name} 失敗")
                
            return success
                
        except Exception as e:
            self.logger.error(f"移動到點位失敗: {e}", exc_info=True)
            return False
    
    def _execute_move_to_point_with_j4(self, params: Dict[str, Any]) -> bool:
        """執行移動到點位並使用計算的J4角度 - 支援DR參數版"""
        try:
            point_name = params['point_name']
            move_type = params['move_type']
            
            # 從點位管理器獲取點位
            point = self.points_manager.get_point(point_name)
            if not point:
                self.logger.error(f"點位管理器中找不到點位: {point_name}")
                return False
            
            # 提取運動參數
            speed_j = params.get('speed_j', 30)  # J4角度控制通常使用較低速度
            acc_j = params.get('acc_j', 30)
            tool = params.get('tool', 0)
            sync_enabled = params.get('sync', self.sync_enable)
            
            # 使用計算的command_angle作為J4值
            if self.command_angle is not None:
                j4_value = self.command_angle
                self.logger.info(f"移動到點位 {point_name} (使用計算J4角度)")
                self.logger.debug(f"  關節角度: (j1:{point.j1:.1f}, j2:{point.j2:.1f}, j3:{point.j3:.1f}, j4:{j4_value:.1f})")
                self.logger.info(f"  計算角度: target={self.target_angle:.1f}° + offset={self.ANGLE_OFFSET}° = command={j4_value:.1f}°")
            else:
                j4_value = point.j4
                self.logger.info(f"移動到點位 {point_name} (使用原始J4角度)")
                self.logger.debug(f"  關節角度: (j1:{point.j1:.1f}, j2:{point.j2:.1f}, j3:{point.j3:.1f}, j4:{j4_value:.1f})")
                self.logger.warning("未獲取角度數據，使用原始J4值")
            
            self.logger.debug(f"  運動參數: speed_j={speed_j}, acc_j={acc_j}, tool={tool}")
            
            success = False
            if move_type in ['J', 'JointMovJ']:
                # 設置關節運動參數
                if hasattr(self.robot, 'set_speed_j'):
                    self.robot.set_speed_j(speed_j)
                if hasattr(self.robot, 'set_acc_j'):
                    self.robot.set_acc_j(acc_j)
                if hasattr(self.robot, 'set_tool'):
                    self.robot.set_tool(tool)
                
                # 使用關節角度運動，J4使用計算角度
                success = self.robot.joint_move_j(point.j1, point.j2, point.j3, j4_value)
            else:
                self.logger.error(f"J4角度控制僅支援關節運動(J)，當前類型: {move_type}")
                return False
            
            # Sync控制
            if success and sync_enabled:
                self.robot.sync()
                self.logger.info(f"  移動到 {point_name} 成功 (J4:{j4_value:.1f}°) (含Sync)")
            elif success:
                self.logger.info(f"  移動到 {point_name} 成功 (J4:{j4_value:.1f}°)")
                
            return success
                
        except Exception as e:
            self.logger.error(f"移動到點位(J4角度控制)失敗: {e}", exc_info=True)
            return False
    
    def _execute_angle_detection_fast(self) -> bool:
        """執行AngleHighLevel角度檢測 - 快速版本"""
        try:
            self.logger.info("[快速角度檢測] 開始檢測...")
            detection_start_time = time.time()
            
            # 優先使用external_modules中的angle模組
            angle_controller = self.external_modules.get('angle')
            
            if not angle_controller:
                self.logger.warning("[快速角度檢測] 角度模組未連接，快速使用預設值")
                self._set_default_angle()
                return True
            
            self.logger.info("[快速角度檢測] 使用外部模組中的角度API")
            
            # 檢查角度控制器連接狀態
            if hasattr(angle_controller, 'connected') and not angle_controller.connected:
                self.logger.warning("[快速角度檢測] 角度控制器未連接，嘗試快速重連...")
                if not angle_controller.connect():
                    self.logger.warning("[快速角度檢測] 快速重連失敗，使用預設值")
                    self._set_default_angle()
                    return True
            
            # 執行快速角度檢測
            self.logger.info("[快速角度檢測] 執行CCD3角度檢測(DR模式)...")
            
            try:
                # 使用超時機制執行角度檢測
                detection_result = self._execute_angle_detection_with_timeout(angle_controller)
                
                if detection_result is None:
                    self.logger.warning(f"[快速角度檢測] 檢測超時({self.angle_detection_timeout}秒)，使用預設值")
                    self._set_default_angle()
                    return True
                
                detection_time = time.time() - detection_start_time
                
                if (hasattr(detection_result, 'result') and 
                    detection_result.result.value == "SUCCESS" and 
                    detection_result.target_angle is not None):
                    
                    self.target_angle = detection_result.target_angle
                    self.command_angle = self.target_angle + self.ANGLE_OFFSET
                    self.angle_acquisition_success = True
                    
                    self.logger.info(f"[快速角度檢測] 角度檢測成功 (耗時: {detection_time:.2f}秒):")
                    self.logger.info(f"    目標角度: {self.target_angle:.2f}°")
                    self.logger.info(f"    指令角度: {self.command_angle:.2f}°")
                    
                    return True
                else:
                    error_msg = getattr(detection_result, 'message', '未知錯誤') if detection_result else '檢測失敗'
                    self.logger.warning(f"[快速角度檢測] 檢測失敗: {error_msg} (耗時: {detection_time:.2f}秒)")
                    self._set_default_angle()
                    return True
                    
            except Exception as detection_error:
                detection_time = time.time() - detection_start_time
                self.logger.error(f"[快速角度檢測] 檢測異常: {detection_error} (耗時: {detection_time:.2f}秒)")
                self._set_default_angle()
                return True
                
        except Exception as e:
            detection_time = time.time() - detection_start_time
            self.logger.error(f"[快速角度檢測] 系統異常: {e} (耗時: {detection_time:.2f}秒)", exc_info=True)
            self._set_default_angle()
            return True
    
    def _execute_angle_detection_with_timeout(self, angle_controller) -> Optional[Any]:
        """帶超時的角度檢測執行"""
        import threading
        import queue
        
        result_queue = queue.Queue()
        
        def detection_thread():
            try:
                result = angle_controller.detect_angle(detection_mode=1)  # DR模式
                result_queue.put(result)
            except Exception as e:
                self.logger.debug(f"角度檢測線程異常: {e}")
                result_queue.put(None)
        
        # 啟動檢測線程
        thread = threading.Thread(target=detection_thread, daemon=True)
        thread.start()
        
        # 等待結果或超時
        try:
            result = result_queue.get(timeout=self.angle_detection_timeout)
            return result
        except queue.Empty:
            self.logger.warning(f"[快速角度檢測] 檢測超時 ({self.angle_detection_timeout}秒)")
            return None
    
    def _set_default_angle(self):
        """設置預設角度值"""
        self.target_angle = 0.0
        self.command_angle = self.target_angle + self.ANGLE_OFFSET
        self.angle_acquisition_success = False
        self.logger.info(f"[快速角度檢測] 使用預設角度: target={self.target_angle}°, command={self.command_angle}°")
    
    def _execute_move_to_detected_position_with_parameters(self, detected_position: Optional[Dict[str, float]], params: Dict[str, Any]) -> bool:
        """移動到檢測位置 - 支援DR參數版本"""
        try:
            if not detected_position:
                self.logger.error("檢測位置為空，無法移動")
                return False
            
            # 取得VP_TOPSIDE點位的Z高度和R值
            vp_topside_point = self.points_manager.get_point('VP_TOPSIDE')  # 改為大寫
            if not vp_topside_point:
                print(f"  ✗ VP_TOPSIDE點位不存在")
                # 嘗試小寫版本作為備用
                vp_topside_point = self.points_manager.get_point('vp_topside')
                if vp_topside_point:
                    print(f"  ✓ 找到小寫版本vp_topside點位")
                else:
                    print(f"  ✗ VP_TOPSIDE和vp_topside點位都不存在")
                    print(f"  可用點位: {self.points_manager.list_points()}")
                    time.sleep(0.1)
                    
            else:
                print(f"  ✓ VP_TOPSIDE點位存在: Z={vp_topside_point.z}, R={vp_topside_point.r}")
            
            
            # 提取運動參數
            speed_l = params.get('speed_l', 50)
            acc_l = params.get('acc_l', 50)
            tool = params.get('tool', 0)
            sync_enabled = params.get('sync', self.sync_enable)
            
            # 決定目標高度 (檢測高度或夾取高度)
            target_height = vp_topside_point.z if 'high' in str(params) else self.PICKUP_HEIGHT
            
            self.logger.info(f"移動到檢測位置:")
            self.logger.debug(f"  AutoProgram座標XY: ({detected_position['x']:.2f}, {detected_position['y']:.2f})")
            self.logger.debug(f"  目標高度Z: {target_height:.2f}")
            self.logger.debug(f"  繼承R: {vp_topside_point.r:.2f} (VP_TOPSIDE角度)")
            self.logger.debug(f"  運動參數: speed_l={speed_l}, acc_l={acc_l}, tool={tool}")
            
            # 設置直線運動參數
            if hasattr(self.robot, 'set_speed_l'):
                self.robot.set_speed_l(speed_l)
            if hasattr(self.robot, 'set_acc_l'):
                self.robot.set_acc_l(acc_l)
            if hasattr(self.robot, 'set_tool'):
                self.robot.set_tool(tool)
            
            # 執行移動
            success = self.robot.move_l(
                detected_position['x'],
                detected_position['y'],
                target_height,
                vp_topside_point.r
            )
            
            # Sync控制
            if success and sync_enabled:
                self.robot.sync()
                self.logger.info("  移動到檢測位置完成 (含Sync)")
            elif success:
                self.logger.info("  移動到檢測位置完成")
            else:
                self.logger.error("  移動到檢測位置失敗")
                
            return success
                
        except Exception as e:
            self.logger.error(f"移動到檢測位置失敗: {e}", exc_info=True)
            return False
    
    def _execute_gripper_close(self) -> bool:
        """執行夾爪關閉"""
        try:
            gripper_api = self.external_modules.get('gripper')
            if gripper_api:
                success = gripper_api.quick_close()
                if success:
                    self.logger.info("夾爪關閉成功")
                else:
                    self.logger.error("夾爪關閉失敗")
                return success
            else:
                self.logger.error("夾爪API未初始化")
                return False
        except Exception as e:
            self.logger.error(f"夾爪關閉失敗: {e}", exc_info=True)
            return False
    
    def _execute_gripper_smart_release(self, params: Dict[str, Any]) -> bool:
        """執行夾爪智能撐開"""
        try:
            position = params.get('position', 370)
            self.logger.info(f"夾爪智能撐開到位置: {position}")
            
            gripper_api = self.external_modules.get('gripper')
            if not gripper_api:
                self.logger.error("夾爪API未初始化")
                return False
            
            # 執行智能撐開操作
            success = gripper_api.smart_release(position)
            
            if success:
                self.logger.info("夾爪智能撐開指令發送成功")
                
                # 等待夾爪撐開操作完全完成
                self.logger.debug("等待夾爪撐開動作完成...")
                time.sleep(0.3)
                
                self.logger.info(f"夾爪智能撐開完成 - 位置{position}")
                return True
            else:
                self.logger.error("夾爪智能撐開失敗")
                return False
                
        except Exception as e:
            self.logger.error(f"夾爪智能撐開異常: {e}", exc_info=True)
            return False
    
    
    def _execute_read_autoprogram_coordinates(self) -> Optional[Dict[str, float]]:
        """從AutoProgram讀取座標 """
        max_retries = 20
        retry_count = 0
        
        print("\n" + "="*50)
        print("開始從AutoProgram讀取座標")
        print("="*50)
        
        while retry_count < max_retries:
            try:
                retry_count += 1
                
                print(f"\n[重試 {retry_count}/{max_retries}] 檢查AutoProgram狀態...")
                
                # Step 1: 檢查連接狀態
                if not self.autoprogram_interface.ensure_connection():
                    print(f"  ✗ AutoProgram連接失敗")
                    if retry_count <= 3:  # 前3次重試時詳細輸出
                        print(f"    - Modbus服務器: {self.autoprogram_interface.modbus_host}:{self.autoprogram_interface.modbus_port}")
                        print(f"    - 連接狀態: {self.autoprogram_interface.connected}")
                    time.sleep(0.2)
                    continue
                else:
                    print(f"  ✓ AutoProgram連接正常")
                
                # Step 2: 讀取AutoProgram系統狀態
                system_status = self.autoprogram_interface.read_register('SYSTEM_STATUS')
                prepare_done = self.autoprogram_interface.read_register('PREPARE_DONE')
                auto_enabled = self.autoprogram_interface.read_register('AUTO_PROGRAM_ENABLED')
                af_dr_f = self.autoprogram_interface.read_register('AF_DR_F_STATUS')
                
                print(f"  AutoProgram狀態檢查:")
                print(f"    - SYSTEM_STATUS(1300): {system_status} (需要1或2)")
                print(f"    - PREPARE_DONE(1301): {prepare_done} (需要0)")
                print(f"    - AUTO_PROGRAM_ENABLED(1302): {auto_enabled}")
                print(f"    - AF_DR_F_STATUS(1303): {af_dr_f}")
                
                # Step 3: 檢查系統狀態是否符合要求
                # 接受狀態: 1=運行中, 2=Flow1觸發, 3=Flow1等待
                if system_status not in [1, 2, 3]:
                    print(f"  ✗ 系統狀態不符合要求: {system_status} (需要1=運行中, 2=Flow1觸發, 或 3=Flow1等待)")
                    time.sleep(0.2)
                    continue
                else:
                    status_name = {1: "運行中", 2: "Flow1觸發", 3: "Flow1等待"}.get(system_status, "未知")
                    print(f"  ✓ 系統狀態符合要求: {system_status} ({status_name})")
                
                # Step 4: 檢查prepare_done狀態
                if prepare_done != 0:
                    print(f"  ✗ prepare_done狀態錯誤: {prepare_done} (需要0=需要取料)")
                    time.sleep(0.2)
                    continue
                else:
                    print(f"  ✓ prepare_done狀態正確: {prepare_done}")
                
                # Step 5: 詳細檢查座標寄存器
                print(f"  座標寄存器檢查:")
                
                # 使用批量讀取
                try:
                    result = self.autoprogram_interface.modbus_client.read_holding_registers(
                        address=self.autoprogram_interface.REGISTERS['TARGET_X_HIGH'], 
                        count=4, 
                        slave=1
                    )
                    if result.isError():
                        print(f"    ✗ 批量讀取座標失敗: {result}")
                        x_high = self.autoprogram_interface.read_register('TARGET_X_HIGH') or 0
                        x_low = self.autoprogram_interface.read_register('TARGET_X_LOW') or 0
                        y_high = self.autoprogram_interface.read_register('TARGET_Y_HIGH') or 0
                        y_low = self.autoprogram_interface.read_register('TARGET_Y_LOW') or 0
                        print(f"    - 改用單個讀取")
                    else:
                        registers = result.registers
                        x_high, x_low, y_high, y_low = registers[0], registers[1], registers[2], registers[3]
                        print(f"    ✓ 批量讀取座標成功")
                except Exception as e:
                    print(f"    ✗ 批量讀取異常: {e}")
                    x_high = self.autoprogram_interface.read_register('TARGET_X_HIGH') or 0
                    x_low = self.autoprogram_interface.read_register('TARGET_X_LOW') or 0
                    y_high = self.autoprogram_interface.read_register('TARGET_Y_HIGH') or 0
                    y_low = self.autoprogram_interface.read_register('TARGET_Y_LOW') or 0
                    print(f"    - 改用單個讀取")
                
                print(f"    - TARGET_X_HIGH(1340): {x_high}")
                print(f"    - TARGET_X_LOW(1341): {x_low}")
                print(f"    - TARGET_Y_HIGH(1342): {y_high}")
                print(f"    - TARGET_Y_LOW(1343): {y_low}")
                
                # Step 6: 檢查座標是否有效（非全零）
                if x_high == 0 and x_low == 0 and y_high == 0 and y_low == 0:
                    print(f"  ✗ 座標寄存器全為0，座標未準備好")
                    time.sleep(0.2)
                    continue
                else:
                    print(f"  ✓ 座標寄存器有有效數據")
                
                # Step 7: 座標轉換和驗證
                print(f"  座標轉換:")
                
                # 32位合併
                world_x_int = (x_high << 16) | x_low
                world_y_int = (y_high << 16) | y_low
                print(f"    - 合併後整數值: X={world_x_int}, Y={world_y_int}")
                
                # 處理負數 (補碼轉換)
                if world_x_int >= 2**31:
                    world_x_int -= 2**32
                    print(f"    - X負數轉換: {world_x_int}")
                if world_y_int >= 2**31:
                    world_y_int -= 2**32
                    print(f"    - Y負數轉換: {world_y_int}")
                
                # 恢復精度 (÷100)
                world_x = world_x_int / 100.0
                world_y = world_y_int / 100.0
                print(f"    - 最終座標: X={world_x:.2f}, Y={world_y:.2f}")
                
                # Step 8: 座標合理性檢查
                if abs(world_x) > 1000 or abs(world_y) > 1000:
                    print(f"  ⚠ 座標值異常，可能存在轉換錯誤")
                    print(f"    - 檢查原始寄存器值是否正確")
                
                # Step 9: 確認座標已讀取
                print(f"  確認座標讀取:")
                coords_taken_success = self.autoprogram_interface.write_register('AF_COORDS_TAKEN', 1)
                if not coords_taken_success:
                    print(f"    ✗ 寫入AF_COORDS_TAKEN(945)失敗")
                    time.sleep(0.1)
                    continue
                else:
                    print(f"    ✓ AF_COORDS_TAKEN(945)寫入成功")
                
                # Step 10: 驗證vp_topside點位
                vp_topside_point = self.points_manager.get_point('VP_TOPSIDE')
                if not vp_topside_point:
                    print(f"  ✗ vp_topside點位不存在")
                    time.sleep(0.1)
                    continue
                else:
                    print(f"  ✓ vp_topside點位存在: Z={vp_topside_point.z}, R={vp_topside_point.r}")
                
                # Step 11: 構建最終結果
                detected_pos = {
                    'x': world_x,
                    'y': world_y,
                    'z': vp_topside_point.z,
                    'r': vp_topside_point.r,
                    'source': 'autoprogram_interface',
                    'retry_count': retry_count,
                    'raw_registers': {
                        'x_high': x_high,
                        'x_low': x_low,
                        'y_high': y_high,
                        'y_low': y_low
                    }
                }
                
                print(f"\n" + "="*50)
                print(f"✓ AutoProgram座標讀取成功！")
                print(f"  重試次數: {retry_count}")
                print(f"  最終座標: ({detected_pos['x']:.2f}, {detected_pos['y']:.2f}, {detected_pos['z']:.2f}, {detected_pos['r']:.2f})")
                print(f"  原始寄存器: X({x_high},{x_low}) Y({y_high},{y_low})")
                print(f"  AutoProgram狀態: system_status={system_status}, prepare_done={prepare_done}")
                print("="*50)
                
                return detected_pos
                
            except Exception as e:
                print(f"  ✗ 重試{retry_count} 發生異常: {e}")
                print(f"    異常類型: {type(e).__name__}")
                if hasattr(e, '__traceback__'):
                    import traceback
                    print(f"    異常詳情: {traceback.format_exc()}")
                time.sleep(0.1)
                continue
        
        # 所有重試都失敗
        print(f"\n" + "="*50)
        print(f"✗ AutoProgram座標讀取失敗！")
        print(f"  已重試: {max_retries}次")
        print(f"  最後狀態檢查:")
        
        try:
            # 最後一次狀態檢查
            final_status = self.autoprogram_interface.get_autoprogram_status_info()
            for key, value in final_status.items():
                print(f"    - {key}: {value}")
        except Exception as e:
            print(f"    - 無法獲取最終狀態: {e}")
        
        print("="*50)
        return None
    
    def _safe_set_flow1_complete_status(self, complete: bool) -> bool:
        """安全設置Flow1完成狀態"""
        try:
            self.logger.info(f"[Flow1] 設置Flow1完成狀態: {complete}")
            
            # 方法1: 透過motion_state_machine設置 (優先且推薦)
            if self.motion_state_machine:
                try:
                    self.motion_state_machine.set_flow_complete(1, complete)
                    self.logger.info(f"[Flow1] 透過狀態機設置Flow1完成狀態: {complete}")
                    return True
                except Exception as e:
                    self.logger.error(f"[Flow1] 狀態機設置失敗: {e}")
            
            # 方法2: 直接Modbus寫入 (備用方案)
            if not self.modbus_client:
                self.modbus_client = ModbusTcpClient(host="127.0.0.1", port=502, timeout=2.0)
                if not self.modbus_client.connect():
                    self.logger.error("[Flow1] Flow1完成狀態設置失敗：無法連接Modbus")
                    return False
            
            value = 1 if complete else 0
            result = self.modbus_client.write_register(
                address=self.FLOW1_COMPLETE_REGISTER, 
                value=value
            )
            
            if hasattr(result, 'isError') and result.isError():
                self.logger.error(f"[Flow1] Flow1完成狀態直接寫入失敗: {result}")
                return False
            else:
                self.logger.info(f"[Flow1] Flow1完成狀態直接寫入成功: 地址{self.FLOW1_COMPLETE_REGISTER} = {value}")
                return True
                
        except Exception as e:
            self.logger.error(f"[Flow1] 設置Flow1完成狀態異常: {e}", exc_info=True)
            return False
    
    def pause(self) -> bool:
        """暫停Flow"""
        self.status = FlowStatus.PAUSED
        self.logger.info("Flow1已暫停")
        return True
        
    def resume(self) -> bool:
        """恢復Flow"""
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            self.logger.info("Flow1已恢復")
            return True
        return False
        
    def stop(self) -> bool:
        """停止Flow"""
        self.status = FlowStatus.ERROR
        self.logger.info("Flow1已停止")
        return True
    
    def get_target_angle(self) -> Optional[float]:
        """供Flow2調用：獲取target_angle"""
        return self.target_angle
        
    def get_command_angle(self) -> Optional[float]:
        """供Flow2調用：獲取command_angle"""
        return self.command_angle
    
    def has_valid_angle_data(self) -> bool:
        """供Flow2調用：檢查角度數據是否有效"""
        return (self.angle_acquisition_success and 
                self.target_angle is not None and 
                self.command_angle is not None)
        
    def get_progress(self) -> int:
        """取得進度百分比"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)
    
    def is_ready(self) -> bool:
        """檢查Flow1是否準備好執行"""
        return (self.points_loaded and 
                self.total_steps > 0 and 
                self.autoprogram_interface.connected)
    
    def cleanup(self):
        """清理資源"""
        if hasattr(self, 'autoprogram_interface'):
            self.autoprogram_interface.disconnect()
        
        # 關閉Flow1的Modbus連接
        if hasattr(self, 'modbus_client') and self.modbus_client:
            try:
                self.modbus_client.close()
                self.logger.info("Flow1 Modbus連接已關閉")
            except Exception as e:
                self.logger.debug(f"關閉Modbus連接異常: {e}")


# 兼容性別名
class Flow1Executor(DrFlow1VisionPickExecutor):
    """Flow1執行器 - 兼容性包裝器"""
    pass