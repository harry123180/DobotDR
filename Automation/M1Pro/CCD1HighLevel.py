# -*- coding: utf-8 -*-
"""
CCD1HighLevel.py - CCD1高層API模組 (最終修正版 - 基於可工作的DR邏輯)
提供簡化的CCD1功能介面，處理複雜的ModbusTCP握手協議和FIFO佇列管理
適用於其他模組import使用

關鍵修正：
1. 採用舊版本DR的成功邏輯
2. 使用批量讀取方式 (_read_multiple_registers)
3. 保持原始的32位合併邏輯
4. 添加調試功能以便排查問題
"""

import time
import threading
from typing import Optional, Tuple, List, Dict, Any
from collections import deque
from enum import IntEnum
import logging
from dataclasses import dataclass

# 導入Modbus TCP Client (適配pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ Modbus Client模組導入失敗: {e}")
    MODBUS_AVAILABLE = False


# ==================== 控制指令枚舉 ====================
class CCD1Command(IntEnum):
    """CCD1控制指令枚舉"""
    CLEAR = 0
    CAPTURE = 8
    CAPTURE_DETECT = 16
    INITIALIZE = 32


# ==================== 狀態位枚舉 ====================
class CCD1StatusBits(IntEnum):
    """CCD1狀態位枚舉"""
    READY = 0
    RUNNING = 1
    ALARM = 2
    INITIALIZED = 3


# ==================== 圓心座標數據結構 ====================
@dataclass
class CircleWorldCoord:
    """圓心世界座標數據"""
    id: int                    # 圓形ID
    world_x: float            # 世界座標X (mm)
    world_y: float            # 世界座標Y (mm)
    pixel_x: int              # 像素座標X
    pixel_y: int              # 像素座標Y
    radius: int               # 半徑 (像素)
    timestamp: str            # 檢測時間戳
    r: float = 0.0            # 旋轉角度 (可由Flow1設定)


# ==================== CCD1高層API類 ====================
class CCD1HighLevelAPI:
    """
    CCD1高層API - 最終修正版 (基於可工作的DR邏輯)
    
    主要功能:
    1. 拍照+檢測指令 (自動處理握手協議)
    2. 獲取物件圓心世界座標 (FIFO佇列管理)
    3. 使用成功驗證的批量讀取邏輯
    """
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        """
        初始化CCD1高層API
        
        Args:
            modbus_host: Modbus TCP服務器IP
            modbus_port: Modbus TCP服務器端口
        """
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # 使用簡化的寄存器映射 (與可工作版本一致)
        self.REGISTERS = {
            'CONTROL_COMMAND': 200,        # 控制指令
            'STATUS_REGISTER': 201,        # 狀態寄存器
            'CIRCLE_COUNT': 240,           # 檢測圓形數量
            'WORLD_COORD_VALID': 256,      # 世界座標有效標誌
        }
        
        # 圓心座標FIFO佇列
        self.coord_queue = deque()  # 圓心座標佇列
        self.queue_lock = threading.Lock()  # 佇列操作鎖
        
        # 狀態追蹤
        self.last_detection_count = 0
        self.operation_timeout = 10.0  # 操作超時時間(秒)
        
        # 設置日誌
        self.logger = logging.getLogger("CCD1HighLevel")
        self.logger.setLevel(logging.INFO)
        
        # 自動連接
        self.connect()
    
    def connect(self) -> bool:
        """
        連接到Modbus TCP服務器
        
        Returns:
            bool: 連接是否成功
        """
        if not MODBUS_AVAILABLE:
            self.logger.error("Modbus Client不可用")
            return False
        
        try:
            if self.modbus_client:
                self.modbus_client.close()
            
            self.logger.info(f"正在連接Modbus TCP服務器: {self.modbus_host}:{self.modbus_port}")
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3.0
            )
            
            if self.modbus_client.connect():
                self.connected = True
                self.logger.info(f"Modbus TCP連接成功: {self.modbus_host}:{self.modbus_port}")
                return True
            else:
                self.logger.error(f"Modbus TCP連接失敗: {self.modbus_host}:{self.modbus_port}")
                self.connected = False
                return False
                
        except Exception as e:
            self.logger.error(f"Modbus TCP連接異常: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """斷開Modbus連接"""
        if self.modbus_client and self.connected:
            try:
                self.modbus_client.close()
                self.logger.info("Modbus TCP連接已斷開")
            except:
                pass
        
        self.connected = False
        self.modbus_client = None
    
    def _read_register(self, register_name: str) -> Optional[int]:
        """讀取寄存器"""
        if not self.connected or not self.modbus_client or register_name not in self.REGISTERS:
            return None
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.read_holding_registers(address, count=1, slave=1)
            
            if not result.isError():
                return result.registers[0]
            else:
                return None
                
        except Exception as e:
            self.logger.error(f"讀取寄存器失敗: {e}")
            return None
    
    def _write_register(self, register_name: str, value: int) -> bool:
        """寫入寄存器"""
        if not self.connected or not self.modbus_client or register_name not in self.REGISTERS:
            return False
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.write_register(address, value, slave=1)
            
            return not result.isError()
                
        except Exception as e:
            self.logger.error(f"寫入寄存器失敗: {e}")
            return False
    
    def _read_multiple_registers(self, start_address: int, count: int) -> Optional[List[int]]:
        """讀取多個寄存器"""
        if not self.connected or not self.modbus_client:
            return None
        
        try:
            result = self.modbus_client.read_holding_registers(start_address, count=count, slave=1)
            
            if not result.isError():
                return result.registers
            else:
                return None
                
        except Exception as e:
            self.logger.error(f"讀取多個寄存器失敗: {e}")
            return None
    
    def _wait_for_ready(self, timeout: float = 10.0) -> bool:
        """
        等待CCD1系統Ready狀態
        
        Args:
            timeout: 超時時間(秒)
            
        Returns:
            bool: 是否Ready
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            status = self._read_register('STATUS_REGISTER')
            if status is not None:
                ready = bool(status & (1 << CCD1StatusBits.READY))
                running = bool(status & (1 << CCD1StatusBits.RUNNING))
                alarm = bool(status & (1 << CCD1StatusBits.ALARM))
                
                if alarm:
                    self.logger.warning("CCD1系統處於Alarm狀態")
                    return False
                
                if ready and not running:
                    return True
            
            time.sleep(0.1)  # 100ms檢查間隔
        
        self.logger.error(f"等待Ready狀態超時: {timeout}秒")
        return False
    
    def _wait_for_command_complete(self, timeout: float = 10.0) -> bool:
        """
        等待指令執行完成
        
        Args:
            timeout: 超時時間(秒)
            
        Returns:
            bool: 指令是否執行完成
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            status = self._read_register('STATUS_REGISTER')
            if status is not None:
                running = bool(status & (1 << CCD1StatusBits.RUNNING))
                alarm = bool(status & (1 << CCD1StatusBits.ALARM))
                
                if alarm:
                    self.logger.warning("CCD1系統執行中發生Alarm")
                    return False
                
                if not running:
                    return True
            
            time.sleep(0.1)  # 100ms檢查間隔
        
        self.logger.error(f"等待指令完成超時: {timeout}秒")
        return False
    
    def _read_world_coordinates(self) -> List[CircleWorldCoord]:
        """
        🔥 最終修正：正確的32位有符號整數重建
        
        關鍵發現：不能先轉換16位再進行位運算，應該先進行32位合併再轉換
        ModbusPoll顯示負數，但pymodbus讀取為無符號，需要正確處理這個差異
        
        Returns:
            List[CircleWorldCoord]: 圓心世界座標列表
        """
        # 檢查世界座標有效性
        world_coord_valid = self._read_register('WORLD_COORD_VALID')
        if not world_coord_valid:
            self.logger.warning("世界座標無效，可能缺少標定數據")
            return []
        
        # 讀取檢測到的圓形數量
        circle_count = self._read_register('CIRCLE_COUNT')
        if not circle_count or circle_count == 0:
            self.logger.info("未檢測到圓形")
            return []
        
        # 限制最多5個圓形
        circle_count = min(circle_count, 5)
        
        # 讀取像素座標結果 (241-255)
        pixel_registers = self._read_multiple_registers(241, 15)  # 241-255
        
        # 讀取世界座標結果 (257-276)
        world_registers = self._read_multiple_registers(257, 20)  # 257-276
        
        if not pixel_registers or not world_registers:
            self.logger.error("讀取檢測結果失敗")
            return []
        
        # 📊 調試：顯示原始寄存器數據
        self.logger.info(f"原始像素座標寄存器 (241-255): {pixel_registers}")
        self.logger.info(f"原始世界座標寄存器 (257-276): {world_registers}")
        
        coordinates = []
        current_time = time.strftime("%Y-%m-%d %H:%M:%S")
        
        for i in range(circle_count):
            try:
                # 像素座標 (每個圓形3個寄存器: X, Y, Radius)
                pixel_start_idx = i * 3
                if pixel_start_idx + 2 < len(pixel_registers):
                    pixel_x = pixel_registers[pixel_start_idx]
                    pixel_y = pixel_registers[pixel_start_idx + 1]
                    radius = pixel_registers[pixel_start_idx + 2]
                else:
                    self.logger.warning(f"圓形{i+1}像素座標索引越界")
                    continue
                
                # 世界座標 (每個圓形4個寄存器: X高位, X低位, Y高位, Y低位)
                world_start_idx = i * 4
                if world_start_idx + 3 < len(world_registers):
                    world_x_high_raw = world_registers[world_start_idx]
                    world_x_low_raw = world_registers[world_start_idx + 1]
                    world_y_high_raw = world_registers[world_start_idx + 2]
                    world_y_low_raw = world_registers[world_start_idx + 3]
                    
                    # 📊 調試：顯示原始數據
                    self.logger.info(f"圓形{i+1}原始無符號16位數據:")
                    self.logger.info(f"  X_HIGH={world_x_high_raw}, X_LOW={world_x_low_raw}")
                    self.logger.info(f"  Y_HIGH={world_y_high_raw}, Y_LOW={world_y_low_raw}")
                    
                    # 🔥 正確的方法：直接進行32位合併，不要先轉換16位
                    # 將無符號16位直接合併為32位無符號整數
                    world_x_uint32 = (world_x_high_raw << 16) | world_x_low_raw
                    world_y_uint32 = (world_y_high_raw << 16) | world_y_low_raw
                    
                    # 📊 調試：顯示32位無符號合併結果
                    self.logger.info(f"  32位無符號合併:")
                    self.logger.info(f"    X_UINT32={world_x_uint32} (0x{world_x_uint32:08X})")
                    self.logger.info(f"    Y_UINT32={world_y_uint32} (0x{world_y_uint32:08X})")
                    
                    # 🔥 然後轉換為32位有符號整數
                    if world_x_uint32 > 2147483647:  # 大於2^31-1的轉換為負數
                        world_x_int = world_x_uint32 - 4294967296  # 減去2^32
                    else:
                        world_x_int = world_x_uint32
                    
                    if world_y_uint32 > 2147483647:
                        world_y_int = world_y_uint32 - 4294967296
                    else:
                        world_y_int = world_y_uint32
                    
                    # 📊 調試：顯示32位有符號轉換結果
                    self.logger.info(f"  32位有符號轉換:")
                    self.logger.info(f"    X_INT={world_x_int}")
                    self.logger.info(f"    Y_INT={world_y_int}")
                    
                    # 恢復精度 (÷100)
                    world_x = world_x_int / 100.0
                    world_y = world_y_int / 100.0
                    
                    # 📊 調試：顯示最終結果
                    self.logger.info(f"  最終座標: X={world_x:.2f}mm, Y={world_y:.2f}mm")
                    
                    # 🔍 合理性檢查
                    if abs(world_x) > 1000 or abs(world_y) > 1000:
                        self.logger.warning(f"  ⚠️ 座標值異常大: X={world_x:.2f}mm, Y={world_y:.2f}mm")
                    else:
                        self.logger.info(f"  ✅ 座標值在合理範圍內")
                    
                else:
                    self.logger.warning(f"圓形{i+1}世界座標索引越界")
                    continue
                
                coord = CircleWorldCoord(
                    id=i + 1,
                    world_x=world_x,
                    world_y=world_y,
                    pixel_x=pixel_x,
                    pixel_y=pixel_y,
                    radius=radius,
                    timestamp=current_time
                )
                coordinates.append(coord)
                
                self.logger.info(f"✅ 圓形{i+1}解析完成: 世界座標=({world_x:.2f}, {world_y:.2f})mm")
                
            except Exception as e:
                self.logger.error(f"❌ 圓形{i+1}解析失敗: {e}")
                continue
        
        self.logger.info(f"世界座標解析完成，共解析 {len(coordinates)} 個圓形")
        return coordinates
    
    def capture_and_detect(self) -> bool:
        """
        執行拍照+檢測指令 (使用可工作的DR邏輯)
        
        本方法處理完整的握手協議，包括:
        1. 檢查Ready狀態
        2. 發送拍照+檢測指令 (16)
        3. 等待執行完成
        4. 讀取檢測結果並更新FIFO佇列
        
        Returns:
            bool: 操作是否成功
        """
        if not self.connected:
            self.logger.error("Modbus未連接")
            return False
        
        try:
            # 1. 等待Ready狀態
            if not self._wait_for_ready(self.operation_timeout):
                self.logger.error("系統未Ready，無法執行檢測")
                return False
            
            # 2. 發送拍照+檢測指令
            self.logger.info("發送拍照+檢測指令...")
            if not self._write_register('CONTROL_COMMAND', CCD1Command.CAPTURE_DETECT):
                self.logger.error("發送檢測指令失敗")
                return False
            
            # 3. 等待執行完成
            if not self._wait_for_command_complete(self.operation_timeout):
                self.logger.error("檢測指令執行失敗或超時")
                return False
            
            # 4. 讀取檢測結果
            coordinates = self._read_world_coordinates()
            
            # 5. 更新FIFO佇列
            with self.queue_lock:
                for coord in coordinates:
                    self.coord_queue.append(coord)
                
                self.last_detection_count = len(coordinates)
            
            # 6. 清空控制指令 (完成握手)
            self._write_register('CONTROL_COMMAND', CCD1Command.CLEAR)
            
            self.logger.info(f"檢測完成，新增 {len(coordinates)} 個圓心座標到佇列")
            return True
            
        except Exception as e:
            self.logger.error(f"拍照檢測執行異常: {e}")
            return False
    
    def get_next_circle_world_coord(self) -> Optional[CircleWorldCoord]:
        """
        獲取下一個物件圓心世界座標
        
        FIFO佇列管理邏輯:
        1. 如果佇列為空，自動觸發拍照+檢測
        2. 從佇列前端取出一個座標
        3. 返回座標，佇列中移除該座標
        
        Returns:
            CircleWorldCoord: 圓心世界座標，如果無可用座標則返回None
        """
        with self.queue_lock:
            # 檢查佇列是否為空
            if len(self.coord_queue) == 0:
                self.logger.info("佇列為空，觸發新的拍照+檢測...")
                
                # 釋放鎖，執行檢測操作
                with_lock_released = True
        
        # 在鎖外執行檢測 (避免死鎖)
        if 'with_lock_released' in locals():
            success = self.capture_and_detect()
            if not success:
                self.logger.error("自動檢測失敗")
                return None
        
        # 重新獲取鎖並取出座標
        with self.queue_lock:
            if len(self.coord_queue) > 0:
                coord = self.coord_queue.popleft()  # FIFO: 從前端取出
                self.logger.info(f"返回圓心座標: ID={coord.id}, 世界座標=({coord.world_x:.2f}, {coord.world_y:.2f})mm")
                return coord
            else:
                self.logger.warning("佇列仍為空，無可用座標")
                return None
    
    def get_queue_status(self) -> Dict[str, Any]:
        """
        獲取佇列狀態資訊
        
        Returns:
            Dict: 包含佇列長度、最後檢測數量等資訊
        """
        with self.queue_lock:
            queue_length = len(self.coord_queue)
            queue_preview = []
            
            # 獲取前3個座標的預覽
            for i, coord in enumerate(list(self.coord_queue)[:3]):
                queue_preview.append({
                    'id': coord.id,
                    'world_x': coord.world_x,
                    'world_y': coord.world_y,
                    'timestamp': coord.timestamp
                })
        
        return {
            'connected': self.connected,
            'queue_length': queue_length,
            'last_detection_count': self.last_detection_count,
            'queue_preview': queue_preview,
            'modbus_server': f"{self.modbus_host}:{self.modbus_port}"
        }
    
    def clear_queue(self):
        """清空座標佇列"""
        with self.queue_lock:
            self.coord_queue.clear()
            self.logger.info("座標佇列已清空")
    
    def is_ready(self) -> bool:
        """
        檢查CCD1系統是否Ready
        
        Returns:
            bool: 系統是否Ready
        """
        if not self.connected:
            return False
        
        status = self._read_register('STATUS_REGISTER')
        if status is not None:
            ready = bool(status & (1 << CCD1StatusBits.READY))
            alarm = bool(status & (1 << CCD1StatusBits.ALARM))
            return ready and not alarm
        
        return False
    
    def get_system_status(self) -> Dict[str, Any]:
        """
        獲取CCD1系統狀態
        
        Returns:
            Dict: 系統狀態資訊
        """
        if not self.connected:
            return {
                'connected': False,
                'ready': False,
                'running': False,
                'alarm': False,
                'initialized': False,
                'world_coord_valid': False
            }
        
        status = self._read_register('STATUS_REGISTER')
        world_coord_valid = self._read_register('WORLD_COORD_VALID')
        
        if status is not None:
            return {
                'connected': True,
                'ready': bool(status & (1 << CCD1StatusBits.READY)),
                'running': bool(status & (1 << CCD1StatusBits.RUNNING)),
                'alarm': bool(status & (1 << CCD1StatusBits.ALARM)),
                'initialized': bool(status & (1 << CCD1StatusBits.INITIALIZED)),
                'world_coord_valid': bool(world_coord_valid) if world_coord_valid is not None else False,
                'status_register_value': status
            }
        
        return {
            'connected': True,
            'ready': False,
            'running': False,
            'alarm': True,
            'initialized': False,
            'world_coord_valid': False,
            'error': '無法讀取狀態寄存器'
        }
    
    def debug_raw_registers(self) -> Dict[str, Any]:
        """
        🔥 新增：調試原始寄存器數據 - 用於問題排查
        
        Returns:
            Dict: 原始寄存器數據
        """
        if not self.connected:
            return {'error': 'Modbus未連接'}
        
        try:
            debug_info = {
                'basic_registers': {
                    'control_command': self._read_register('CONTROL_COMMAND'),
                    'status_register': self._read_register('STATUS_REGISTER'),
                    'circle_count': self._read_register('CIRCLE_COUNT'),
                    'world_coord_valid': self._read_register('WORLD_COORD_VALID'),
                },
                'pixel_registers_241_255': self._read_multiple_registers(241, 15),
                'world_registers_257_276': self._read_multiple_registers(257, 20),
            }
            
            # 如果有檢測結果，分析第一個圓形
            if (debug_info['basic_registers']['circle_count'] and 
                debug_info['basic_registers']['circle_count'] > 0):
                
                pixel_regs = debug_info['pixel_registers_241_255']
                world_regs = debug_info['world_registers_257_276']
                
                if pixel_regs and world_regs:
                    debug_info['circle_1_analysis'] = {
                        'pixel_x': pixel_regs[0],
                        'pixel_y': pixel_regs[1],
                        'radius': pixel_regs[2],
                        'world_x_high': world_regs[0],
                        'world_x_low': world_regs[1],
                        'world_y_high': world_regs[2],
                        'world_y_low': world_regs[3],
                    }
                    
                    # 使用可工作DR的計算邏輯
                    world_x_int = (world_regs[0] << 16) | world_regs[1]
                    world_y_int = (world_regs[2] << 16) | world_regs[3]
                    
                    if world_x_int >= 2147483648:
                        world_x_int -= 4294967296
                    if world_y_int >= 2147483648:
                        world_y_int -= 4294967296
                    
                    world_x_mm = world_x_int / 100.0
                    world_y_mm = world_y_int / 100.0
                    
                    debug_info['circle_1_calculation'] = {
                        'world_x_int': world_x_int,
                        'world_y_int': world_y_int,
                        'world_x_mm': world_x_mm,
                        'world_y_mm': world_y_mm,
                        'reasonable_range': (-1000 <= world_x_mm <= 1000) and (-1000 <= world_y_mm <= 1000)
                    }
            
            return debug_info
            
        except Exception as e:
            return {'error': f'調試讀取失敗: {str(e)}'}
    
    def test_modbus_poll_data(self) -> Dict[str, Any]:
        """
        🔥 測試：使用ModbusPoll實測數據進行驗證 (修正版)
        
        Returns:
            Dict: 測試結果
        """
        # ModbusPoll實測數據 (你提供的截圖數據)
        test_data = {
            'world_x_high': -9242,
            'world_x_low': -26375,
            'world_y_high': 29945,
            'world_y_low': 0
        }
        
        # 但實際讀取的數據 (從你的調試輸出)
        actual_data = {
            'world_x_high': 65535,  # 這是 -1 的無符號16位表示
            'world_x_low': 56294,   # 這是 -9242 的無符號16位表示  
            'world_y_high': 370,
            'world_y_low': 32896
        }
        
        # 16位無符號到有符號轉換函數
        def uint16_to_int16(value):
            if value > 32767:
                return value - 65536
            return value
        
        try:
            # 🔥 修正版計算：先轉換為有符號16位
            # 使用實際讀取的數據
            x_high_signed = uint16_to_int16(actual_data['world_x_high'])  # 65535 -> -1
            x_low_signed = uint16_to_int16(actual_data['world_x_low'])    # 56294 -> -9242
            y_high_signed = uint16_to_int16(actual_data['world_y_high'])  # 370 -> 370
            y_low_signed = uint16_to_int16(actual_data['world_y_low'])    # 32896 -> 32896
            
            # 32位合併
            world_x_int = (x_high_signed << 16) | (x_low_signed & 0xFFFF)
            world_y_int = (y_high_signed << 16) | (y_low_signed & 0xFFFF)
            
            # 處理32位有符號範圍
            if world_x_int > 2147483647:
                world_x_int -= 4294967296
            if world_y_int > 2147483647:
                world_y_int -= 4294967296
            
            # 恢復精度
            world_x_mm = world_x_int / 100.0
            world_y_mm = world_y_int / 100.0
            
            return {
                'modbus_poll_data': test_data,
                'actual_read_data': actual_data,
                'conversion_process': {
                    'x_high_signed': x_high_signed,
                    'x_low_signed': x_low_signed,
                    'y_high_signed': y_high_signed,
                    'y_low_signed': y_low_signed
                },
                'calculation_steps': {
                    'world_x_int': world_x_int,
                    'world_y_int': world_y_int,
                    'world_x_mm': world_x_mm,
                    'world_y_mm': world_y_mm
                },
                'analysis': {
                    'x_reasonable': -1000 <= world_x_mm <= 1000,
                    'y_reasonable': -1000 <= world_y_mm <= 1000,
                    'overall_reasonable': (-1000 <= world_x_mm <= 1000) and (-1000 <= world_y_mm <= 1000),
                    'expected_improvement': '修正後Y座標應該變為合理數值'
                }
            }
            
        except Exception as e:
            return {'error': f'測試計算失敗: {str(e)}'}
    
    def test_correct_calculation(self) -> Dict[str, Any]:
        """
        🔥 新增：測試正確的計算方法
        
        使用實際讀取的數據測試新的計算邏輯
        """
        # 實際讀取的數據 (第一個圓形)
        actual_data = {
            'world_x_high': 65535,  # 應該對應 -1
            'world_x_low': 56294,   # 應該對應 -9242
            'world_y_high': 370,    # 應該對應 370
            'world_y_low': 32896    # 應該對應 -32640，但這裡有問題
        }
        
        try:
            # 🔥 新的正確方法：直接32位合併，不先轉換16位
            world_x_uint32 = (actual_data['world_x_high'] << 16) | actual_data['world_x_low']
            world_y_uint32 = (actual_data['world_y_high'] << 16) | actual_data['world_y_low']
            
            # 轉換為32位有符號
            if world_x_uint32 > 2147483647:
                world_x_int = world_x_uint32 - 4294967296
            else:
                world_x_int = world_x_uint32
            
            if world_y_uint32 > 2147483647:
                world_y_int = world_y_uint32 - 4294967296
            else:
                world_y_int = world_y_uint32
            
            # 恢復精度
            world_x_mm = world_x_int / 100.0
            world_y_mm = world_y_int / 100.0
            
            return {
                'method': '直接32位合併法',
                'input_data': actual_data,
                'calculation_steps': {
                    'x_uint32': world_x_uint32,
                    'y_uint32': world_y_uint32,
                    'x_uint32_hex': f'0x{world_x_uint32:08X}',
                    'y_uint32_hex': f'0x{world_y_uint32:08X}',
                    'x_int32': world_x_int,
                    'y_int32': world_y_int,
                    'x_mm': world_x_mm,
                    'y_mm': world_y_mm
                },
                'analysis': {
                    'x_reasonable': -1000 <= world_x_mm <= 1000,
                    'y_reasonable': -1000 <= world_y_mm <= 1000,
                    'overall_reasonable': (-1000 <= world_x_mm <= 1000) and (-1000 <= world_y_mm <= 1000),
                    'x_matches_expected': abs(world_x_mm - (-92.42)) < 0.1,
                    'improvement': f'Y座標從242812.16變為{world_y_mm:.2f}'
                }
            }
            
        except Exception as e:
            return {'error': f'測試計算失敗: {str(e)}'}
    
    def debug_y_coordinate_problem(self) -> Dict[str, Any]:
        """
        🔥 專門調試Y座標問題
        
        分析為什麼Y座標會是242812.16
        """
        # Y座標的實際數據
        y_high = 370
        y_low = 32896
        
        steps = []
        
        # 方法1：錯誤的方法 (先轉換16位)
        def uint16_to_int16(value):
            if value > 32767:
                return value - 65536
            return value
        
        y_high_signed = uint16_to_int16(y_high)
        y_low_signed = uint16_to_int16(y_low)
        y_int_wrong = (y_high_signed << 16) | (y_low_signed & 0xFFFF)
        y_mm_wrong = y_int_wrong / 100.0
        
        steps.append({
            'method': '錯誤方法 (先轉換16位)',
            'y_high_signed': y_high_signed,
            'y_low_signed': y_low_signed,
            'y_low_masked': y_low_signed & 0xFFFF,
            'calculation': f'({y_high_signed} << 16) | {y_low_signed & 0xFFFF}',
            'y_int': y_int_wrong,
            'y_mm': y_mm_wrong,
            'problem': f'{y_low_signed} & 0xFFFF = {y_low_signed & 0xFFFF}，負數被掩碼後又變回正數'
        })
        
        # 方法2：正確的方法 (直接32位合併)
        y_uint32 = (y_high << 16) | y_low
        if y_uint32 > 2147483647:
            y_int_correct = y_uint32 - 4294967296
        else:
            y_int_correct = y_uint32
        y_mm_correct = y_int_correct / 100.0
        
        steps.append({
            'method': '正確方法 (直接32位合併)',
            'y_uint32': y_uint32,
            'y_uint32_hex': f'0x{y_uint32:08X}',
            'calculation': f'({y_high} << 16) | {y_low}',
            'y_int': y_int_correct,
            'y_mm': y_mm_correct,
            'explanation': '直接合併無符號16位，然後轉換32位有符號'
        })
        
        return {
            'input': {'y_high': y_high, 'y_low': y_low},
            'steps': steps,
            'conclusion': f'錯誤方法得到{y_mm_wrong}mm，正確方法得到{y_mm_correct}mm'
        }


# ==================== 使用範例 ====================
def example_usage():
    """使用範例 (基於可工作的DR邏輯)"""
    # 創建CCD1高層API實例
    ccd1 = CCD1HighLevelAPI()
    
    try:
        print("=== CCD1高層API使用範例 (基於可工作的DR邏輯) ===")
        
        # 🔥 專門調試Y座標問題
        print("\n=== 調試Y座標問題 ===")
        y_debug = ccd1.debug_y_coordinate_problem()
        print(f"Y座標問題分析: {y_debug}")
        
        # 🔥 測試正確的計算方法
        print("\n=== 測試正確的計算方法 ===")
        correct_test = ccd1.test_correct_calculation()
        print(f"正確計算測試: {correct_test}")
        
        # 🔥 調試原始寄存器數據
        print("\n=== 調試原始寄存器數據 ===")
        debug_info = ccd1.debug_raw_registers()
        print(f"原始寄存器數據: {debug_info}")
        
        # 檢查系統狀態
        status = ccd1.get_system_status()
        print(f"\n系統狀態: {status}")
        
        # 檢查佇列狀態
        queue_status = ccd1.get_queue_status()
        print(f"佇列狀態: {queue_status}")
        
        # 手動執行檢測 (可選)
        print("\n手動執行拍照+檢測...")
        success = ccd1.capture_and_detect()
        print(f"檢測結果: {'成功' if success else '失敗'}")
        
        # 檢測後再次調試寄存器
        if success:
            print("\n=== 檢測後調試寄存器數據 ===")
            debug_info_after = ccd1.debug_raw_registers()
            print(f"檢測後寄存器數據: {debug_info_after}")
        
        # 逐一獲取圓心座標
        print("\n逐一獲取圓心座標:")
        for i in range(3):  # 嘗試獲取3個座標
            coord = ccd1.get_next_circle_world_coord()
            if coord:
                print(f"圓心{coord.id}: 世界座標=({coord.world_x:.2f}, {coord.world_y:.2f})mm, "
                      f"像素座標=({coord.pixel_x}, {coord.pixel_y}), 半徑={coord.radius}")
                # 檢查合理性
                if abs(coord.world_x) > 1000 or abs(coord.world_y) > 1000:
                    print(f"  ⚠️ 警告：座標值異常大，可能存在問題")
                else:
                    print(f"  ✅ 座標值在合理範圍內")
            else:
                print(f"第{i+1}次獲取座標失敗")
                break
        
        # 檢查最終佇列狀態
        final_status = ccd1.get_queue_status()
        print(f"\n最終佇列狀態: {final_status}")
        
    finally:
        # 清理資源
        ccd1.disconnect()
        print("\nCCD1連接已斷開")


if __name__ == "__main__":
    example_usage()