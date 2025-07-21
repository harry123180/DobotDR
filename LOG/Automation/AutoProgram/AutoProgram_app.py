#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AutoProgram_app.py - DR專案AutoProgram Web控制界面 (更新版)
提供DR專案AutoProgram協調控制、AutoFeeding狀態監控、手動操作等功能
基於Flask + SocketIO架構
支援自動程序啟用/停用控制
檢測類型: DR_F/STACK二分類，流程配置: Flow1+Flow2
"""

import os
import time
import json
import threading
import logging
from logging.handlers import RotatingFileHandler
from typing import Dict, Any, Optional
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException, ConnectionException

def setup_logging(module_name: str):
    """統一設置logging配置"""
    # 日誌目錄：執行檔同層目錄下的logs資料夾
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
    os.makedirs(log_dir, exist_ok=True)
    
    # 格式化器
    formatter = logging.Formatter(
        '%(asctime)s [%(levelname)s] %(name)s:%(funcName)s:%(lineno)d - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # 文件處理器 (輪替日誌，保存一週)
    file_handler = RotatingFileHandler(
        os.path.join(log_dir, f'{module_name}.log'),
        maxBytes=10*1024*1024,  # 10MB
        backupCount=7,          # 保留7個檔案
        encoding='utf-8'
    )
    file_handler.setFormatter(formatter)
    
    # 控制台處理器
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    
    # 配置logger
    logger = logging.getLogger(module_name)
    logger.setLevel(logging.DEBUG)
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger

# 初始化logging
logger = setup_logging('DR_AutoProgram_Web')

# 創建Flask應用
app = Flask(__name__)
app.config['SECRET_KEY'] = 'dr_autoprogram_v2.0'
socketio = SocketIO(app, cors_allowed_origins="*")

class DrAutoProgramWebController:
    """DR專案AutoProgram Web控制器 (更新版)"""
    
    def __init__(self, modbus_host="127.0.0.1", modbus_port=502):
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # 狀態監控執行緒
        self.monitor_thread = None
        self.monitoring = False
        
        # DR專案寄存器地址映射 (更新版)
        self.REGISTERS = {
            # AutoProgram狀態 (1300-1319)
            'SYSTEM_STATUS': 1300,              # 系統狀態
            'PREPARE_DONE': 1301,               # prepare_done狀態
            'AUTO_PROGRAM_ENABLED': 1302,       # 自動程序啟用狀態
            'AF_DR_F_AVAILABLE': 1303,          # AutoFeeding DR_F狀態
            'FLOW2_COMPLETE_STATUS': 1304,      # Flow2完成狀態 (DR專案用Flow2)
            'COORDINATION_CYCLE_COUNT': 1305,   # 協調週期計數
            'FLOW1_TRIGGER_COUNT': 1306,        # Flow1觸發次數
            'FLOW2_COMPLETE_COUNT': 1307,       # Flow2完成次數 (DR專案用Flow2)
            'DR_F_TAKEN_COUNT': 1308,           # DR_F取得次數
            'ERROR_CODE': 1309,                 # 錯誤代碼
            
            # AutoProgram控制 (1320-1339)
            'SYSTEM_CONTROL': 1320,             # 系統控制
            'AUTO_PROGRAM_CONTROL': 1321,       # 自動程序啟用控制
            'ERROR_CLEAR': 1322,                # 錯誤清除
            'FORCE_RESET': 1323,                # 強制重置
            
            # AutoFeeding座標 (1340-1359)
            'AF_TARGET_X_HIGH': 1340,           # 目標座標X高位
            'AF_TARGET_X_LOW': 1341,            # 目標座標X低位
            'AF_TARGET_Y_HIGH': 1342,           # 目標座標Y高位
            'AF_TARGET_Y_LOW': 1343,            # 目標座標Y低位
            
            # AutoFeeding模組直接讀取 (940-945)
            'AF_MODULE_STATUS': 900,            # AutoFeeding模組狀態
            'AF_DR_F_AVAILABLE_DIRECT': 940,    # DR_F可用標誌(直讀)
            'AF_TARGET_X_HIGH_DIRECT': 941,     # 目標座標X高位(直讀)
            'AF_TARGET_X_LOW_DIRECT': 942,      # 目標座標X低位(直讀)
            'AF_TARGET_Y_HIGH_DIRECT': 943,     # 目標座標Y高位(直讀)
            'AF_TARGET_Y_LOW_DIRECT': 944,      # 目標座標Y低位(直讀)
            'AF_COORDS_TAKEN': 945,             # 座標已讀取標誌
            
            # Dobot M1Pro (1200-1299)
            'DOBOT_MOTION_STATUS': 1200,        # 運動狀態寄存器
            'DOBOT_CURRENT_FLOW': 1201,         # 當前運動Flow
            'DOBOT_MOTION_PROGRESS': 1202,      # 運動進度
            'DOBOT_FLOW1_COMPLETE': 1204,       # Flow1完成狀態
            'DOBOT_FLOW2_COMPLETE': 1205,       # Flow2完成狀態 (DR專案用Flow2)
            'DOBOT_FLOW1_CONTROL': 1240,        # Flow1控制
            'DOBOT_FLOW2_CONTROL': 1241,        # Flow2控制 (DR專案用Flow2)
            
            # CCD1檢測結果 (DR專案二分類)
            'CCD1_STATUS': 201,                 # CCD1狀態
            'DR_F_COUNT': 240,                  # DR_F數量
            'STACK_COUNT': 242,                 # STACK數量
            'TOTAL_DETECTIONS': 243,            # 總檢測數量
            
            # VP狀態
            'VP_STATUS': 300,                   # VP模組狀態
            'VP_DEVICE_CONNECTION': 301,        # VP設備連接
        }
        
        logger.info("DR專案AutoProgram Web控制器初始化完成 (更新版)")
        logger.info("檢測類型: DR_F/STACK二分類")
        logger.info("流程配置: Flow1+Flow2")
    
    def connect_modbus(self) -> bool:
        """連接Modbus服務器"""
        try:
            logger.info(f"正在連接Modbus服務器 {self.modbus_host}:{self.modbus_port}")
            
            if self.modbus_client:
                self.modbus_client.close()
                logger.debug("已關閉現有Modbus連接")
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3.0
            )
            
            self.connected = self.modbus_client.connect()
            
            if self.connected:
                logger.info(f"Modbus連接建立成功: {self.modbus_host}:{self.modbus_port}")
                # 啟動狀態監控
                self.start_monitoring()
            else:
                logger.warning(f"Modbus連接失敗: {self.modbus_host}:{self.modbus_port}")
            
            return self.connected
            
        except Exception as e:
            logger.error(f"Modbus連接異常: {e}", exc_info=True)
            self.connected = False
            return False
    
    def disconnect_modbus(self):
        """斷開Modbus連接"""
        logger.info("開始斷開Modbus連接")
        self.stop_monitoring()
        
        if self.modbus_client and self.connected:
            try:
                self.modbus_client.close()
                self.connected = False
                logger.info("Modbus連接已成功斷開")
            except Exception as e:
                logger.error(f"斷開Modbus連接時發生異常: {e}", exc_info=True)
    
    def read_register(self, register_name: str) -> Optional[int]:
        """讀取寄存器"""
        if not self.connected or register_name not in self.REGISTERS:
            if not self.connected:
                logger.debug(f"無法讀取寄存器 {register_name}: Modbus未連接")
            else:
                logger.warning(f"無效的寄存器名稱: {register_name}")
            return None
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.read_holding_registers(address, count=1, slave=1)
            
            if not result.isError():
                value = result.registers[0]
                logger.debug(f"讀取寄存器 {register_name}({address}): {value}")
                return value
            else:
                logger.warning(f"讀取寄存器 {register_name}({address}) 失敗: {result}")
                return None
            
        except Exception as e:
            logger.error(f"讀取寄存器 {register_name} 異常: {e}", exc_info=True)
            return None
    
    def write_register(self, register_name: str, value: int) -> bool:
        """寫入寄存器"""
        if not self.connected or register_name not in self.REGISTERS:
            if not self.connected:
                logger.warning(f"無法寫入寄存器 {register_name}: Modbus未連接")
            else:
                logger.warning(f"無效的寄存器名稱: {register_name}")
            return False
        
        try:
            address = self.REGISTERS[register_name]
            logger.debug(f"寫入寄存器 {register_name}({address}): {value}")
            result = self.modbus_client.write_register(address, value, slave=1)
            
            success = not result.isError()
            if success:
                logger.debug(f"寫入寄存器 {register_name}({address})={value} 成功")
            else:
                logger.warning(f"寫入寄存器 {register_name}({address})={value} 失敗: {result}")
            
            return success
            
        except Exception as e:
            logger.error(f"寫入寄存器 {register_name}={value} 異常: {e}", exc_info=True)
            return False
    
    def read_32bit_coordinate(self, high_reg: str, low_reg: str) -> float:
        """讀取32位座標"""
        high_val = self.read_register(high_reg) or 0
        low_val = self.read_register(low_reg) or 0
        
        # 合併32位值
        combined = (high_val << 16) + low_val
        
        # 處理補碼(負數)
        if combined >= 2147483648:  # 2^31
            combined = combined - 4294967296  # 2^32
        
        # 轉換為毫米(除以100)
        coordinate = combined / 100.0
        logger.debug(f"32位座標轉換 {high_reg}({high_val})+{low_reg}({low_val}) = {coordinate:.2f}mm")
        
        return coordinate
    
    def get_system_status(self) -> Dict[str, Any]:
        """獲取系統狀態"""
        logger.debug("開始獲取系統狀態")
        
        status = {
            # 連接狀態
            'connected': self.connected,
            'modbus_host': self.modbus_host,
            'modbus_port': self.modbus_port,
            
            # DR專案資訊
            'project_name': 'DR',
            'detection_types': ['DR_F', 'STACK'],
            'flow_config': 'Flow1+Flow2',
            
            # AutoProgram狀態
            'system_status': self.read_register('SYSTEM_STATUS') or 0,
            'prepare_done': bool(self.read_register('PREPARE_DONE')),
            'auto_program_enabled': bool(self.read_register('AUTO_PROGRAM_ENABLED')),
            'af_dr_f_available': bool(self.read_register('AF_DR_F_AVAILABLE')),
            'flow2_complete_status': bool(self.read_register('FLOW2_COMPLETE_STATUS')),  # DR專案用Flow2
            'coordination_cycle_count': self.read_register('COORDINATION_CYCLE_COUNT') or 0,
            'flow1_trigger_count': self.read_register('FLOW1_TRIGGER_COUNT') or 0,
            'flow2_complete_count': self.read_register('FLOW2_COMPLETE_COUNT') or 0,  # DR專案用Flow2
            'dr_f_taken_count': self.read_register('DR_F_TAKEN_COUNT') or 0,
            'error_code': self.read_register('ERROR_CODE') or 0,
            
            # AutoFeeding模組狀態(直讀)
            'af_module_status': self.read_register('AF_MODULE_STATUS') or 0,
            'af_dr_f_available_direct': bool(self.read_register('AF_DR_F_AVAILABLE_DIRECT')),
            'af_coords_taken': bool(self.read_register('AF_COORDS_TAKEN')),
            
            # Dobot M1Pro狀態
            'dobot_motion_status': self.read_register('DOBOT_MOTION_STATUS') or 0,
            'dobot_current_flow': self.read_register('DOBOT_CURRENT_FLOW') or 0,
            'dobot_motion_progress': self.read_register('DOBOT_MOTION_PROGRESS') or 0,
            'dobot_flow1_complete': bool(self.read_register('DOBOT_FLOW1_COMPLETE')),
            'dobot_flow2_complete': bool(self.read_register('DOBOT_FLOW2_COMPLETE')),  # DR專案用Flow2
            
            # CCD1檢測結果 (DR專案二分類)
            'ccd1_status': self.read_register('CCD1_STATUS') or 0,
            'dr_f_count': self.read_register('DR_F_COUNT') or 0,
            'stack_count': self.read_register('STACK_COUNT') or 0,
            'total_detections': self.read_register('TOTAL_DETECTIONS') or 0,
            
            # VP狀態
            'vp_status': self.read_register('VP_STATUS') or 0,
            'vp_device_connection': bool(self.read_register('VP_DEVICE_CONNECTION')),
            
            # 目標座標(來自AutoProgram複製)
            'target_x': self.read_32bit_coordinate('AF_TARGET_X_HIGH', 'AF_TARGET_X_LOW'),
            'target_y': self.read_32bit_coordinate('AF_TARGET_Y_HIGH', 'AF_TARGET_Y_LOW'),
            
            # 目標座標(直接來自AutoFeeding)
            'target_x_direct': self.read_32bit_coordinate('AF_TARGET_X_HIGH_DIRECT', 'AF_TARGET_X_LOW_DIRECT'),
            'target_y_direct': self.read_32bit_coordinate('AF_TARGET_Y_HIGH_DIRECT', 'AF_TARGET_Y_LOW_DIRECT'),
            
            # 時間戳
            'timestamp': time.strftime("%Y-%m-%d %H:%M:%S")
        }
        
        # 判斷系統運行狀態
        status['system_running'] = self._get_system_running_status(status)
        status['autofeeding_process_status'] = self._get_autofeeding_process_status(status)
        
        logger.debug("系統狀態獲取完成")
        return status
    
    def _get_system_running_status(self, status: Dict) -> str:
        """判斷系統運行狀態"""
        system_status = status['system_status']
        auto_enabled = status['auto_program_enabled']
        
        if system_status == 0:
            status_text = "系統停止"
        elif system_status == 1:
            if auto_enabled:
                status_text = "運行中 (自動程序啟用)"
            else:
                status_text = "運行中 (自動程序停用)"
        elif system_status == 2:
            status_text = "Flow1已觸發"
        elif system_status == 3:
            status_text = "Flow2已完成"  # DR專案用Flow2
        elif system_status == 4:
            status_text = "錯誤"
        else:
            status_text = f"未知狀態({system_status})"
        
        logger.debug(f"系統運行狀態判斷: {status_text}")
        return status_text
    
    def _get_autofeeding_process_status(self, status: Dict) -> str:
        """判斷AutoFeeding流程狀態"""
        af_status = status['af_module_status']
        dr_f_available = status['af_dr_f_available_direct']
        coords_taken = status['af_coords_taken']
        
        status_text = ""
        if af_status == 0:
            status_text = "AutoFeeding模組停止"
        elif af_status == 1:
            status_text = "AutoFeeding模組運行中"
        elif af_status == 2:
            status_text = "AutoFeeding模組暫停"
        elif af_status == 3:
            status_text = "CCD1檢測中"
        elif af_status == 4:
            status_text = "VP震動中"
        elif af_status == 5:
            status_text = "AutoFeeding模組錯誤"
        else:
            status_text = f"未知狀態({af_status})"
        
        # 添加DR_F狀態
        if dr_f_available:
            if coords_taken:
                status_text += " (DR_F已被讀取)"
            else:
                status_text += " (DR_F可用)"
        else:
            status_text += " (無DR_F)"
        
        logger.debug(f"AutoFeeding流程狀態判斷: {status_text}")
        return status_text
    
    def start_monitoring(self):
        """啟動狀態監控"""
        if self.monitoring:
            logger.warning("狀態監控已在運行中，跳過啟動")
            return
        
        logger.info("啟動狀態監控執行緒")
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        logger.info("狀態監控執行緒已啟動")
    
    def stop_monitoring(self):
        """停止狀態監控"""
        if self.monitoring:
            logger.info("停止狀態監控執行緒")
            self.monitoring = False
            if self.monitor_thread and self.monitor_thread.is_alive():
                self.monitor_thread.join(timeout=2.0)
                if self.monitor_thread.is_alive():
                    logger.warning("狀態監控執行緒未能在2秒內停止")
                else:
                    logger.info("狀態監控執行緒已停止")
    
    def _monitor_loop(self):
        """狀態監控循環"""
        logger.info("狀態監控循環開始")
        cycle_count = 0
        
        while self.monitoring and self.connected:
            try:
                # 每100個週期輸出一次debug日誌，避免日誌過多
                cycle_count += 1
                if cycle_count % 100 == 0:
                    logger.debug(f"狀態監控週期計數: {cycle_count}")
                
                # 獲取系統狀態
                status = self.get_system_status()
                
                # 通過SocketIO發送狀態更新
                socketio.emit('status_update', status)
                
                # 檢查錯誤狀態
                if status.get('error_code', 0) != 0:
                    logger.warning(f"檢測到系統錯誤代碼: {status['error_code']}")
                
                # 2秒間隔
                time.sleep(2.0)
                
            except Exception as e:
                logger.error(f"狀態監控循環異常: {e}", exc_info=True)
                time.sleep(5.0)
        
        logger.info("狀態監控循環結束")

# 創建全局控制器實例
controller = DrAutoProgramWebController()

# ==================== Flask路由 ====================

@app.route('/')
def index():
    """主頁面"""
    logger.info("收到主頁面請求")
    return render_template('AutoProgram.html')

@app.route('/test')
def test():
    logger.info("收到測試頁面請求")
    return "DR專案AutoProgram Web Server is running! (更新版)"

@app.route('/api/connect', methods=['POST'])
def connect_modbus():
    """連接Modbus服務器"""
    try:
        logger.info("收到Modbus連接請求")
        success = controller.connect_modbus()
        
        response_data = {
            'success': success,
            'message': '連接成功' if success else '連接失敗',
            'status': controller.get_system_status() if success else None
        }
        
        if success:
            logger.info("Modbus連接請求處理成功")
        else:
            logger.warning("Modbus連接請求處理失敗")
        
        return jsonify(response_data)
        
    except Exception as e:
        logger.error(f"處理Modbus連接請求時發生異常: {e}", exc_info=True)
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/disconnect', methods=['POST'])
def disconnect_modbus():
    """斷開Modbus連接"""
    try:
        logger.info("收到Modbus斷開請求")
        controller.disconnect_modbus()
        
        logger.info("Modbus斷開請求處理成功")
        return jsonify({
            'success': True,
            'message': '連接已斷開'
        })
        
    except Exception as e:
        logger.error(f"處理Modbus斷開請求時發生異常: {e}", exc_info=True)
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/status', methods=['GET'])
def get_status():
    """獲取系統狀態"""
    try:
        logger.debug("收到系統狀態查詢請求")
        status = controller.get_system_status()
        return jsonify({'success': True, 'status': status})
        
    except Exception as e:
        logger.error(f"處理系統狀態查詢時發生異常: {e}", exc_info=True)
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/control/system', methods=['POST'])
def control_system():
    """控制AutoProgram系統"""
    try:
        data = request.get_json()
        action = data.get('action')  # 'start' or 'stop'
        
        logger.info(f"收到系統控制請求: {action}")
        
        if action == 'start':
            success = controller.write_register('SYSTEM_CONTROL', 1)
            logger.info(f"寫入寄存器SYSTEM_CONTROL=1，結果: {success}")
            message = 'DR專案AutoProgram系統已啟動 (1320=1)' if success else 'DR專案AutoProgram系統啟動失敗'
        elif action == 'stop':
            success = controller.write_register('SYSTEM_CONTROL', 0)
            logger.info(f"寫入寄存器SYSTEM_CONTROL=0，結果: {success}")
            message = 'DR專案AutoProgram系統已停止 (1320=0)' if success else 'DR專案AutoProgram系統停止失敗'
        else:
            logger.warning(f"收到無效的系統控制操作: {action}")
            return jsonify({'success': False, 'message': '無效的操作'})
        
        # 驗證寫入結果
        verify_value = controller.read_register('SYSTEM_CONTROL')
        logger.debug(f"驗證讀取寄存器SYSTEM_CONTROL值: {verify_value}")
        
        if success:
            logger.info(f"系統控制操作 {action} 執行成功")
        else:
            logger.error(f"系統控制操作 {action} 執行失敗")
        
        return jsonify({
            'success': success,
            'message': message,
            'debug_info': f"寫入結果:{success}, 驗證值:{verify_value}"
        })
        
    except Exception as e:
        logger.error(f"系統控制操作異常: {e}", exc_info=True)
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/control/auto_program', methods=['POST'])
def control_auto_program():
    """控制自動程序啟用/停用"""
    try:
        data = request.get_json()
        action = data.get('action')  # 'enable' or 'disable'
        
        logger.info(f"收到自動程序控制請求: {action}")
        
        if action == 'enable':
            success = controller.write_register('AUTO_PROGRAM_CONTROL', 1)
            logger.info(f"寫入寄存器AUTO_PROGRAM_CONTROL=1，結果: {success}")
            message = '自動程序已啟用 (1321=1)' if success else '自動程序啟用失敗'
        elif action == 'disable':
            success = controller.write_register('AUTO_PROGRAM_CONTROL', 0)
            logger.info(f"寫入寄存器AUTO_PROGRAM_CONTROL=0，結果: {success}")
            message = '自動程序已停用 (1321=0)' if success else '自動程序停用失敗'
        else:
            logger.warning(f"收到無效的自動程序控制操作: {action}")
            return jsonify({'success': False, 'message': '無效的操作'})
        
        # 驗證寫入結果
        verify_value = controller.read_register('AUTO_PROGRAM_CONTROL')
        logger.debug(f"驗證讀取寄存器AUTO_PROGRAM_CONTROL值: {verify_value}")
        
        if success:
            logger.info(f"自動程序控制操作 {action} 執行成功")
        else:
            logger.error(f"自動程序控制操作 {action} 執行失敗")
        
        return jsonify({
            'success': success,
            'message': message,
            'debug_info': f"寫入結果:{success}, 驗證值:{verify_value}"
        })
        
    except Exception as e:
        logger.error(f"自動程序控制操作異常: {e}", exc_info=True)
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/control/dobot_flow1', methods=['POST'])
def control_dobot_flow1():
    """直接控制Dobot Flow1"""
    try:
        data = request.get_json()
        action = data.get('action')  # 'trigger', 'clear'
        
        logger.info(f"收到Dobot Flow1控制請求: {action}")
        
        if action == 'trigger':
            success = controller.write_register('DOBOT_FLOW1_CONTROL', 1)
            message = 'Dobot Flow1已觸發 (1240=1)' if success else 'Dobot Flow1觸發失敗'
        elif action == 'clear':
            success = controller.write_register('DOBOT_FLOW1_CONTROL', 0)
            message = 'Dobot Flow1控制已清除 (1240=0)' if success else 'Dobot Flow1控制清除失敗'
        else:
            logger.warning(f"收到無效的Dobot Flow1控制操作: {action}")
            return jsonify({'success': False, 'message': '無效的操作'})
        
        if success:
            logger.info(f"Dobot Flow1控制操作 {action} 執行成功")
        else:
            logger.error(f"Dobot Flow1控制操作 {action} 執行失敗")
        
        return jsonify({
            'success': success,
            'message': message
        })
        
    except Exception as e:
        logger.error(f"Dobot Flow1控制操作異常: {e}", exc_info=True)
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/control/dobot_flow2', methods=['POST'])
def control_dobot_flow2():
    """直接控制Dobot Flow2 (DR專案用Flow2)"""
    try:
        data = request.get_json()
        action = data.get('action')  # 'trigger', 'clear'
        
        logger.info(f"收到Dobot Flow2控制請求: {action}")
        
        if action == 'trigger':
            success = controller.write_register('DOBOT_FLOW2_CONTROL', 1)
            message = 'Dobot Flow2已觸發 (1241=1)' if success else 'Dobot Flow2觸發失敗'
        elif action == 'clear':
            success = controller.write_register('DOBOT_FLOW2_CONTROL', 0)
            message = 'Dobot Flow2控制已清除 (1241=0)' if success else 'Dobot Flow2控制清除失敗'
        else:
            logger.warning(f"收到無效的Dobot Flow2控制操作: {action}")
            return jsonify({'success': False, 'message': '無效的操作'})
        
        if success:
            logger.info(f"Dobot Flow2控制操作 {action} 執行成功")
        else:
            logger.error(f"Dobot Flow2控制操作 {action} 執行失敗")
        
        return jsonify({
            'success': success,
            'message': message
        })
        
    except Exception as e:
        logger.error(f"Dobot Flow2控制操作異常: {e}", exc_info=True)
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/control/flow_complete', methods=['POST'])
def control_flow_complete():
    """清除Flow完成狀態"""
    try:
        data = request.get_json()
        action = data.get('action')  # 'clear_flow1' or 'clear_flow2'
        
        logger.info(f"收到Flow完成狀態清除請求: {action}")
        
        if action == 'clear_flow1':
            success = controller.write_register('DOBOT_FLOW1_COMPLETE', 0)
            message = 'Flow1完成狀態已清除 (1204=0)' if success else 'Flow1完成狀態清除失敗'
        elif action == 'clear_flow2':
            success = controller.write_register('DOBOT_FLOW2_COMPLETE', 0)
            message = 'Flow2完成狀態已清除 (1205=0)' if success else 'Flow2完成狀態清除失敗'  # DR專案用Flow2
        else:
            logger.warning(f"收到無效的Flow完成狀態清除操作: {action}")
            return jsonify({'success': False, 'message': '無效的操作'})
        
        if success:
            logger.info(f"Flow完成狀態清除操作 {action} 執行成功")
        else:
            logger.error(f"Flow完成狀態清除操作 {action} 執行失敗")
        
        return jsonify({
            'success': success,
            'message': message
        })
        
    except Exception as e:
        logger.error(f"Flow完成狀態清除操作異常: {e}", exc_info=True)
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/control/auto_handshake', methods=['POST'])
def auto_handshake():
    """自動交握 - Flow1完成後自動觸發Flow2 (DR專案用Flow2)"""
    try:
        logger.info("開始執行自動交握流程")
        logMessage = []
        
        # 1. 檢查Flow1完成狀態
        flow1_complete = controller.read_register('DOBOT_FLOW1_COMPLETE')
        logMessage.append(f"檢查Flow1完成狀態: {flow1_complete}")
        logger.debug(f"Flow1完成狀態檢查結果: {flow1_complete}")
        
        if not flow1_complete:
            logger.warning("Flow1尚未完成，無法執行自動交握")
            return jsonify({
                'success': False,
                'message': 'Flow1尚未完成，無法執行自動交握'
            })
        
        logMessage.append("Flow1已完成，開始自動交握流程")
        logger.info("Flow1已完成，開始自動交握流程")
        
        # 2. 清除Flow1完成狀態
        clear_success = controller.write_register('DOBOT_FLOW1_COMPLETE', 0)
        if clear_success:
            logMessage.append("Flow1完成狀態已清除")
            logger.info("Flow1完成狀態清除成功")
        else:
            logMessage.append("Flow1完成狀態清除失敗")
            logger.error("Flow1完成狀態清除失敗")
            
        # 3. 觸發Flow2 (DR專案用Flow2)
        trigger_success = controller.write_register('DOBOT_FLOW2_CONTROL', 1)
        if trigger_success:
            logMessage.append("Flow2已觸發")
            logger.info("Flow2觸發成功")
        else:
            logMessage.append("Flow2觸發失敗")
            logger.error("Flow2觸發失敗")
            
        # 4. 等待一小段時間後清除Flow2控制狀態
        time.sleep(0.1)
        controller.write_register('DOBOT_FLOW2_CONTROL', 0)
        logMessage.append("Flow2控制狀態已清除")
        logger.debug("Flow2控制狀態已清除")
        
        success = clear_success and trigger_success
        message = " | ".join(logMessage)
        
        if success:
            logger.info("自動交握流程執行成功")
        else:
            logger.warning("自動交握流程部分失敗")
        
        return jsonify({
            'success': success,
            'message': f"自動交握{'成功' if success else '部分失敗'}: {message}"
        })
        
    except Exception as e:
        logger.error(f"自動交握執行異常: {e}", exc_info=True)
        return jsonify({
            'success': False, 
            'message': f'自動交握執行失敗: {str(e)}'
        })

@app.route('/api/control/coords_taken', methods=['POST'])
def set_coords_taken():
    """設置座標已讀取標誌"""
    try:
        logger.info("收到設置座標已讀取標誌請求")
        success = controller.write_register('AF_COORDS_TAKEN', 1)
        message = '座標已讀取標誌已設置 (945=1)' if success else '座標已讀取標誌設置失敗'
        
        if success:
            logger.info("座標已讀取標誌設置成功")
        else:
            logger.error("座標已讀取標誌設置失敗")
        
        return jsonify({
            'success': success,
            'message': message
        })
        
    except Exception as e:
        logger.error(f"設置座標已讀取標誌異常: {e}", exc_info=True)
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/control/error_clear', methods=['POST'])
def error_clear():
    """清除錯誤"""
    try:
        logger.info("收到錯誤清除請求")
        success = controller.write_register('ERROR_CLEAR', 1)
        message = '錯誤已清除 (1322=1)' if success else '錯誤清除失敗'
        
        if success:
            logger.info("錯誤清除成功")
        else:
            logger.error("錯誤清除失敗")
        
        return jsonify({
            'success': success,
            'message': message
        })
        
    except Exception as e:
        logger.error(f"錯誤清除操作異常: {e}", exc_info=True)
        return jsonify({'success': False, 'message': str(e)})

# ==================== SocketIO事件處理 ====================

@socketio.on('connect')
def handle_connect():
    """客戶端連接"""
    logger.info("SocketIO客戶端已連接")
    emit('status_update', controller.get_system_status())

@socketio.on('disconnect')
def handle_disconnect():
    """客戶端斷開連接"""
    logger.info("SocketIO客戶端已斷開連接")

@socketio.on('request_status')
def handle_request_status():
    """請求狀態更新"""
    logger.debug("收到SocketIO狀態更新請求")
    emit('status_update', controller.get_system_status())

def main():
    """主函數"""
    logger.info("=" * 60)
    logger.info("DR專案AutoProgram Web控制界面啟動中... (更新版)")
    logger.info("DR專案機械臂協調控制與監控")
    logger.info("檢測類型: DR_F/STACK二分類")
    logger.info("流程配置: Flow1+Flow2")
    logger.info("新增功能: 自動程序啟用/停用控制、AutoFeeding狀態監控")
    logger.info("=" * 60)
    
    # 檢查模板文件
    template_dir = os.path.join(os.path.dirname(__file__), 'templates')
    template_file = os.path.join(template_dir, 'AutoProgram.html')
    
    if not os.path.exists(template_dir):
        os.makedirs(template_dir)
        logger.info(f"已創建模板目錄: {template_dir}")
    
    if not os.path.exists(template_file):
        logger.warning(f"模板文件不存在: {template_file}")
        logger.warning("請確保AutoProgram.html文件在templates目錄中")
    
    try:
        logger.info("Web服務器啟動中...")
        logger.info("訪問地址: http://localhost:5094")
        logger.info("功能特性:")
        logger.info("   • DR專案AutoProgram協調控制 (1300基地址)")
        logger.info("   • 自動程序啟用/停用控制 (1321)")
        logger.info("   • AutoFeeding狀態監控 (940-945)")
        logger.info("   • Dobot M1Pro Flow控制 (1240/1241)")
        logger.info("   • 自動交握控制 (Flow1→Flow2)")
        logger.info("   • 即時座標顯示")
        logger.info("   • 協調週期統計")
        logger.info("   • DR_F/STACK二分類檢測")
        logger.info("=" * 60)
        
        # 啟動Web服務器
        socketio.run(
            app,
            host='0.0.0.0',
            port=5094,  # 使用不同端口避免與CASE版本衝突
            debug=False
        )
        
    except KeyboardInterrupt:
        logger.info("收到中斷信號，正在關閉...")
    except Exception as e:
        logger.error(f"Web服務器錯誤: {e}", exc_info=True)
    finally:
        # 清理資源
        logger.info("開始清理系統資源")
        controller.disconnect_modbus()
        logger.info("DR專案Web服務器已關閉")

if __name__ == '__main__':
    main()