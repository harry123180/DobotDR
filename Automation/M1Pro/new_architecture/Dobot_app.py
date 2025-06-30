#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_app.py - Dobot新架構Flow控制Web應用
支援CCD1拍照檢測、Flow1/2/4控制、CCD3角度檢測、夾爪快速關閉
基於new_architecture的寄存器映射和交握協議
"""

import os
import json
import time
import threading
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import logging

# 導入高階API模組
try:
    from AngleHighLevel import AngleHighLevel
    from GripperHighLevel import GripperHighLevelAPI, GripperType
    from pymodbus.client import ModbusTcpClient
    API_AVAILABLE = True
    
    # 導入修正版CCD1HighLevel
    try:
        from CCD1HighLevel import CCD1HighLevelAPI
        CCD1_AVAILABLE = True
    except ImportError:
        CCD1_AVAILABLE = False
        print("⚠️ CCD1HighLevel模組未找到，將使用直接寄存器控制")
        
except ImportError as e:
    print(f"⚠️ 高階API模組導入失敗: {e}")
    API_AVAILABLE = False
    CCD1_AVAILABLE = False

# Flask應用設定
app = Flask(__name__)
app.config['SECRET_KEY'] = 'dobot_flow_control_2025'
socketio = SocketIO(app, cors_allowed_origins="*")

class DobotFlowController:
    """Dobot Flow控制器"""
    
    def __init__(self, config_file="dobot_flow_config.json"):
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_file = os.path.join(self.current_dir, config_file)
        self.config = self.load_config()
        
        # Modbus連接
        self.modbus_client = None
        self.connected = False
        
        # 高階API實例
        self.ccd1_api = None
        self.ccd3_api = None
        self.gripper_api = None
        
        # 狀態監控
        self.monitoring = False
        self.monitor_thread = None
        
        # 設置日誌
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger("DobotFlowController")
        
        # 初始化連接
        self.connect_systems()
        
    def load_config(self):
        """載入配置"""
        default_config = {
            "modbus_server": {
                "host": "127.0.0.1",
                "port": 502,
                "timeout": 3.0
            },
            "web_server": {
                "host": "0.0.0.0",
                "port": 5058,
                "debug": False
            },
            "gripper": {
                "type": "PGE",
                "enabled": True
            },
            "ui_settings": {
                "auto_refresh": True,
                "refresh_interval": 2.0
            },
            "registers": {
                "ccd1_control": 200,
                "ccd1_status": 201,
                "ccd1_count": 240,
                "ccd3_control": 800,
                "ccd3_status": 801,
                "ccd3_result": 840,
                "flow1_control": 1240,
                "flow2_control": 1241,
                "flow4_control": 448,
                "motion_status": 1200,
                "motion_flow": 1201,
                "motion_progress": 1202,
                "flow1_complete": 1204,
                "flow2_complete": 1205
            }
        }
        
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    user_config = json.load(f)
                    default_config.update(user_config)
            except Exception as e:
                self.logger.error(f"載入配置失敗: {e}")
        else:
            self.save_config(default_config)
            
        return default_config
    
    def save_config(self, config):
        """保存配置"""
        try:
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=2, ensure_ascii=False)
        except Exception as e:
            self.logger.error(f"保存配置失敗: {e}")
    
    def connect_systems(self):
        """連接各個系統"""
        try:
            # 連接Modbus服務器
            self.logger.info("正在連接Modbus服務器...")
            self.modbus_client = ModbusTcpClient(
                host=self.config["modbus_server"]["host"],
                port=self.config["modbus_server"]["port"],
                timeout=self.config["modbus_server"]["timeout"]
            )
            
            if self.modbus_client.connect():
                self.connected = True
                self.logger.info("✓ Modbus服務器連接成功")
            else:
                self.logger.error("✗ Modbus服務器連接失敗")
                return
            
            # 初始化高階API
            if API_AVAILABLE:
                # 初始化CCD1 API (如果可用)
                if CCD1_AVAILABLE:
                    self.logger.info("正在初始化CCD1 HighLevel API...")
                    self.ccd1_api = CCD1HighLevelAPI(
                        self.config["modbus_server"]["host"],
                        self.config["modbus_server"]["port"]
                    )
                    if self.ccd1_api.connected:
                        self.logger.info("✓ CCD1 HighLevel API連接成功")
                    else:
                        self.logger.warning("⚠️ CCD1 HighLevel API連接失敗")
                        self.ccd1_api = None
                
                # 初始化CCD3 API
                self.logger.info("正在初始化CCD3 API...")
                self.ccd3_api = AngleHighLevel(
                    self.config["modbus_server"]["host"],
                    self.config["modbus_server"]["port"]
                )
                if self.ccd3_api.connect():
                    self.logger.info("✓ CCD3 API連接成功")
                else:
                    self.logger.warning("⚠️ CCD3 API連接失敗")
                
                # 初始化夾爪API
                if self.config["gripper"]["enabled"]:
                    self.logger.info("正在初始化夾爪API...")
                    gripper_type = GripperType.PGE if self.config["gripper"]["type"] == "PGE" else GripperType.PGC
                    self.gripper_api = GripperHighLevelAPI(
                        gripper_type=gripper_type,
                        modbus_host=self.config["modbus_server"]["host"],
                        modbus_port=self.config["modbus_server"]["port"]
                    )
                    if self.gripper_api.connected:
                        self.logger.info(f"✓ {self.config['gripper']['type']}夾爪API連接成功")
                    else:
                        self.logger.warning(f"⚠️ {self.config['gripper']['type']}夾爪API連接失敗")
            
        except Exception as e:
            self.logger.error(f"系統連接異常: {e}")
    
    def read_register(self, address):
        """讀取寄存器"""
        if not self.connected or not self.modbus_client:
            return None
        
        try:
            result = self.modbus_client.read_holding_registers(address=address, count=1, slave=1)
            if not result.isError():
                return result.registers[0]
        except Exception as e:
            self.logger.error(f"讀取寄存器{address}失敗: {e}")
        return None
    
    def write_register(self, address, value):
        """寫入寄存器"""
        if not self.connected or not self.modbus_client:
            return False
        
        try:
            result = self.modbus_client.write_register(address=address, value=value, slave=1)
            return not result.isError()
        except Exception as e:
            self.logger.error(f"寫入寄存器{address}失敗: {e}")
            return False
    
    def get_system_status(self):
        """獲取系統狀態"""
        status = {
            "connected": self.connected,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "ccd1": {"connected": False, "ready": False, "count": 0},
            "ccd3": {"connected": False, "ready": False, "result": 0},
            "motion": {"status": 0, "current_flow": 0, "progress": 0},
            "flows": {"flow1_complete": 0, "flow2_complete": 0},
            "gripper": {"connected": False, "position": 0}
        }
        
        if self.connected:
            try:
                # CCD1狀態 (優先使用HighLevel API)
                if self.ccd1_api and self.ccd1_api.connected:
                    # 使用HighLevel API獲取狀態
                    ccd1_system_status = self.ccd1_api.get_system_status()
                    status["ccd1"]["connected"] = ccd1_system_status["connected"]
                    status["ccd1"]["ready"] = ccd1_system_status["is_status_9"]
                    
                    # 從HighLevel API獲取佇列狀態
                    queue_status = self.ccd1_api.get_queue_status()
                    status["ccd1"]["count"] = queue_status.get("last_detection_count", 0)
                else:
                    # 備用：直接讀取寄存器
                    ccd1_status = self.read_register(self.config["registers"]["ccd1_status"])
                    ccd1_count = self.read_register(self.config["registers"]["ccd1_count"])
                    if ccd1_status is not None:
                        status["ccd1"]["connected"] = True
                        status["ccd1"]["ready"] = (ccd1_status == 9)
                        status["ccd1"]["count"] = ccd1_count or 0
                
                # CCD3狀態
                ccd3_status = self.read_register(self.config["registers"]["ccd3_status"])
                ccd3_result = self.read_register(self.config["registers"]["ccd3_result"])
                if ccd3_status is not None:
                    status["ccd3"]["connected"] = True
                    status["ccd3"]["ready"] = (ccd3_status == 9)
                    status["ccd3"]["result"] = ccd3_result or 0
                
                # 運動系統狀態
                motion_status = self.read_register(self.config["registers"]["motion_status"])
                motion_flow = self.read_register(self.config["registers"]["motion_flow"])
                motion_progress = self.read_register(self.config["registers"]["motion_progress"])
                
                if motion_status is not None:
                    status["motion"]["status"] = motion_status
                    status["motion"]["current_flow"] = motion_flow or 0
                    status["motion"]["progress"] = motion_progress or 0
                
                # Flow完成狀態
                flow1_complete = self.read_register(self.config["registers"]["flow1_complete"])
                flow2_complete = self.read_register(self.config["registers"]["flow2_complete"])
                status["flows"]["flow1_complete"] = flow1_complete or 0
                status["flows"]["flow2_complete"] = flow2_complete or 0
                
                # 夾爪狀態
                if self.gripper_api and self.gripper_api.connected:
                    status["gripper"]["connected"] = True
                    position = self.gripper_api.get_current_position()
                    status["gripper"]["position"] = position or 0
                
            except Exception as e:
                self.logger.error(f"獲取系統狀態異常: {e}")
        
        return status
    
    def execute_ccd1_detection(self):
        """執行CCD1拍照檢測 - 優先使用HighLevel API，備用直接寄存器控制"""
        try:
            self.logger.info("開始執行CCD1拍照檢測...")
            
            # 優先使用CCD1 HighLevel API (如果可用)
            if self.ccd1_api and self.ccd1_api.connected:
                self.logger.info("使用CCD1 HighLevel API執行檢測...")
                
                # 檢查系統是否Ready (狀態=9)
                if not self.ccd1_api.is_ready():
                    return {"success": False, "message": "CCD1系統未準備就緒 (HighLevel API)"}
                
                # 執行拍照檢測 (自動完成完整交握)
                success = self.ccd1_api.capture_and_detect()
                
                if success:
                    # 獲取檢測結果
                    queue_status = self.ccd1_api.get_queue_status()
                    count = queue_status.get("last_detection_count", 0)
                    
                    return {
                        "success": True,
                        "message": f"CCD1檢測完成 (HighLevel API)，檢測到{count}個物件",
                        "count": count
                    }
                else:
                    return {"success": False, "message": "CCD1檢測失敗 (HighLevel API)"}
            
            # 備用：直接寄存器控制版本
            else:
                self.logger.info("使用直接寄存器控制執行檢測...")
                return self._execute_ccd1_detection_direct()
                
        except Exception as e:
            self.logger.error(f"CCD1檢測異常: {e}")
            return {"success": False, "message": f"CCD1檢測異常: {str(e)}"}
    
    def _execute_ccd1_detection_direct(self):
        """直接寄存器控制版本的CCD1檢測 (備用方案)"""
        try:
            # 步驟1: 檢查CCD1狀態寄存器是否為9
            ccd1_status = self.read_register(self.config["registers"]["ccd1_status"])
            if ccd1_status is None:
                return {"success": False, "message": "無法讀取CCD1狀態寄存器"}
            
            if ccd1_status != 9:
                return {"success": False, "message": f"CCD1系統未準備就緒 (狀態={ccd1_status}, 期望=9)"}
            
            # 步驟2: 向寄存器200發送拍照+檢測指令 (16)
            self.logger.info("向寄存器200發送拍照+檢測指令16...")
            if not self.write_register(self.config["registers"]["ccd1_control"], 16):
                return {"success": False, "message": "CCD1拍照指令發送失敗"}
            
            # 步驟3: 等待檢測完成 - 確認201=8且240非0
            self.logger.info("等待CCD1檢測完成 (期望201=8且240非0)...")
            timeout = 10.0  # 10秒超時
            start_time = time.time()
            detection_completed = False
            
            while time.time() - start_time < timeout:
                status = self.read_register(self.config["registers"]["ccd1_status"])
                count = self.read_register(self.config["registers"]["ccd1_count"])
                
                if status is not None and count is not None:
                    # 檢查Alarm
                    if status & 4:  # bit2 = Alarm
                        return {"success": False, "message": "CCD1檢測過程發生錯誤"}
                    
                    # 檢查完成條件: 201=8且240非0
                    if status == 8 and count > 0:
                        self.logger.info(f"檢測完成: 201={status}, 240={count}")
                        detection_completed = True
                        break
                
                time.sleep(0.1)  # 100ms檢查間隔
            
            if not detection_completed:
                return {"success": False, "message": "CCD1檢測超時或未檢測到物件"}
            
            # 步驟4: 讀取檢測結果
            count = self.read_register(self.config["registers"]["ccd1_count"])
            if count is None:
                count = 0
            
            # 步驟5: 向200寫入0清空控制指令
            self.logger.info("清空CCD1控制寄存器200...")
            if not self.write_register(self.config["registers"]["ccd1_control"], 0):
                return {"success": False, "message": "清空CCD1控制寄存器失敗"}
            
            # 步驟6: 確認201回到9
            self.logger.info("確認201狀態回到9...")
            timeout = 3.0  # 3秒超時
            start_time = time.time()
            status_recovered = False
            
            while time.time() - start_time < timeout:
                status = self.read_register(self.config["registers"]["ccd1_status"])
                if status == 9:
                    self.logger.info("201狀態已回到9")
                    status_recovered = True
                    break
                time.sleep(0.1)
            
            if not status_recovered:
                self.logger.warning("201狀態未回到9，但檢測已完成")
            
            # 步驟7: 向240寫入0清空檢測數量
            self.logger.info("清空CCD1檢測數量寄存器240...")
            if not self.write_register(self.config["registers"]["ccd1_count"], 0):
                self.logger.warning("清空CCD1檢測數量寄存器失敗")
            
            return {
                "success": True,
                "message": f"CCD1檢測完成 (直接控制)，檢測到{count}個物件", 
                "count": count
            }
                
        except Exception as e:
            self.logger.error(f"CCD1直接控制檢測異常: {e}")
            return {"success": False, "message": f"CCD1直接控制檢測異常: {str(e)}"}
    
    def execute_ccd3_detection(self):
        """執行CCD3角度檢測"""
        try:
            self.logger.info("開始執行CCD3角度檢測...")
            
            if not self.ccd3_api:
                return {"success": False, "message": "CCD3 API未初始化"}
            
            # 檢查CCD3系統是否Ready
            if not self.ccd3_api.is_ccd3_ready():
                return {"success": False, "message": "CCD3系統未準備就緒"}
            
            # 執行角度檢測
            result = self.ccd3_api.detect_angle(detection_mode=1)  # DR模式
            
            if result.result.value == "SUCCESS":
                # 清空控制寄存器
                self.write_register(self.config["registers"]["ccd3_control"], 0)
                self.write_register(self.config["registers"]["ccd3_result"], 0)
                
                return {
                    "success": True,
                    "message": "CCD3角度檢測完成",
                    "angle": result.target_angle,
                    "center": result.detected_center,
                    "area": result.contour_area
                }
            else:
                return {
                    "success": False, 
                    "message": f"CCD3檢測失敗: {result.message}"
                }
                
        except Exception as e:
            self.logger.error(f"CCD3檢測異常: {e}")
            return {"success": False, "message": f"CCD3檢測異常: {str(e)}"}
    
    def execute_flow_control(self, flow_number):
        """執行Flow控制"""
        try:
            flow_names = {1: "VP視覺取料", 2: "組立料件", 4: "直振投料"}
            flow_name = flow_names.get(flow_number, f"Flow{flow_number}")
            
            self.logger.info(f"開始執行{flow_name} (Flow{flow_number})...")
            
            # 獲取控制寄存器地址
            if flow_number == 1:
                control_reg = self.config["registers"]["flow1_control"]
            elif flow_number == 2:
                control_reg = self.config["registers"]["flow2_control"] 
            elif flow_number == 4:
                control_reg = self.config["registers"]["flow4_control"]
            else:
                return {"success": False, "message": f"不支援的Flow: {flow_number}"}
            
            # 寫入啟動指令
            if not self.write_register(control_reg, 1):
                return {"success": False, "message": f"{flow_name}啟動指令發送失敗"}
            
            return {"success": True, "message": f"{flow_name}已啟動"}
            
        except Exception as e:
            self.logger.error(f"Flow{flow_number}執行異常: {e}")
            return {"success": False, "message": f"Flow{flow_number}執行異常: {str(e)}"}
    
    def clear_flow_control(self, flow_number):
        """清空Flow控制寄存器"""
        try:
            # 獲取控制寄存器地址
            if flow_number == 1:
                control_reg = self.config["registers"]["flow1_control"]
            elif flow_number == 2:
                control_reg = self.config["registers"]["flow2_control"]
            elif flow_number == 4:
                control_reg = self.config["registers"]["flow4_control"]
            else:
                return {"success": False, "message": f"不支援的Flow: {flow_number}"}
            
            # 清空控制寄存器
            if self.write_register(control_reg, 0):
                return {"success": True, "message": f"Flow{flow_number}控制寄存器已清空"}
            else:
                return {"success": False, "message": f"Flow{flow_number}控制寄存器清空失敗"}
                
        except Exception as e:
            self.logger.error(f"清空Flow{flow_number}控制寄存器異常: {e}")
            return {"success": False, "message": f"清空異常: {str(e)}"}
    
    def clear_ccd1_control(self):
        """清除CCD1控制寄存器"""
        try:
            self.logger.info("清除CCD1控制寄存器...")
            
            # 直接向寄存器200寫入0
            if self.write_register(self.config["registers"]["ccd1_control"], 0):
                return {"success": True, "message": "CCD1控制寄存器已清除 (200=0)"}
            else:
                return {"success": False, "message": "CCD1控制寄存器清除失敗"}
                
        except Exception as e:
            self.logger.error(f"清除CCD1控制寄存器異常: {e}")
            return {"success": False, "message": f"清除異常: {str(e)}"}
        """夾爪快速關閉"""
        try:
            if not self.gripper_api or not self.gripper_api.connected:
                return {"success": False, "message": "夾爪API未連接"}
            
            self.logger.info("執行夾爪快速關閉...")
            success = self.gripper_api.quick_close()
            
            if success:
                return {"success": True, "message": "夾爪快速關閉指令已發送"}
            else:
                return {"success": False, "message": "夾爪快速關閉失敗"}
                
        except Exception as e:
            self.logger.error(f"夾爪快速關閉異常: {e}")
            return {"success": False, "message": f"夾爪快速關閉異常: {str(e)}"}

# 全域控制器實例
controller = DobotFlowController()

@app.route('/')
def index():
    """主頁面"""
    return render_template('DobotFlowUI.html')

@socketio.on('connect')
def handle_connect():
    """WebSocket連接事件"""
    print("客戶端已連接")
    emit('status_update', controller.get_system_status())

@socketio.on('disconnect')
def handle_disconnect():
    """WebSocket斷開事件"""
    print("客戶端已斷開")

@socketio.on('get_status')
def handle_get_status():
    """獲取系統狀態"""
    emit('status_update', controller.get_system_status())

@socketio.on('ccd1_detection')
def handle_ccd1_detection():
    """CCD1拍照檢測"""
    result = controller.execute_ccd1_detection()
    emit('ccd1_result', result)

@socketio.on('ccd1_clear_control')
def handle_ccd1_clear_control():
    """CCD1清除控制"""
    result = controller.clear_ccd1_control()
    emit('ccd1_clear_result', result)

@socketio.on('ccd3_detection')
def handle_ccd3_detection():
    """CCD3角度檢測"""
    result = controller.execute_ccd3_detection()
    emit('ccd3_result', result)

@socketio.on('flow_control')
def handle_flow_control(data):
    """Flow控制"""
    flow_number = data.get('flow_number')
    action = data.get('action')  # 'start' or 'clear'
    
    if action == 'start':
        result = controller.execute_flow_control(flow_number)
    elif action == 'clear':
        result = controller.clear_flow_control(flow_number)
    else:
        result = {"success": False, "message": "未知的動作"}
    
    emit('flow_result', {"flow_number": flow_number, "action": action, **result})

@socketio.on('gripper_quick_close')
def handle_gripper_quick_close():
    """夾爪快速關閉"""
    result = controller.gripper_quick_close()
    emit('gripper_result', result)

def status_monitor():
    """狀態監控執行緒"""
    while controller.monitoring:
        try:
            status = controller.get_system_status()
            socketio.emit('status_update', status)
            time.sleep(controller.config["ui_settings"]["refresh_interval"])
        except Exception as e:
            controller.logger.error(f"狀態監控錯誤: {e}")
            time.sleep(1)

def start_monitoring():
    """啟動狀態監控"""
    if not controller.monitoring and controller.config["ui_settings"]["auto_refresh"]:
        controller.monitoring = True
        controller.monitor_thread = threading.Thread(target=status_monitor, daemon=True)
        controller.monitor_thread.start()

def run_app():
    """運行Web應用"""
    web_config = controller.config["web_server"]
    
    print(f"Dobot Flow控制器Web應用啟動中...")
    print(f"Web服務器: http://{web_config['host']}:{web_config['port']}")
    print(f"Modbus服務器: {controller.config['modbus_server']['host']}:{controller.config['modbus_server']['port']}")
    print(f"支援功能: CCD1檢測, CCD3角度檢測, Flow1/2/4控制, 夾爪快速關閉")
    
    # 啟動狀態監控
    start_monitoring()
    
    # 運行Flask應用
    socketio.run(
        app,
        host=web_config["host"],
        port=web_config["port"],
        debug=web_config["debug"]
    )

if __name__ == "__main__":
    try:
        run_app()
    except KeyboardInterrupt:
        print("\n收到停止信號...")
        controller.monitoring = False
    except Exception as e:
        print(f"應用錯誤: {e}")
        controller.monitoring = False