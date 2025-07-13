#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
start.py - DobotDR系統模組啟動器 (Flask Web版)
依序啟動所有模組並提供Web介面控制和監控
"""

import subprocess
import time
import threading
import os
import signal
import sys
from datetime import datetime
from queue import Queue, Empty
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import json

class ModuleLauncher:
    def __init__(self):
        self.processes = {}
        self.output_queues = {}
        self.module_logs = {}
        self.conda_env = "ROBOT"
        self.running = True
        
        # 模組配置 (名稱, 路徑, 延遲秒數)
        self.modules = {
            "modbus": {
                "name": "ModbusTCP Server",
                "path": r"C:\Users\user\Documents\GitHub\DobotDR\ModbusServer\TCPServer.py",
                "delay": 0,
                "required": True,
                "enabled": False,
                "status": "停止"
            },
            "vp": {
                "name": "VP震動盤",
                "path": r"C:\Users\user\Documents\GitHub\DobotDR\Automation\VP\VP_main.py",
                "delay": 2,
                "required": False,
                "enabled": False,
                "status": "停止"
            },
            "led": {
                "name": "LED控制器",
                "path": r"C:\Users\user\Documents\GitHub\DobotDR\Automation\light\LED_main.py",
                "delay": 1,
                "required": False,
                "enabled": False,
                "status": "停止"
            },
            "gripper": {
                "name": "Gripper夾爪",
                "path": r"C:\Users\user\Documents\GitHub\DobotDR\Automation\Gripper\Gripper.py",
                "delay": 1,
                "required": False,
                "enabled": False,
                "status": "停止"
            },
            "ccd1": {
                "name": "CCD1視覺",
                "path": r"C:\Users\user\Documents\GitHub\DobotDR\Automation\CCD1\CCD1VisionCodeYOLO.py",
                "delay": 1,
                "required": False,
                "enabled": False,
                "status": "停止"
            },
            "ccd3": {
                "name": "CCD3角度",
                "path": r"C:\Users\user\Documents\GitHub\DobotDR\Automation\CCD3\CCD3_main_app.py",
                "delay": 1,
                "required": False,
                "enabled": False,
                "status": "停止"
            }
        }
        
        # 初始化日誌
        for module_id in self.modules:
            self.module_logs[module_id] = []
    
    def get_conda_python_path(self):
        """獲取conda環境的python路徑"""
        conda_paths = [
            f"C:\\Users\\{os.getenv('USERNAME')}\\anaconda3\\envs\\{self.conda_env}\\python.exe",
            f"C:\\Users\\{os.getenv('USERNAME')}\\miniconda3\\envs\\{self.conda_env}\\python.exe",
            f"C:\\ProgramData\\Anaconda3\\envs\\{self.conda_env}\\python.exe"
        ]
        
        for path in conda_paths:
            if os.path.exists(path):
                return path
        
        try:
            result = subprocess.run(
                "conda info --envs", 
                shell=True, capture_output=True, text=True, encoding='utf-8'
            )
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if self.conda_env in line and '*' not in line:
                        parts = line.split()
                        if len(parts) >= 2:
                            env_path = parts[-1]
                            python_path = os.path.join(env_path, "python.exe")
                            if os.path.exists(python_path):
                                return python_path
        except:
            pass
                
        return "python"
    
    def start_module(self, module_id):
        """啟動單個模組"""
        if module_id not in self.modules:
            return False
            
        module = self.modules[module_id]
        
        # 檢查ModbusTCP Server是否先啟動
        if module_id != "modbus" and not self.modules["modbus"]["enabled"]:
            self.add_log(module_id, "錯誤: 必須先啟動ModbusTCP Server")
            return False
        
        if module["enabled"]:
            self.add_log(module_id, "模組已在運行中")
            return True
            
        self.add_log(module_id, f"正在啟動 {module['name']}...")
        
        try:
            # 檢查檔案
            if not os.path.exists(module["path"]):
                self.add_log(module_id, f"錯誤: 找不到檔案 {module['path']}")
                return False
                
            python_cmd = self.get_conda_python_path()
            cmd = f'"{python_cmd}" "{module["path"]}"'
            
            # 設置環境變量
            env = os.environ.copy()
            env['PYTHONIOENCODING'] = 'utf-8'
            env['CONDA_NO_PLUGINS'] = 'true'
            
            # 啟動進程
            process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True,
                env=env,
                encoding='utf-8',
                errors='replace'
            )
            
            # 儲存進程
            self.processes[module_id] = process
            
            # 創建輸出隊列
            output_queue = Queue()
            self.output_queues[module_id] = output_queue
            
            # 啟動輸出讀取線程
            def read_output():
                try:
                    for line in iter(process.stdout.readline, ''):
                        if line and self.running:
                            clean_line = line.rstrip()
                            output_queue.put(clean_line)
                            self.add_log(module_id, clean_line)
                except:
                    pass
                finally:
                    output_queue.put(None)
                    
            thread = threading.Thread(target=read_output, daemon=True)
            thread.start()
            
            # 更新狀態
            module["enabled"] = True
            module["status"] = "運行中"
            
            self.add_log(module_id, f"✓ {module['name']} 啟動成功 (PID: {process.pid})")
            return True
            
        except Exception as e:
            self.add_log(module_id, f"✗ {module['name']} 啟動失敗: {e}")
            module["status"] = "錯誤"
            return False
    
    def stop_module(self, module_id):
        """停止單個模組"""
        if module_id not in self.modules:
            return False
            
        module = self.modules[module_id]
        
        if not module["enabled"]:
            return True
            
        try:
            process = self.processes.get(module_id)
            if process and process.poll() is None:
                self.add_log(module_id, f"正在停止 {module['name']}...")
                process.terminate()
                
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.add_log(module_id, f"強制終止 {module['name']}...")
                    process.kill()
                    
                self.add_log(module_id, f"✓ {module['name']} 已停止")
            
            # 更新狀態
            module["enabled"] = False
            module["status"] = "停止"
            
            # 清理
            if module_id in self.processes:
                del self.processes[module_id]
            if module_id in self.output_queues:
                del self.output_queues[module_id]
                
            return True
            
        except Exception as e:
            self.add_log(module_id, f"停止失敗: {e}")
            return False
    
    def start_all_modules(self):
        """一鍵啟動所有模組 (按順序)"""
        # 模組啟動順序
        start_order = ["modbus", "vp", "led", "gripper", "ccd1", "ccd3"]
        
        for module_id in start_order:
            if module_id in self.modules:
                module = self.modules[module_id]
                
                # 添加延遲 (除了第一個)
                if module_id != "modbus":
                    delay = module.get("delay", 1)
                    self.add_log(module_id, f"等待 {delay} 秒後啟動...")
                    time.sleep(delay)
                
                # 啟動模組
                if not module["enabled"]:
                    self.start_module(module_id)
                    
        return True
    
    def stop_all_modules(self):
        """一鍵關閉所有模組 (按逆序)"""
        # 模組關閉順序 (逆序，ModbusTCP最後關閉)
        stop_order = ["ccd3", "ccd1", "gripper", "led", "vp", "modbus"]
        
        for module_id in stop_order:
            if module_id in self.modules and self.modules[module_id]["enabled"]:
                self.stop_module(module_id)
                # 關閉間隔
                if module_id != "modbus":
                    time.sleep(1)
                    
        return True
    
    def add_log(self, module_id, message):
        """添加日誌"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        
        if module_id in self.module_logs:
            self.module_logs[module_id].append(log_entry)
            # 保持最新100條日誌
            if len(self.module_logs[module_id]) > 100:
                self.module_logs[module_id] = self.module_logs[module_id][-100:]
    
    def get_status(self):
        """獲取系統狀態"""
        status = {}
        for module_id, module in self.modules.items():
            # 檢查進程狀態
            if module["enabled"]:
                process = self.processes.get(module_id)
                if process and process.poll() is not None:
                    # 進程已結束
                    module["enabled"] = False
                    module["status"] = "錯誤"
                    self.add_log(module_id, f"{module['name']} 意外停止")
            
            status[module_id] = {
                "name": module["name"],
                "enabled": module["enabled"],
                "status": module["status"],
                "required": module["required"],
                "logs": self.module_logs[module_id][-20:]  # 最新20條日誌
            }
        
        return status
    
    def stop_all(self):
        """停止所有模組"""
        self.running = False
        self.stop_all_modules()

# Flask應用
app = Flask(__name__)
app.config['SECRET_KEY'] = 'dobot_launcher_secret'
socketio = SocketIO(app, cors_allowed_origins="*")

# 全局啟動器實例
launcher = ModuleLauncher()

@app.route('/')
def index():
    """主頁面"""
    return render_template('start.html')

@app.route('/api/status')
def api_status():
    """獲取系統狀態API"""
    return jsonify(launcher.get_status())

@app.route('/api/toggle/<module_id>', methods=['POST'])
def api_toggle(module_id):
    """切換模組狀態API"""
    data = request.get_json()
    enable = data.get('enable', False)
    
    if enable:
        success = launcher.start_module(module_id)
    else:
        success = launcher.stop_module(module_id)
    
    return jsonify({
        'success': success,
        'status': launcher.get_status()
    })

@app.route('/api/start_all', methods=['POST'])
def api_start_all():
    """一鍵啟動所有模組API"""
    def start_all_async():
        launcher.start_all_modules()
    
    # 在背景執行啟動流程
    thread = threading.Thread(target=start_all_async, daemon=True)
    thread.start()
    
    return jsonify({
        'success': True,
        'message': '正在依序啟動所有模組...'
    })

@app.route('/api/stop_all', methods=['POST'])
def api_stop_all():
    """一鍵關閉所有模組API"""
    def stop_all_async():
        launcher.stop_all_modules()
    
    # 在背景執行關閉流程
    thread = threading.Thread(target=stop_all_async, daemon=True)
    thread.start()
    
    return jsonify({
        'success': True,
        'message': '正在依序關閉所有模組...'
    })

@socketio.on('connect')
def handle_connect():
    """WebSocket連接"""
    emit('status_update', launcher.get_status())

def status_broadcast_thread():
    """狀態廣播線程"""
    while launcher.running:
        try:
            socketio.emit('status_update', launcher.get_status())
            time.sleep(2)  # 每2秒更新一次
        except:
            break

def main():
    """主函數"""
    print("DobotDR系統模組啟動器 - Web版")
    print("Web介面: http://localhost:5000")
    
    # 啟動狀態廣播線程
    broadcast_thread = threading.Thread(target=status_broadcast_thread, daemon=True)
    broadcast_thread.start()
    
    try:
        socketio.run(app, host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("\n正在關閉...")
    finally:
        launcher.stop_all()

if __name__ == "__main__":
    main()