# -*- coding: utf-8 -*-
"""
ModbusTestTool_Fixed.py - CCD1視覺系統 Modbus TCP 測試工具 (修正版)
基於CustomTkinter的GUI工具，用於模擬PLC操作和監控系統狀態
"""

import customtkinter as ctk
import threading
import time
import json
from typing import Optional, Dict, Any
import tkinter as tk
from tkinter import messagebox, scrolledtext

# 導入Modbus TCP Client
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
    print("✅ Modbus Client模組可用")
except ImportError as e:
    print(f"⚠️ Modbus Client模組不可用: {e}")
    MODBUS_AVAILABLE = False

# 設置外觀模式
ctk.set_appearance_mode("light")  # "light" 或 "dark"
ctk.set_default_color_theme("blue")  # "blue", "green", "dark-blue"


class ModbusTestTool:
    """Modbus TCP 測試工具主類"""
    
    def __init__(self):
        # 創建主窗口
        self.root = ctk.CTk()
        self.root.title("🤝 CCD1視覺系統 - Modbus TCP 測試工具")
        self.root.geometry("1200x800")
        self.root.resizable(True, True)
        
        # Modbus連接參數
        self.server_ip = "127.0.0.1"
        self.server_port = 502
        self.client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # 監控線程控制
        self.monitoring = False
        self.monitor_thread = None
        self.update_interval = 1.0  # 1秒更新間隔
        
        # 寄存器映射 (與CCD1VisionCode.py保持一致)
        self.REGISTERS = {
            'CONTROL_COMMAND': 200,        # 控制指令寄存器
            'STATUS_REGISTER': 201,        # 狀態寄存器
            'MIN_AREA_HIGH': 210,          # 最小面積 (高16位)
            'MIN_AREA_LOW': 211,           # 最小面積 (低16位)
            'MIN_ROUNDNESS': 212,          # 最小圓度 (×1000)
            'GAUSSIAN_KERNEL': 213,        # 高斯核大小
            'CANNY_LOW': 214,              # Canny低閾值
            'CANNY_HIGH': 215,             # Canny高閾值
            'CIRCLE_COUNT': 240,           # 檢測圓形數量
            'CIRCLE_1_X': 241,             # 圓形1 X座標
            'CIRCLE_1_Y': 242,             # 圓形1 Y座標
            'CIRCLE_1_RADIUS': 243,        # 圓形1 半徑
            'CIRCLE_2_X': 244,             # 圓形2 X座標
            'CIRCLE_2_Y': 245,             # 圓形2 Y座標
            'CIRCLE_2_RADIUS': 246,        # 圓形2 半徑
            'CIRCLE_3_X': 247,             # 圓形3 X座標
            'CIRCLE_3_Y': 248,             # 圓形3 Y座標
            'CIRCLE_3_RADIUS': 249,        # 圓形3 半徑
            'OPERATION_COUNT': 283,        # 操作計數器
            'ERROR_COUNT': 284,            # 錯誤計數器
            'LAST_CAPTURE_TIME': 280,      # 最後拍照耗時
            'LAST_PROCESS_TIME': 281,      # 最後處理耗時
            'LAST_TOTAL_TIME': 282,        # 最後總耗時
        }
        
        # 狀態變量
        self.status_bits = {'ready': 0, 'running': 0, 'alarm': 0, 'initialized': 0}
        self.detection_results = {'circle_count': 0, 'circles': []}
        self.last_values = {}  # 用於檢測數值變化
        
        # 固定狀態值定義 (對應CCD1VisionCode_Enhanced.py)
        self.STATUS_VALUES = {
            0: "全部停止 (0000)",      # 所有位都是0
            1: "準備就緒 (0001)",      # Ready=1 (初始狀態)
            2: "執行中_A (0010)",      # Running=1
            3: "執行中_B (0011)",      # Ready=1, Running=1
            4: "系統異常 (0100)",      # Alarm=1
            5: "異常準備 (0101)",      # Ready=1, Alarm=1
            8: "已初始化 (1000)",      # Initialized=1
            9: "完全就緒 (1001)",      # Ready=1, Initialized=1 (正常工作狀態)
            10: "初始執行 (1010)",     # Running=1, Initialized=1
            11: "全部啟動 (1011)",     # Ready=1, Running=1, Initialized=1
            12: "初始異常 (1100)",     # Alarm=1, Initialized=1
        }
        
        # 連接狀態標記
        self.connection_error_message = ""
        self.auto_connect_attempted = False
        
        # 創建GUI
        self.create_widgets()
        
        # 綁定關閉事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # 延遲自動連接 (等待GUI完全加載)
        self.root.after(1000, self.auto_connect_on_startup)

    def auto_connect_on_startup(self):
        """程序啟動時自動連接"""
        if not MODBUS_AVAILABLE:
            self.connection_error_message = "Modbus模組不可用，請安裝pymodbus"
            self.conn_status.configure(text="❌ 模組錯誤", text_color="red")
            self.conn_detail.configure(text=self.connection_error_message)
            self.log_message("❌ " + self.connection_error_message)
            return
        
        self.log_message("🔄 程序啟動，嘗試自動連接到 127.0.0.1:502...")
        self.auto_connect_attempted = True
        
        try:
            # 更新狀態顯示
            self.conn_status.configure(text="🔄 自動連接中...", text_color="orange")
            self.conn_detail.configure(text="正在嘗試連接到Modbus TCP服務器...")
            
            # 確保使用正確的IP和端口
            self.server_ip = "127.0.0.1"
            self.server_port = 502
            
            # 嘗試連接
            success = self._attempt_connection()
            
            if success:
                self.log_message("✅ 自動連接成功，開始監控")
                # 立即讀取一次狀態
                self.root.after(500, self.initial_status_read)
            else:
                self._show_connection_error()
                
        except Exception as e:
            self.connection_error_message = f"自動連接異常: {str(e)}"
            self._show_connection_error()
    
    def _attempt_connection(self) -> bool:
        """嘗試建立連接"""
        try:
            # 先關閉現有連接（如果有）
            if self.client:
                try:
                    self.client.close()
                except:
                    pass
                self.client = None
            
            self.log_message(f"🔗 嘗試連接到 {self.server_ip}:{self.server_port}")
            
            # pymodbus 3.9.2 的基本參數（移除不支援的參數）
            self.client = ModbusTcpClient(
                host=self.server_ip,
                port=self.server_port,
                timeout=5,
                retries=3,
                reconnect_delay=1,
                reconnect_delay_max=10
            )
            
            self.log_message("✅ 使用 pymodbus 3.9.2 參數創建客戶端")
            
            # 嘗試連接
            connect_result = self.client.connect()
            self.log_message(f"🔗 連接結果: {connect_result}")
            
            if connect_result:
                self.connected = True
                self.log_message("✅ TCP連接建立成功")
                
                # 等待一下讓連接穩定
                time.sleep(0.1)
                
                # 測試讀取狀態寄存器 - 使用 pymodbus 3.x 的方式
                self.log_message(f"🔧 測試讀取狀態寄存器地址: {self.REGISTERS['STATUS_REGISTER']}")
                
                test_result = self.client.read_holding_registers(
                    address=self.REGISTERS['STATUS_REGISTER'], 
                    count=1, 
                    slave=1  # pymodbus 3.x 仍然支持 slave 參數
                )
                
                if test_result.isError():
                    self.log_message(f"❌ 讀取測試失敗: {test_result}")
                    self.connection_error_message = f"連接成功但讀取狀態寄存器失敗: {test_result}"
                    self.client.close()
                    self.connected = False
                    return False
                else:
                    status_value = test_result.registers[0]
                    self.log_message(f"✅ 讀取測試成功，狀態值: {status_value}")
                    self.conn_status.configure(text="✅ 已連接", text_color="green")
                    self.conn_detail.configure(text=f"成功連接到 {self.server_ip}:{self.server_port}，狀態寄存器: {status_value}")
                    
                    # 啟用控制按鈕
                    self.enable_control_buttons(True)
                    
                    # 開始監控
                    if not self.monitoring:
                        self.start_monitoring()
                    
                    return True
                    
            else:
                self.connection_error_message = f"無法建立TCP連接到 {self.server_ip}:{self.server_port}"
                self.log_message(f"❌ {self.connection_error_message}")
                return False
                
        except Exception as e:
            self.connection_error_message = f"連接錯誤: {str(e)}"
            self.log_message(f"❌ 連接時發生錯誤: {str(e)}")
            self.log_message(f"❌ 錯誤類型: {type(e).__name__}")
            import traceback
            self.log_message(f"❌ 詳細錯誤: {traceback.format_exc()}")
            return False
    
    def _show_connection_error(self):
        """顯示連接錯誤信息"""
        self.conn_status.configure(text="❌ 連接失敗", text_color="red")
        self.conn_detail.configure(text=self.connection_error_message)
        self.log_message(f"❌ {self.connection_error_message}")
        self.log_message("💡 請確認:")
        self.log_message("   1. CCD1VisionCode_Enhanced.py 是否正在運行")
        self.log_message("   2. Modbus TCP服務器是否在 127.0.0.1:502 監聽")
        self.log_message("   3. 防火牆是否阻擋連接")
    
    def initial_status_read(self):
        """初始狀態讀取"""
        if self.connected:
            threading.Thread(target=self._initial_read_worker, daemon=True).start()
    
    def _initial_read_worker(self):
        """初始讀取工作線程"""
        try:
            self.read_status_register()
            self.read_detection_results()
            self.read_statistics()
        except Exception as e:
            self.log_message(f"❌ 初始狀態讀取失敗: {str(e)}")
    
    def create_widgets(self):
        """創建GUI組件"""
        # 創建主框架
        main_frame = ctk.CTkFrame(self.root)
        main_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        # 頂部標題
        title_label = ctk.CTkLabel(main_frame, text="🤝 運動控制握手測試工具", 
                                 font=ctk.CTkFont(size=24, weight="bold"))
        title_label.pack(pady=(10, 20))
        
        # 創建左右分欄
        content_frame = ctk.CTkFrame(main_frame)
        content_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        # 左側控制面板
        left_frame = ctk.CTkFrame(content_frame)
        left_frame.pack(side="left", fill="y", padx=(0, 10), pady=10)
        left_frame.configure(width=400)
        
        # 右側監控面板
        right_frame = ctk.CTkFrame(content_frame)
        right_frame.pack(side="right", fill="both", expand=True, padx=(10, 0), pady=10)
        
        # 創建左側面板內容
        self.create_control_panel(left_frame)
        
        # 創建右側面板內容
        self.create_monitor_panel(right_frame)
    
    def create_control_panel(self, parent):
        """創建控制面板"""
        # === 連接控制區域 ===
        conn_frame = ctk.CTkFrame(parent)
        conn_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(conn_frame, text="🔗 Modbus TCP 連接", 
                    font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(10, 5))
        
        # IP地址輸入
        ip_frame = ctk.CTkFrame(conn_frame)
        ip_frame.pack(fill="x", padx=10, pady=5)
        
        ctk.CTkLabel(ip_frame, text="服務器IP:").pack(side="left", padx=(10, 5))
        self.ip_entry = ctk.CTkEntry(ip_frame, placeholder_text="127.0.0.1")
        self.ip_entry.pack(side="left", fill="x", expand=True, padx=5)
        self.ip_entry.insert(0, self.server_ip)
        
        # 端口輸入
        port_frame = ctk.CTkFrame(conn_frame)
        port_frame.pack(fill="x", padx=10, pady=5)
        
        ctk.CTkLabel(port_frame, text="端口:").pack(side="left", padx=(10, 5))
        self.port_entry = ctk.CTkEntry(port_frame, placeholder_text="502", width=100)
        self.port_entry.pack(side="left", padx=5)
        self.port_entry.insert(0, str(self.server_port))
        
        # 連接按鈕
        conn_btn_frame = ctk.CTkFrame(conn_frame)
        conn_btn_frame.pack(fill="x", padx=10, pady=10)
        
        self.connect_btn = ctk.CTkButton(conn_btn_frame, text="🔗 連接", 
                                       command=self.toggle_connection, width=100)
        self.connect_btn.pack(side="left", padx=5)
        
        self.test_btn = ctk.CTkButton(conn_btn_frame, text="🔧 測試", 
                                    command=self.test_connection, width=100)
        self.test_btn.pack(side="left", padx=5)
        
        # 連接狀態指示 - 增強版
        status_container = ctk.CTkFrame(conn_frame)
        status_container.pack(fill="x", padx=10, pady=5)
        
        self.conn_status = ctk.CTkLabel(status_container, text="❌ 未連接", 
                                      text_color="red", font=ctk.CTkFont(size=12, weight="bold"))
        self.conn_status.pack(pady=5)
        
        # 連接詳細信息顯示
        self.conn_detail = ctk.CTkLabel(status_container, text="", 
                                      text_color="gray", font=ctk.CTkFont(size=10),
                                      wraplength=350)
        self.conn_detail.pack(pady=2)
        
        # === 控制指令區域 ===
        cmd_frame = ctk.CTkFrame(parent)
        cmd_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(cmd_frame, text="🎮 控制指令", 
                    font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(10, 5))
        
        # 控制按鈕
        cmd_btn_frame = ctk.CTkFrame(cmd_frame)
        cmd_btn_frame.pack(fill="x", padx=10, pady=10)
        
        self.clear_btn = ctk.CTkButton(cmd_btn_frame, text="🗑️ 清空 (0)", 
                                     command=lambda: self.send_command(0), width=160)
        self.clear_btn.pack(pady=2)
        
        self.capture_btn = ctk.CTkButton(cmd_btn_frame, text="📸 拍照 (8)", 
                                       command=lambda: self.send_command(8), width=160)
        self.capture_btn.pack(pady=2)
        
        self.detect_btn = ctk.CTkButton(cmd_btn_frame, text="🔍 拍照+檢測 (16)", 
                                      command=lambda: self.send_command(16), width=160)
        self.detect_btn.pack(pady=2)
        
        self.init_btn = ctk.CTkButton(cmd_btn_frame, text="🔄 重新初始化 (32)", 
                                    command=lambda: self.send_command(32), width=160)
        self.init_btn.pack(pady=2)
        
        # === 視覺參數設定區域 ===
        param_frame = ctk.CTkFrame(parent)
        param_frame.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(param_frame, text="⚙️ 視覺檢測參數", 
                    font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(10, 5))
        
        # 參數輸入
        self.create_param_inputs(param_frame)
        
        # 參數按鈕
        param_btn_frame = ctk.CTkFrame(param_frame)
        param_btn_frame.pack(fill="x", padx=10, pady=10)
        
        self.read_params_btn = ctk.CTkButton(param_btn_frame, text="📖 讀取參數", 
                                           command=self.read_parameters, width=100)
        self.read_params_btn.pack(side="left", padx=5)
        
        self.write_params_btn = ctk.CTkButton(param_btn_frame, text="💾 寫入參數", 
                                            command=self.write_parameters, width=100)
        self.write_params_btn.pack(side="left", padx=5)
    
    def create_param_inputs(self, parent):
        """創建參數輸入組件"""
        params_container = ctk.CTkFrame(parent)
        params_container.pack(fill="x", padx=10, pady=5)
        
        # 最小面積
        area_frame = ctk.CTkFrame(params_container)
        area_frame.pack(fill="x", padx=5, pady=2)
        ctk.CTkLabel(area_frame, text="最小面積:", width=80).pack(side="left", padx=5)
        self.area_entry = ctk.CTkEntry(area_frame, placeholder_text="50000", width=100)
        self.area_entry.pack(side="left", padx=5)
        
        # 最小圓度
        roundness_frame = ctk.CTkFrame(params_container)
        roundness_frame.pack(fill="x", padx=5, pady=2)
        ctk.CTkLabel(roundness_frame, text="最小圓度:", width=80).pack(side="left", padx=5)
        self.roundness_entry = ctk.CTkEntry(roundness_frame, placeholder_text="0.8", width=100)
        self.roundness_entry.pack(side="left", padx=5)
        
        # 高斯核大小
        gaussian_frame = ctk.CTkFrame(params_container)
        gaussian_frame.pack(fill="x", padx=5, pady=2)
        ctk.CTkLabel(gaussian_frame, text="高斯核:", width=80).pack(side="left", padx=5)
        self.gaussian_entry = ctk.CTkEntry(gaussian_frame, placeholder_text="9", width=100)
        self.gaussian_entry.pack(side="left", padx=5)
        
        # Canny閾值
        canny_frame = ctk.CTkFrame(params_container)
        canny_frame.pack(fill="x", padx=5, pady=2)
        ctk.CTkLabel(canny_frame, text="Canny低:", width=80).pack(side="left", padx=5)
        self.canny_low_entry = ctk.CTkEntry(canny_frame, placeholder_text="20", width=60)
        self.canny_low_entry.pack(side="left", padx=2)
        ctk.CTkLabel(canny_frame, text="高:", width=30).pack(side="left", padx=2)
        self.canny_high_entry = ctk.CTkEntry(canny_frame, placeholder_text="60", width=60)
        self.canny_high_entry.pack(side="left", padx=2)
    
    def create_monitor_panel(self, parent):
        """創建監控面板"""
        # 創建可滾動的主框架
        main_scroll_frame = ctk.CTkScrollableFrame(parent)
        main_scroll_frame.pack(fill="both", expand=True, padx=5, pady=5)
        
        # === 系統狀態區域 ===
        status_frame = ctk.CTkFrame(main_scroll_frame)
        status_frame.pack(fill="x", padx=5, pady=10)
        
        ctk.CTkLabel(status_frame, text="📊 系統狀態機", 
                    font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(10, 5))
        
        # 狀態寄存器值顯示 - 更醒目的設計
        status_header_frame = ctk.CTkFrame(status_frame)
        status_header_frame.pack(fill="x", padx=10, pady=5)
        
        # 大字體顯示狀態寄存器值
        status_value_frame = ctk.CTkFrame(status_header_frame)
        status_value_frame.pack(fill="x", padx=5, pady=5)
        
        ctk.CTkLabel(status_value_frame, text="狀態寄存器 (地址201):", 
                    font=ctk.CTkFont(size=14, weight="bold")).pack(side="left", padx=5)
        
        self.status_reg_label = ctk.CTkLabel(status_value_frame, text="0", 
                                           font=ctk.CTkFont(size=28, weight="bold"),
                                           text_color="blue")
        self.status_reg_label.pack(side="left", padx=10)
        
        self.status_binary_label = ctk.CTkLabel(status_value_frame, text="(0000)", 
                                              font=ctk.CTkFont(size=16),
                                              text_color="gray")
        self.status_binary_label.pack(side="left", padx=5)
        
        # 顯示讀取地址信息
        address_info_frame = ctk.CTkFrame(status_header_frame)
        address_info_frame.pack(fill="x", padx=5, pady=2)
        
        self.address_info_label = ctk.CTkLabel(address_info_frame, 
                                             text=f"讀取地址: {self.REGISTERS['STATUS_REGISTER']} (STATUS_REGISTER)", 
                                             font=ctk.CTkFont(size=11),
                                             text_color="darkblue")
        self.address_info_label.pack(anchor="w", padx=5)
        
        # 狀態描述
        self.status_desc_label = ctk.CTkLabel(status_header_frame, text="系統未連接", 
                                            font=ctk.CTkFont(size=14, weight="bold"),
                                            text_color="gray")
        self.status_desc_label.pack(pady=5)
        
        # 狀態值歷史記錄
        history_frame = ctk.CTkFrame(status_frame)
        history_frame.pack(fill="x", padx=10, pady=5)
        
        ctk.CTkLabel(history_frame, text="📋 常見狀態值含義:", 
                    font=ctk.CTkFont(size=12, weight="bold")).pack(anchor="w", padx=5, pady=2)
        
        # 常見狀態值說明 - 表格形式
        states_info_frame = ctk.CTkFrame(history_frame)
        states_info_frame.pack(fill="x", padx=5, pady=5)
        
        common_states = [
            ("1", "準備就緒", "Modbus連接完成", "orange"),
            ("9", "完全就緒", "相機+Modbus正常", "green"),
            ("10", "執行中", "正在處理指令", "blue"),
            ("4", "系統異常", "有錯誤發生", "red")
        ]
        
        for i, (value, name, desc, color) in enumerate(common_states):
            state_row = ctk.CTkFrame(states_info_frame)
            state_row.pack(fill="x", padx=2, pady=1)
            
            ctk.CTkLabel(state_row, text=value, 
                        font=ctk.CTkFont(size=12, weight="bold"),
                        text_color=color, width=30).pack(side="left", padx=5)
            ctk.CTkLabel(state_row, text=name, 
                        font=ctk.CTkFont(size=11, weight="bold"),
                        width=80).pack(side="left", padx=5)
            ctk.CTkLabel(state_row, text=desc, 
                        font=ctk.CTkFont(size=10),
                        text_color="gray").pack(side="left", padx=5)
        
        # 寄存器映射信息顯示
        registers_info_frame = ctk.CTkFrame(history_frame)
        registers_info_frame.pack(fill="x", padx=5, pady=5)
        
        ctk.CTkLabel(registers_info_frame, text="📋 主要寄存器地址:", 
                    font=ctk.CTkFont(size=11, weight="bold")).pack(anchor="w", padx=5)
        
        # 顯示關鍵寄存器地址
        key_registers = [
            ("控制指令", "CONTROL_COMMAND", 200),
            ("狀態寄存器", "STATUS_REGISTER", 201),
            ("圓形數量", "CIRCLE_COUNT", 240),
            ("操作計數", "OPERATION_COUNT", 283)
        ]
        
        for name, var_name, addr in key_registers:
            reg_row = ctk.CTkFrame(registers_info_frame)
            reg_row.pack(fill="x", padx=2, pady=1)
            
            color = "red" if var_name == "STATUS_REGISTER" else "gray"
            weight = "bold" if var_name == "STATUS_REGISTER" else "normal"
            
            ctk.CTkLabel(reg_row, text=f"地址{addr}:", 
                        font=ctk.CTkFont(size=10, weight=weight),
                        text_color=color, width=60).pack(side="left", padx=2)
            ctk.CTkLabel(reg_row, text=name, 
                        font=ctk.CTkFont(size=10, weight=weight),
                        text_color=color, width=60).pack(side="left", padx=2)
            ctk.CTkLabel(reg_row, text=f"({var_name})", 
                        font=ctk.CTkFont(size=9),
                        text_color="darkgray").pack(side="left", padx=2)
        
        # 四個狀態位顯示
        status_bits_frame = ctk.CTkFrame(status_frame)
        status_bits_frame.pack(fill="x", padx=10, pady=10)
        
        # 創建四個狀態位顯示
        self.create_status_bits(status_bits_frame)
        
        # === 檢測結果區域 ===
        results_frame = ctk.CTkFrame(main_scroll_frame)
        results_frame.pack(fill="x", padx=5, pady=10)
        
        ctk.CTkLabel(results_frame, text="🎯 檢測結果", 
                    font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(10, 5))
        
        # 結果統計
        self.create_results_summary(results_frame)
        
        # 詳細結果顯示
        self.create_results_detail(results_frame)
        
        # === 實時監控日誌 ===
        log_frame = ctk.CTkFrame(main_scroll_frame)
        log_frame.pack(fill="x", padx=5, pady=10)
        
        ctk.CTkLabel(log_frame, text="📝 實時監控日誌", 
                    font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(10, 5))
        
        # 日誌文本框
        self.log_text = ctk.CTkTextbox(log_frame, height=200)
        self.log_text.pack(fill="x", expand=False, padx=10, pady=10)
        
        # 日誌控制按鈕
        log_btn_frame = ctk.CTkFrame(log_frame)
        log_btn_frame.pack(fill="x", padx=10, pady=(0, 10))
        
        self.monitor_btn = ctk.CTkButton(log_btn_frame, text="▶️ 開始監控", 
                                       command=self.toggle_monitoring, width=100)
        self.monitor_btn.pack(side="left", padx=5)
        
        self.clear_log_btn = ctk.CTkButton(log_btn_frame, text="🗑️ 清空日誌", 
                                         command=self.clear_log, width=100)
        self.clear_log_btn.pack(side="left", padx=5)
        
        # 增加手動刷新按鈕
        self.refresh_btn = ctk.CTkButton(log_btn_frame, text="🔄 立即刷新", 
                                       command=self.force_refresh_all, width=100)
        self.refresh_btn.pack(side="left", padx=5)
    
    def create_status_bits(self, parent):
        """創建狀態位顯示"""
        # 使用網格布局
        status_grid = ctk.CTkFrame(parent)
        status_grid.pack(fill="x", padx=5, pady=5)
        
        # Ready狀態
        self.ready_frame = ctk.CTkFrame(status_grid)
        self.ready_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkLabel(self.ready_frame, text="Ready", font=ctk.CTkFont(weight="bold")).pack(pady=2)
        ctk.CTkLabel(self.ready_frame, text="bit0", font=ctk.CTkFont(size=10)).pack()
        self.ready_status = ctk.CTkLabel(self.ready_frame, text="0", 
                                       font=ctk.CTkFont(size=20, weight="bold"),
                                       text_color="gray")
        self.ready_status.pack(pady=2)
        
        # Running狀態
        self.running_frame = ctk.CTkFrame(status_grid)
        self.running_frame.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        ctk.CTkLabel(self.running_frame, text="Running", font=ctk.CTkFont(weight="bold")).pack(pady=2)
        ctk.CTkLabel(self.running_frame, text="bit1", font=ctk.CTkFont(size=10)).pack()
        self.running_status = ctk.CTkLabel(self.running_frame, text="0", 
                                         font=ctk.CTkFont(size=20, weight="bold"),
                                         text_color="gray")
        self.running_status.pack(pady=2)
        
        # Alarm狀態
        self.alarm_frame = ctk.CTkFrame(status_grid)
        self.alarm_frame.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkLabel(self.alarm_frame, text="Alarm", font=ctk.CTkFont(weight="bold")).pack(pady=2)
        ctk.CTkLabel(self.alarm_frame, text="bit2", font=ctk.CTkFont(size=10)).pack()
        self.alarm_status = ctk.CTkLabel(self.alarm_frame, text="0", 
                                       font=ctk.CTkFont(size=20, weight="bold"),
                                       text_color="gray")
        self.alarm_status.pack(pady=2)
        
        # Initialized狀態
        self.initialized_frame = ctk.CTkFrame(status_grid)
        self.initialized_frame.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        ctk.CTkLabel(self.initialized_frame, text="Initialized", font=ctk.CTkFont(weight="bold")).pack(pady=2)
        ctk.CTkLabel(self.initialized_frame, text="bit3", font=ctk.CTkFont(size=10)).pack()
        self.initialized_status = ctk.CTkLabel(self.initialized_frame, text="0", 
                                             font=ctk.CTkFont(size=20, weight="bold"),
                                             text_color="gray")
        self.initialized_status.pack(pady=2)
        
        # 設置網格權重
        status_grid.grid_columnconfigure(0, weight=1)
        status_grid.grid_columnconfigure(1, weight=1)
    
    def create_results_summary(self, parent):
        """創建結果統計顯示"""
        summary_frame = ctk.CTkFrame(parent)
        summary_frame.pack(fill="x", padx=10, pady=5)
        
        # 統計信息
        stats_grid = ctk.CTkFrame(summary_frame)
        stats_grid.pack(fill="x", padx=5, pady=5)
        
        # 圓形數量
        circle_frame = ctk.CTkFrame(stats_grid)
        circle_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkLabel(circle_frame, text="檢測圓形", font=ctk.CTkFont(size=12)).pack()
        self.circle_count_label = ctk.CTkLabel(circle_frame, text="0", 
                                             font=ctk.CTkFont(size=18, weight="bold"),
                                             text_color="blue")
        self.circle_count_label.pack()
        
        # 操作計數
        op_frame = ctk.CTkFrame(stats_grid)
        op_frame.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        ctk.CTkLabel(op_frame, text="操作計數", font=ctk.CTkFont(size=12)).pack()
        self.op_count_label = ctk.CTkLabel(op_frame, text="0", 
                                         font=ctk.CTkFont(size=18, weight="bold"))
        self.op_count_label.pack()
        
        # 錯誤計數
        err_frame = ctk.CTkFrame(stats_grid)
        err_frame.grid(row=0, column=2, padx=5, pady=5, sticky="ew")
        ctk.CTkLabel(err_frame, text="錯誤計數", font=ctk.CTkFont(size=12)).pack()
        self.err_count_label = ctk.CTkLabel(err_frame, text="0", 
                                          font=ctk.CTkFont(size=18, weight="bold"),
                                          text_color="red")
        self.err_count_label.pack()
        
        # 時間統計
        time_frame = ctk.CTkFrame(stats_grid)
        time_frame.grid(row=0, column=3, padx=5, pady=5, sticky="ew")
        ctk.CTkLabel(time_frame, text="總耗時(ms)", font=ctk.CTkFont(size=12)).pack()
        self.time_label = ctk.CTkLabel(time_frame, text="0", 
                                     font=ctk.CTkFont(size=18, weight="bold"),
                                     text_color="green")
        self.time_label.pack()
        
        # 設置網格權重
        stats_grid.grid_columnconfigure(0, weight=1)
        stats_grid.grid_columnconfigure(1, weight=1)
        stats_grid.grid_columnconfigure(2, weight=1)
        stats_grid.grid_columnconfigure(3, weight=1)
    
    def create_results_detail(self, parent):
        """創建詳細結果顯示"""
        detail_frame = ctk.CTkFrame(parent)
        detail_frame.pack(fill="x", padx=10, pady=5)
        
        ctk.CTkLabel(detail_frame, text="圓形詳細信息", 
                    font=ctk.CTkFont(size=14, weight="bold")).pack(pady=5)
        
        # 圓形信息文本框 - 固定高度
        self.results_text = ctk.CTkTextbox(detail_frame, height=100)
        self.results_text.pack(fill="x", expand=False, padx=5, pady=5)
    
    def toggle_connection(self):
        """切換連接狀態"""
        if not MODBUS_AVAILABLE:
            messagebox.showerror("錯誤", "Modbus模組不可用，請安裝pymodbus")
            return
        
        if self.connected:
            self.disconnect()
        else:
            self.connect()
    
    def connect(self):
        """連接到Modbus服務器"""
        try:
            # 從輸入框獲取IP和端口
            input_ip = self.ip_entry.get().strip()
            input_port = self.port_entry.get().strip()
            
            if input_ip:
                self.server_ip = input_ip
            if input_port:
                self.server_port = int(input_port)
            
            if not self.server_ip:
                messagebox.showerror("錯誤", "請輸入有效的IP地址")
                return
            
            self.log_message(f"🔗 手動連接到 {self.server_ip}:{self.server_port}...")
            
            # 更新連接狀態顯示
            self.conn_status.configure(text="🔄 連接中...", text_color="orange")
            self.conn_detail.configure(text=f"正在連接到 {self.server_ip}:{self.server_port}...")
            
            # 在新線程中嘗試連接，避免阻塞UI
            threading.Thread(target=self._connect_worker, daemon=True).start()
                
        except ValueError:
            error_msg = "端口號必須是數字"
            self.log_message(f"❌ {error_msg}")
            messagebox.showerror("參數錯誤", error_msg)
        except Exception as e:
            error_msg = f"連接失敗: {str(e)}"
            self.log_message(f"❌ {error_msg}")
            messagebox.showerror("連接失敗", error_msg)
    
    def _connect_worker(self):
        """連接工作線程"""
        success = self._attempt_connection()
        
        # 在主線程中更新UI
        self.root.after(0, self._update_connection_result, success)
    
    def _update_connection_result(self, success):
        """更新連接結果（主線程中調用）"""
        if success:
            self.connect_btn.configure(text="🔌 斷開")
            self.log_message("✅ 手動連接成功")
        else:
            self._show_connection_error()
            messagebox.showerror("連接失敗", self.connection_error_message)
    
    def disconnect(self):
        """斷開Modbus連接"""
        try:
            if self.monitoring:
                self.stop_monitoring()
            
            if self.client:
                self.client.close()
            
            self.connected = False
            self.connect_btn.configure(text="🔗 連接")
            self.conn_status.configure(text="❌ 已斷開", text_color="red")
            self.conn_detail.configure(text="手動斷開連接")
            self.log_message("🔌 已斷開連接")
            
            # 禁用控制按鈕
            self.enable_control_buttons(False)
            
            # 重置狀態顯示
            self.update_status_display(0)
            
        except Exception as e:
            self.log_message(f"❌ 斷開連接失敗: {str(e)}")
    
    def enable_control_buttons(self, enabled):
        """啟用/禁用控制按鈕"""
        state = "normal" if enabled else "disabled"
        self.clear_btn.configure(state=state)
        self.capture_btn.configure(state=state)
        self.detect_btn.configure(state=state)
        self.init_btn.configure(state=state)
        self.read_params_btn.configure(state=state)
        self.write_params_btn.configure(state=state)
    
    def test_connection(self):
        """測試連接"""
        if not self.connected:
            messagebox.showwarning("警告", "請先連接到Modbus服務器")
            return
        
        try:
            # 測試讀取狀態寄存器
            status_addr = self.REGISTERS['STATUS_REGISTER']
            
            # pymodbus 3.9.2 的標準讀取方式
            result = self.client.read_holding_registers(
                address=status_addr, 
                count=1, 
                slave=1
            )
            
            if not result.isError():
                status_value = result.registers[0]
                status_desc = self.STATUS_VALUES.get(status_value, f"未知狀態")
                
                test_msg = (f"🔧 連接測試成功\n"
                          f"讀取地址: {status_addr} (STATUS_REGISTER)\n"
                          f"狀態寄存器值: {status_value}\n"
                          f"狀態描述: {status_desc}")
                
                self.log_message(test_msg.replace('\n', ', '))
                messagebox.showinfo("測試成功", test_msg)
            else:
                raise Exception(f"讀取失敗: {result}")
                
        except Exception as e:
            self.log_message(f"❌ 連接測試失敗: {str(e)}")
            messagebox.showerror("測試失敗", str(e))
    
    def send_command(self, command):
        """發送控制指令"""
        if not self.connected:
            messagebox.showwarning("警告", "請先連接到Modbus服務器")
            return
        
        try:
            command_names = {0: "清空", 8: "拍照", 16: "拍照+檢測", 32: "重新初始化"}
            command_name = command_names.get(command, f"未知({command})")
            
            # 預期的狀態變化說明
            expected_changes = {
                0: "狀態應回到: 1(準備) 或 9(完全就緒)",
                8: "狀態變化: 9→10(執行)→9(完成)",
                16: "狀態變化: 9→10(執行)→9(完成)",
                32: "狀態變化: 重新初始化過程"
            }
            
            self.log_message(f"🎮 發送控制指令: {command} ({command_name})")
            self.log_message(f"💡 {expected_changes.get(command, '狀態變化未知')}")
            
            # pymodbus 3.9.2 的寫入方式
            result = self.client.write_register(
                address=self.REGISTERS['CONTROL_COMMAND'], 
                value=command, 
                slave=1
            )
            
            if not result.isError():
                self.log_message(f"✅ 控制指令發送成功: {command_name}")
                # 立即讀取一次狀態查看變化
                self.root.after(100, self.force_status_read)
            else:
                raise Exception(f"寫入失敗: {result}")
                
        except Exception as e:
            self.log_message(f"❌ 發送控制指令失敗: {str(e)}")
            messagebox.showerror("發送失敗", str(e))
    
    def force_status_read(self):
        """強制讀取一次狀態 (用於觀察指令後的狀態變化)"""
        if self.connected:
            threading.Thread(target=self.read_status_register, daemon=True).start()
    
    def read_parameters(self):
        """讀取視覺檢測參數"""
        if not self.connected:
            messagebox.showwarning("警告", "請先連接到Modbus服務器")
            return
        
        try:
            self.log_message("📖 正在讀取視覺檢測參數...")
            
            # 讀取參數寄存器
            params = {}
            param_registers = ['MIN_AREA_HIGH', 'MIN_AREA_LOW', 'MIN_ROUNDNESS', 
                             'GAUSSIAN_KERNEL', 'CANNY_LOW', 'CANNY_HIGH']
            
            for reg_name in param_registers:
                result = self.client.read_holding_registers(
                    self.REGISTERS[reg_name], 1, slave=1
                )
                if not result.isError():
                    params[reg_name] = result.registers[0]
                else:
                    raise Exception(f"讀取{reg_name}失敗")
            
            # 組合面積值
            area_value = (params['MIN_AREA_HIGH'] << 16) + params['MIN_AREA_LOW']
            roundness_value = params['MIN_ROUNDNESS'] / 1000.0
            
            # 更新界面
            self.area_entry.delete(0, tk.END)
            self.area_entry.insert(0, str(area_value))
            
            self.roundness_entry.delete(0, tk.END)
            self.roundness_entry.insert(0, str(roundness_value))
            
            self.gaussian_entry.delete(0, tk.END)
            self.gaussian_entry.insert(0, str(params['GAUSSIAN_KERNEL']))
            
            self.canny_low_entry.delete(0, tk.END)
            self.canny_low_entry.insert(0, str(params['CANNY_LOW']))
            
            self.canny_high_entry.delete(0, tk.END)
            self.canny_high_entry.insert(0, str(params['CANNY_HIGH']))
            
            self.log_message(f"✅ 參數讀取成功: 面積={area_value}, 圓度={roundness_value}")
            
        except Exception as e:
            self.log_message(f"❌ 讀取參數失敗: {str(e)}")
            messagebox.showerror("讀取失敗", str(e))
    
    def write_parameters(self):
        """寫入視覺檢測參數"""
        if not self.connected:
            messagebox.showwarning("警告", "請先連接到Modbus服務器")
            return
        
        try:
            self.log_message("💾 正在寫入視覺檢測參數...")
            
            # 獲取參數值
            area_value = int(float(self.area_entry.get() or "50000"))
            roundness_value = float(self.roundness_entry.get() or "0.8")
            gaussian_value = int(self.gaussian_entry.get() or "9")
            canny_low_value = int(self.canny_low_entry.get() or "20")
            canny_high_value = int(self.canny_high_entry.get() or "60")
            
            # 分解面積值為高低16位
            area_high = (area_value >> 16) & 0xFFFF
            area_low = area_value & 0xFFFF
            roundness_int = int(roundness_value * 1000)
            
            # 寫入參數
            params_to_write = [
                ('MIN_AREA_HIGH', area_high),
                ('MIN_AREA_LOW', area_low),
                ('MIN_ROUNDNESS', roundness_int),
                ('GAUSSIAN_KERNEL', gaussian_value),
                ('CANNY_LOW', canny_low_value),
                ('CANNY_HIGH', canny_high_value)
            ]
            
            for reg_name, value in params_to_write:
                result = self.client.write_register(
                    self.REGISTERS[reg_name], value, slave=1
                )
                if result.isError():
                    raise Exception(f"寫入{reg_name}失敗")
            
            self.log_message(f"✅ 參數寫入成功: 面積={area_value}, 圓度={roundness_value}")
            
        except ValueError as e:
            messagebox.showerror("參數錯誤", "請輸入有效的數值")
        except Exception as e:
            self.log_message(f"❌ 寫入參數失敗: {str(e)}")
            messagebox.showerror("寫入失敗", str(e))
    
    def toggle_monitoring(self):
        """切換監控狀態"""
        if self.monitoring:
            self.stop_monitoring()
        else:
            if self.connected:
                self.start_monitoring()
            else:
                messagebox.showwarning("警告", "請先連接到Modbus服務器")
    
    def start_monitoring(self):
        """開始監控"""
        if not self.connected:
            return
        
        self.monitoring = True
        self.monitor_btn.configure(text="⏸️ 停止監控")
        self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
        self.monitor_thread.start()
        self.log_message("▶️ 開始實時監控")
    
    def stop_monitoring(self):
        """停止監控"""
        self.monitoring = False
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2.0)
        self.monitor_btn.configure(text="▶️ 開始監控")
        self.log_message("⏸️ 停止實時監控")
    
    def monitor_loop(self):
        """監控循環"""
        while self.monitoring and self.connected:
            try:
                # 讀取狀態寄存器
                self.read_status_register()
                
                # 讀取檢測結果
                self.read_detection_results()
                
                # 讀取統計信息
                self.read_statistics()
                
                time.sleep(self.update_interval)
                
            except Exception as e:
                self.log_message(f"❌ 監控錯誤: {str(e)}")
                time.sleep(2.0)  # 錯誤時延長間隔
    
    def read_status_register(self):
        """讀取狀態寄存器"""
        try:
            status_addr = self.REGISTERS['STATUS_REGISTER']
            
            # pymodbus 3.9.2 的標準讀取方式
            result = self.client.read_holding_registers(
                address=status_addr, 
                count=1, 
                slave=1
            )
            
            if not result.isError():
                status_value = result.registers[0]
                
                # 解析狀態位
                ready = (status_value >> 0) & 1
                running = (status_value >> 1) & 1
                alarm = (status_value >> 2) & 1
                initialized = (status_value >> 3) & 1
                
                # 檢查狀態變化
                old_status = self.status_bits.copy()
                old_status_value = self.last_values.get('status_register', -1)
                
                self.status_bits = {
                    'ready': ready,
                    'running': running,
                    'alarm': alarm,
                    'initialized': initialized
                }
                
                # 記錄狀態寄存器值變化
                if old_status_value != status_value:
                    self.last_values['status_register'] = status_value
                    status_desc = self.STATUS_VALUES.get(status_value, f"未知狀態 ({status_value:04b})")
                    self.log_message(f"📊 狀態寄存器變化 [地址{status_addr}]: {old_status_value} → {status_value} ({status_desc})")
                
                # 更新界面 (在主線程中)
                self.root.after(0, self.update_status_display, status_value)
                
                # 記錄個別狀態位變化
                for bit_name, new_value in self.status_bits.items():
                    old_value = old_status.get(bit_name, -1)
                    if old_value != new_value and old_value != -1:
                        change_type = "啟動" if new_value else "停止"
                        self.log_message(f"🔹 {bit_name} {change_type}: {old_value} → {new_value}")
                
        except Exception as e:
            pass  # 靜默處理讀取錯誤
    
    def read_detection_results(self):
        """讀取檢測結果"""
        try:
            # 讀取圓形數量
            result = self.client.read_holding_registers(
                address=self.REGISTERS['CIRCLE_COUNT'], 
                count=1, 
                slave=1
            )
            
            if not result.isError():
                circle_count = result.registers[0]
                
                # 檢查數量變化
                if self.last_values.get('circle_count', -1) != circle_count:
                    self.last_values['circle_count'] = circle_count
                    
                    # 讀取圓形詳細信息
                    circles = []
                    for i in range(min(circle_count, 3)):  # 最多顯示3個圓形
                        x_result = self.client.read_holding_registers(
                            address=self.REGISTERS[f'CIRCLE_{i+1}_X'], 
                            count=1, 
                            slave=1
                        )
                        y_result = self.client.read_holding_registers(
                            address=self.REGISTERS[f'CIRCLE_{i+1}_Y'], 
                            count=1, 
                            slave=1
                        )
                        r_result = self.client.read_holding_registers(
                            address=self.REGISTERS[f'CIRCLE_{i+1}_RADIUS'], 
                            count=1, 
                            slave=1
                        )
                        
                        if (not x_result.isError() and not y_result.isError() and 
                            not r_result.isError()):
                            circles.append({
                                'id': i + 1,
                                'x': x_result.registers[0],
                                'y': y_result.registers[0],
                                'radius': r_result.registers[0]
                            })
                    
                    # 更新檢測結果顯示
                    self.root.after(0, self.update_results_display, circle_count, circles)
                    
                    if circle_count > 0:
                        self.log_message(f"🎯 檢測結果更新: 找到 {circle_count} 個圓形")
                
        except Exception as e:
            pass  # 靜默處理讀取錯誤
    
    def read_statistics(self):
        """讀取統計信息"""
        try:
            # 讀取操作計數和錯誤計數
            op_result = self.client.read_holding_registers(
                address=self.REGISTERS['OPERATION_COUNT'], 
                count=1, 
                slave=1
            )
            err_result = self.client.read_holding_registers(
                address=self.REGISTERS['ERROR_COUNT'], 
                count=1, 
                slave=1
            )
            time_result = self.client.read_holding_registers(
                address=self.REGISTERS['LAST_TOTAL_TIME'], 
                count=1, 
                slave=1
            )
            
            if (not op_result.isError() and not err_result.isError() and 
                not time_result.isError()):
                
                op_count = op_result.registers[0]
                err_count = err_result.registers[0]
                total_time = time_result.registers[0]
                
                # 檢查變化
                old_op = self.last_values.get('op_count', -1)
                old_err = self.last_values.get('err_count', -1)
                
                if old_op != op_count:
                    self.last_values['op_count'] = op_count
                    self.log_message(f"📈 操作計數: {op_count}")
                
                if old_err != err_count:
                    self.last_values['err_count'] = err_count
                    if err_count > old_err and old_err >= 0:
                        self.log_message(f"⚠️ 錯誤計數增加: {err_count}")
                
                # 更新統計顯示
                self.root.after(0, self.update_statistics_display, op_count, err_count, total_time)
                
        except Exception as e:
            pass  # 靜默處理讀取錯誤
    
    def update_status_display(self, status_value):
        """更新狀態顯示 (主線程中調用)"""
        # 更新狀態寄存器顯示 - 大字體醒目顯示
        binary_str = f"{status_value:04b}"
        self.status_reg_label.configure(text=str(status_value))
        self.status_binary_label.configure(text=f"({binary_str})")
        
        # 更新狀態描述和顏色
        status_desc = self.STATUS_VALUES.get(status_value, f"未知狀態")
        if status_value == 1:
            self.status_desc_label.configure(text=status_desc, text_color="orange")
            self.status_reg_label.configure(text_color="orange")
        elif status_value == 9:
            self.status_desc_label.configure(text=status_desc, text_color="green")
            self.status_reg_label.configure(text_color="green")
        elif status_value in [10, 11]:
            self.status_desc_label.configure(text=status_desc, text_color="blue")
            self.status_reg_label.configure(text_color="blue")
        elif status_value in [4, 5, 12]:
            self.status_desc_label.configure(text=status_desc, text_color="red")
            self.status_reg_label.configure(text_color="red")
        else:
            self.status_desc_label.configure(text=status_desc, text_color="gray")
            self.status_reg_label.configure(text_color="gray")
        
        # 更新各狀態位顯示
        self.update_status_bit(self.ready_status, self.ready_frame, self.status_bits['ready'])
        self.update_status_bit(self.running_status, self.running_frame, self.status_bits['running'])
        self.update_status_bit(self.alarm_status, self.alarm_frame, self.status_bits['alarm'])
        self.update_status_bit(self.initialized_status, self.initialized_frame, self.status_bits['initialized'])
    
    def update_status_bit(self, label, frame, value):
        """更新單個狀態位顯示"""
        label.configure(text=str(value))
        
        if value == 1:
            label.configure(text_color="white")
            frame.configure(fg_color=("green", "darkgreen"))  # 啟動狀態-綠色
        else:
            label.configure(text_color="gray")
            frame.configure(fg_color=("gray85", "gray20"))  # 停止狀態-灰色
    
    def update_results_display(self, circle_count, circles):
        """更新檢測結果顯示 (主線程中調用)"""
        self.circle_count_label.configure(text=str(circle_count))
        
        # 清空並更新詳細結果
        self.results_text.delete("1.0", tk.END)
        
        if circle_count > 0:
            result_text = f"檢測到 {circle_count} 個圓形:\n\n"
            for circle in circles:
                result_text += (f"圓形 {circle['id']}: "
                              f"中心({circle['x']}, {circle['y']}) "
                              f"半徑={circle['radius']}\n")
            
            self.circle_count_label.configure(text_color="blue")
        else:
            result_text = "未檢測到圓形"
            self.circle_count_label.configure(text_color="gray")
        
        self.results_text.insert("1.0", result_text)
    
    def update_statistics_display(self, op_count, err_count, total_time):
        """更新統計信息顯示 (主線程中調用)"""
        self.op_count_label.configure(text=str(op_count))
        self.err_count_label.configure(text=str(err_count))
        self.time_label.configure(text=str(total_time))
        
        # 根據錯誤數量設置顏色
        if err_count > 0:
            self.err_count_label.configure(text_color="red")
        else:
            self.err_count_label.configure(text_color="green")
    
    def log_message(self, message):
        """添加日誌訊息 (線程安全)"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        # 在主線程中更新UI
        self.root.after(0, self._append_log, log_entry)
    
    def _append_log(self, log_entry):
        """在主線程中添加日誌 (內部方法)"""
        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)  # 自動滾動到底部
        
        # 限制日誌長度，保留最後1000行
        lines = self.log_text.get("1.0", tk.END).split('\n')
        if len(lines) > 1000:
            self.log_text.delete("1.0", f"{len(lines)-1000}.0")
    
    def force_refresh_all(self):
        """強制刷新所有數據"""
        if self.connected:
            self.log_message("🔄 手動刷新所有數據...")
            # 立即執行一次完整的監控循環
            threading.Thread(target=self._manual_refresh, daemon=True).start()
        else:
            messagebox.showwarning("警告", "請先連接到Modbus服務器")
    
    def _manual_refresh(self):
        """手動刷新的內部方法"""
        try:
            self.read_status_register()
            self.read_detection_results()
            self.read_statistics()
            self.log_message("✅ 手動刷新完成")
        except Exception as e:
            self.log_message(f"❌ 手動刷新失敗: {str(e)}")
    
    def clear_log(self):
        """清空日誌"""
        self.log_text.delete("1.0", tk.END)
        self.log_message("📝 日誌已清空")
    
    def on_closing(self):
        """關閉程序時的清理工作"""
        if self.monitoring:
            self.stop_monitoring()
        if self.connected:
            self.disconnect()
        self.root.destroy()
    
    def run(self):
        """運行GUI程序"""
        self.log_message("🚀 CCD1視覺系統 Modbus TCP 測試工具已啟動")
        self.log_message("🔄 正在自動連接到 127.0.0.1:502...")
        self.log_message("📊 狀態寄存器固定值說明:")
        self.log_message("   • 1: 準備就緒 (Modbus連接完成)")
        self.log_message("   • 9: 完全就緒 (Modbus+相機都連接)")
        self.log_message("   • 10: 執行中 (處理指令時)")
        self.log_message("   • 4: 系統異常 (有錯誤發生)")
        self.log_message("🎯 使用滾動條查看完整界面")
        self.root.mainloop()


def main():
    """主函數"""
    print("🚀 正在啟動 CCD1視覺系統 Modbus TCP 測試工具...")
    
    if not MODBUS_AVAILABLE:
        print("⚠️ 警告: Modbus模組不可用，部分功能將無法使用")
        print("💡 請安裝: pip install pymodbus>=3.0.0")
    
    try:
        app = ModbusTestTool()
        app.run()
    except Exception as e:
        print(f"❌ 程序運行錯誤: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()