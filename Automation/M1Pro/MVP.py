#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
機械臂狀態機測試程序
基地址400規劃，來回運動Rotate_top和Rotate_down點位
"""

import json
import time
import threading
from pymodbus.client.tcp import ModbusTcpClient
from dobot_api import DobotApiDashboard, DobotApiMove

# Modbus地址配置（基地址400）
MODBUS_BASE_ADDR = 400
CONTROL_ADDR = MODBUS_BASE_ADDR + 0      # 400: 控制指令地址
RUNNING_ADDR = MODBUS_BASE_ADDR + 1      # 401: 運行狀態地址  
STANDBY_ADDR = MODBUS_BASE_ADDR + 2      # 402: 準備狀態地址

# 機械臂連接配置
ROBOT_IP = "192.168.1.6"
DASHBOARD_PORT = 29999
MOVE_PORT = 30003

# Modbus連接配置
MODBUS_IP = "127.0.0.1"
MODBUS_PORT = 502
SLAVE_ID = 1

# 測試參數
SPEED_RATIO = 20  # 全局速度20%
CYCLE_COUNT = 10  # 10個完整循環
POINT_DELAY = 0.5  # 兩點間等待0.5秒

class RobotStateMachine:
    def __init__(self):
        self.modbus_client = None
        self.dashboard_api = None
        self.move_api = None
        self.robot_points = {}
        self.running = False
        
    def load_robot_points(self):
        """載入機械臂點位數據"""
        try:
            with open("Automation/M1Pro/saved_points/robot_points.json", "r", encoding="utf-8") as f:
                points_list = json.load(f)
            
            # 轉換為字典格式便於查找
            for point in points_list:
                self.robot_points[point["name"]] = point
                
            print(f"載入點位數據成功，共{len(self.robot_points)}個點位")
            return True
            
        except Exception as e:
            print(f"載入點位數據失敗: {e}")
            return False
    
    def connect_modbus(self):
        """連接Modbus TCP Server"""
        try:
            self.modbus_client = ModbusTcpClient(MODBUS_IP, port=MODBUS_PORT)
            if self.modbus_client.connect():
                print(f"Modbus連接成功: {MODBUS_IP}:{MODBUS_PORT}")
                # 初始化狀態寄存器
                self.write_modbus(RUNNING_ADDR, 0)
                self.write_modbus(STANDBY_ADDR, 1)
                return True
            else:
                print("Modbus連接失敗")
                return False
        except Exception as e:
            print(f"Modbus連接異常: {e}")
            return False
    
    def connect_robot(self):
        """連接機械臂"""
        try:
            # 連接Dashboard API
            self.dashboard_api = DobotApiDashboard(ROBOT_IP, DASHBOARD_PORT)
            print(f"Dashboard連接成功: {ROBOT_IP}:{DASHBOARD_PORT}")
            
            # 連接Move API
            self.move_api = DobotApiMove(ROBOT_IP, MOVE_PORT)
            print(f"Move API連接成功: {ROBOT_IP}:{MOVE_PORT}")
            
            # 機械臂初始化
            self.dashboard_api.ClearError()
            self.dashboard_api.EnableRobot()
            self.dashboard_api.SpeedFactor(SPEED_RATIO)
            self.dashboard_api.SpeedJ(SPEED_RATIO)
            self.dashboard_api.AccJ(SPEED_RATIO)
            
            print(f"機械臂初始化完成，全局速度設定: {SPEED_RATIO}%")
            return True
            
        except Exception as e:
            print(f"機械臂連接失敗: {e}")
            return False
    
    def read_modbus(self, address):
        """讀取Modbus寄存器"""
        try:
            # 根據專案經驗，使用正確的API格式
            result = self.modbus_client.read_holding_registers(address, count=1)
            
            if hasattr(result, 'isError') and result.isError():
                print(f"讀取寄存器{address}失敗")
                return None
            elif hasattr(result, 'registers') and len(result.registers) > 0:
                return result.registers[0]
            else:
                print(f"讀取寄存器{address}無數據返回")
                return None
                
        except Exception as e:
            print(f"讀取寄存器{address}異常: {e}")
            return None
    
    def write_modbus(self, address, value):
        """寫入Modbus寄存器"""
        try:
            # 根據專案經驗，使用正確的API格式
            result = self.modbus_client.write_register(address, value)
            
            if hasattr(result, 'isError') and result.isError():
                print(f"寫入寄存器{address}={value}失敗")
                return False
            else:
                return True
                
        except Exception as e:
            print(f"寫入寄存器{address}={value}異常: {e}")
            return False
    
    def move_to_point(self, point_name):
        """移動到指定點位"""
        if point_name not in self.robot_points:
            print(f"點位{point_name}不存在")
            return False
            
        point = self.robot_points[point_name]
        joint = point["joint"]
        
        try:
            # 使用JointMovJ進行關節運動
            result = self.move_api.JointMovJ(
                joint["j1"], 
                joint["j2"], 
                joint["j3"], 
                joint["j4"]
            )
            print(f"移動到{point_name}: J1={joint['j1']:.2f}, J2={joint['j2']:.2f}, J3={joint['j3']:.2f}, J4={joint['j4']:.2f}")
            return True
            
        except Exception as e:
            print(f"移動到{point_name}失敗: {e}")
            return False
    
    def execute_cycle_motion(self):
        """執行循環運動"""
        print(f"開始執行{CYCLE_COUNT}個完整循環...")
        
        for cycle in range(CYCLE_COUNT):
            print(f"\n=== 第{cycle+1}/{CYCLE_COUNT}個循環 ===")
            
            # 移動到Rotate_top
            if not self.move_to_point("Rotate_top"):
                print("移動到Rotate_top失敗，停止執行")
                return False
            
            # 等待運動完成
            self.move_api.Sync()
            time.sleep(POINT_DELAY)
            
            # 移動到Rotate_down  
            if not self.move_to_point("Rotate_down"):
                print("移動到Rotate_down失敗，停止執行")
                return False
                
            # 等待運動完成
            self.move_api.Sync()
            time.sleep(POINT_DELAY)
            
            print(f"第{cycle+1}個循環完成")
        
        print(f"\n所有{CYCLE_COUNT}個循環執行完成！")
        return True
    
    def state_machine_loop(self):
        """狀態機主循環"""
        print("狀態機開始運行...")
        
        while self.running:
            try:
                # 讀取控制指令
                control_value = self.read_modbus(CONTROL_ADDR)
                
                if control_value == 1:
                    print("\n收到控制指令，開始執行動作...")
                    
                    # 設定運行狀態
                    self.write_modbus(RUNNING_ADDR, 1)
                    self.write_modbus(STANDBY_ADDR, 0)
                    
                    # 執行循環運動
                    success = self.execute_cycle_motion()
                    
                    # 清除運行狀態
                    self.write_modbus(RUNNING_ADDR, 0)
                    
                    if success:
                        print("動作執行完成，等待控制指令清除...")
                    else:
                        print("動作執行失敗！")
                    
                    # 等待對方清除控制指令
                    while self.read_modbus(CONTROL_ADDR) == 1:
                        time.sleep(0.1)
                    
                    # 設定準備狀態
                    self.write_modbus(STANDBY_ADDR, 1)
                    print("準備好接收下次指令\n")
                
                time.sleep(0.1)  # 狀態機循環間隔
                
            except KeyboardInterrupt:
                print("\n收到中斷信號，停止狀態機...")
                break
            except Exception as e:
                print(f"狀態機異常: {e}")
                time.sleep(1)
    
    def start(self):
        """啟動測試程序"""
        print("=== 機械臂狀態機測試程序 ===")
        
        # 載入點位數據
        if not self.load_robot_points():
            return
        
        # 檢查必要點位
        required_points = ["Rotate_top", "Rotate_down"]
        for point_name in required_points:
            if point_name not in self.robot_points:
                print(f"缺少必要點位: {point_name}")
                return
        
        # 連接Modbus
        if not self.connect_modbus():
            return
        
        # 連接機械臂
        if not self.connect_robot():
            return
        
        print(f"\n狀態機地址配置:")
        print(f"控制指令地址: {CONTROL_ADDR}")
        print(f"運行狀態地址: {RUNNING_ADDR}")
        print(f"準備狀態地址: {STANDBY_ADDR}")
        print(f"\n等待控制指令 (向地址{CONTROL_ADDR}寫入1開始執行)...")
        
        # 啟動狀態機
        self.running = True
        try:
            self.state_machine_loop()
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理資源"""
        print("清理資源...")
        
        self.running = False
        
        # 清除Modbus狀態
        if self.modbus_client:
            self.write_modbus(RUNNING_ADDR, 0)
            self.write_modbus(STANDBY_ADDR, 0)
            self.modbus_client.close()
        
        # 關閉機械臂連接
        if self.dashboard_api:
            self.dashboard_api.close()
        if self.move_api:
            self.move_api.close()
        
        print("資源清理完成")

def main():
    robot_sm = RobotStateMachine()
    robot_sm.start()

if __name__ == "__main__":
    main()