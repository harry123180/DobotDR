# -*- coding: utf-8 -*-
"""
VM_Test_Client.py - VisionModule測試程式
模擬外部模組對VM進行各種Modbus寫入測試
"""

import time
import threading
from typing import Dict, Any
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException, ConnectionException

class VMTestClient:
    """VM測試客戶端"""
    
    def __init__(self, host="127.0.0.1", port=502):
        self.host = host
        self.port = port
        self.client = None
        self.connected = False
        self.test_results = {}
        
        # 連接到Modbus服務器
        self._connect()
    
    def _connect(self):
        """連接到Modbus服務器"""
        try:
            self.client = ModbusTcpClient(self.host, port=self.port)
            self.connected = self.client.connect()
            if self.connected:
                print(f"測試客戶端連接成功: {self.host}:{self.port}")
            else:
                print(f"測試客戶端連接失敗: {self.host}:{self.port}")
        except Exception as e:
            print(f"測試客戶端連接異常: {e}")
            self.connected = False
    
    def read_register(self, address: int) -> int:
        """讀取寄存器"""
        if not self.connected:
            return 0
        
        try:
            response = self.client.read_holding_registers(address, 1)
            if response.isError():
                return 0
            return response.registers[0]
        except Exception:
            return 0
    
    def write_register(self, address: int, value: int) -> bool:
        """寫入寄存器"""
        if not self.connected:
            return False
        
        try:
            response = self.client.write_register(address, value)
            return not response.isError()
        except Exception:
            return False
    
    def test_basic_write_read(self):
        """測試基本讀寫功能"""
        print("\n=== 測試1: 基本讀寫功能 ===")
        
        test_addresses = [102, 103, 104, 105, 116, 124]
        test_values = [1, 1, 2, 900, 1, 1]
        
        for addr, val in zip(test_addresses, test_values):
            print(f"寫入地址{addr} = {val}")
            write_success = self.write_register(addr, val)
            time.sleep(0.1)  # 等待寫入完成
            
            read_value = self.read_register(addr)
            print(f"讀取地址{addr} = {read_value}")
            
            if write_success and read_value == val:
                print(f"✅ 地址{addr}讀寫正常")
            else:
                print(f"❌ 地址{addr}讀寫異常: 寫入={write_success}, 期望={val}, 實際={read_value}")
            
            time.sleep(0.5)
    
    def test_ccd1_initialization(self):
        """測試CCD1初始化觸發"""
        print("\n=== 測試2: CCD1初始化觸發 ===")
        
        # 讀取初始狀態
        initial_status = self.read_register(100)
        print(f"CCD1初始狀態: {initial_status}")
        
        # 觸發初始化
        print("觸發CCD1初始化 (地址102=1)")
        self.write_register(102, 1)
        
        # 監控狀態變化
        print("監控CCD1狀態變化...")
        for i in range(20):  # 監控10秒
            status = self.read_register(100)
            control = self.read_register(102)
            print(f"第{i+1}次檢查: 狀態={status}, 控制={control}")
            
            if status == 8:
                print("✅ 檢測到CCD1進入運作中狀態(8)")
            elif status == 9:
                print("✅ 檢測到CCD1進入準備好狀態(9)")
                break
            elif status == 12:
                print("❌ 檢測到CCD1進入錯誤狀態(12)")
                break
            elif status == 99:
                print("⚠️ 檢測到CCD1相機斷線狀態(99)")
                break
            
            time.sleep(0.5)
    
    def test_ccd3_initialization(self):
        """測試CCD3初始化觸發"""
        print("\n=== 測試3: CCD3初始化觸發 ===")
        
        # 讀取初始狀態
        initial_status = self.read_register(101)
        print(f"CCD3初始狀態: {initial_status}")
        
        # 觸發初始化
        print("觸發CCD3初始化 (地址103=1)")
        self.write_register(103, 1)
        
        # 監控狀態變化
        print("監控CCD3狀態變化...")
        for i in range(20):  # 監控10秒
            status = self.read_register(101)
            control = self.read_register(103)
            print(f"第{i+1}次檢查: 狀態={status}, 控制={control}")
            
            if status == 8:
                print("✅ 檢測到CCD3進入運作中狀態(8)")
            elif status == 9:
                print("✅ 檢測到CCD3進入準備好狀態(9)")
                break
            elif status == 12:
                print("❌ 檢測到CCD3進入錯誤狀態(12)")
                break
            elif status == 99:
                print("⚠️ 檢測到CCD3相機斷線狀態(99)")
                break
            
            time.sleep(0.5)
    
    def test_multiple_triggers(self):
        """測試多次觸發"""
        print("\n=== 測試4: 多次觸發測試 ===")
        
        for round_num in range(3):
            print(f"\n--- 第{round_num+1}輪觸發 ---")
            
            # 觸發CCD1初始化
            print("觸發CCD1初始化")
            self.write_register(102, 1)
            time.sleep(1)
            
            status_102 = self.read_register(102)
            status_100 = self.read_register(100)
            print(f"CCD1控制地址102: {status_102}")
            print(f"CCD1狀態地址100: {status_100}")
            
            time.sleep(2)
    
    def test_rapid_writes(self):
        """測試快速連續寫入"""
        print("\n=== 測試5: 快速連續寫入 ===")
        
        print("快速連續寫入地址102=1 (10次)")
        for i in range(10):
            self.write_register(102, 1)
            print(f"第{i+1}次寫入完成")
            time.sleep(0.05)  # 50ms間隔
        
        # 檢查最終狀態
        time.sleep(2)
        final_control = self.read_register(102)
        final_status = self.read_register(100)
        print(f"最終控制值: {final_control}")
        print(f"最終狀態值: {final_status}")
    
    def test_mixed_addresses(self):
        """測試混合地址寫入"""
        print("\n=== 測試6: 混合地址寫入 ===")
        
        # 同時寫入多個地址
        operations = [
            (102, 1, "CCD1初始化"),
            (104, 2, "CCD1模型選擇"),
            (105, 750, "CCD1置信度"),
            (107, 1, "CCD3檢測方法"),
            (108, 1, "CCD1LOG等級"),
            (127, 1, "CCD1靜默模式")
        ]
        
        print("同時寫入多個控制地址...")
        for addr, val, desc in operations:
            success = self.write_register(addr, val)
            print(f"寫入{desc} 地址{addr}={val}: {'成功' if success else '失敗'}")
            time.sleep(0.1)
        
        # 讀取結果
        print("\n讀取寫入結果:")
        for addr, val, desc in operations:
            actual = self.read_register(addr)
            print(f"{desc} 地址{addr}: 期望={val}, 實際={actual}")
    
    def test_continuous_monitoring(self):
        """測試持續監控"""
        print("\n=== 測試7: 持續監控關鍵地址 ===")
        
        key_addresses = {
            100: "CCD1狀態",
            101: "CCD3狀態", 
            102: "CCD1初始化控制",
            103: "CCD3初始化控制",
            106: "標定文件狀態"
        }
        
        print("開始30秒持續監控...")
        start_time = time.time()
        
        while time.time() - start_time < 30:
            print(f"\n--- {time.time() - start_time:.1f}秒 ---")
            
            for addr, desc in key_addresses.items():
                value = self.read_register(addr)
                print(f"{desc}({addr}): {value}")
            
            # 每10秒觸發一次初始化
            if int(time.time() - start_time) % 10 == 5:
                print(">> 觸發CCD1初始化")
                self.write_register(102, 1)
            
            time.sleep(2)
    
    def test_vm_response_time(self):
        """測試VM響應時間"""
        print("\n=== 測試8: VM響應時間測試 ===")
        
        response_times = []
        
        for i in range(5):
            print(f"\n第{i+1}次響應時間測試:")
            
            # 記錄寫入時間
            write_time = time.time()
            self.write_register(102, 1)
            print(f"寫入時間: {write_time}")
            
            # 輪詢檢測狀態變化
            last_status = 0
            while True:
                current_status = self.read_register(100)
                current_time = time.time()
                
                if current_status != last_status:
                    response_time = (current_time - write_time) * 1000
                    response_times.append(response_time)
                    print(f"狀態變化: {last_status} -> {current_status}")
                    print(f"響應時間: {response_time:.1f}ms")
                    break
                
                if current_time - write_time > 5:  # 5秒超時
                    print("❌ 響應超時")
                    break
                
                last_status = current_status
                time.sleep(0.01)  # 10ms輪詢
            
            time.sleep(3)  # 等待處理完成
        
        if response_times:
            avg_response = sum(response_times) / len(response_times)
            print(f"\n平均響應時間: {avg_response:.1f}ms")
            print(f"最快響應: {min(response_times):.1f}ms")
            print(f"最慢響應: {max(response_times):.1f}ms")
    
    def run_all_tests(self):
        """運行所有測試"""
        print("🔧 開始VM模組完整測試")
        print("=" * 50)
        
        if not self.connected:
            print("❌ 無法連接到Modbus服務器，測試終止")
            return
        
        try:
            # 等待VM啟動
            print("等待VM完全啟動...")
            time.sleep(3)
            
            # 執行各項測試
            self.test_basic_write_read()
            self.test_ccd1_initialization() 
            self.test_ccd3_initialization()
            self.test_multiple_triggers()
            self.test_rapid_writes()
            self.test_mixed_addresses()
            self.test_vm_response_time()
            # self.test_continuous_monitoring()  # 這個測試較長，可選執行
            
            print("\n🎯 測試完成")
            print("=" * 50)
            
        except KeyboardInterrupt:
            print("\n測試被中斷")
        except Exception as e:
            print(f"測試異常: {e}")
        finally:
            if self.client:
                self.client.close()

def main():
    """主程序"""
    print("VM模組測試程式啟動")
    
    # 創建測試客戶端
    test_client = VMTestClient()
    
    # 運行所有測試
    test_client.run_all_tests()

if __name__ == "__main__":
    main()