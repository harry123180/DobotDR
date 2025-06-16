# Dobot機械臂狀態機交握設計 (基地址400-449)

## 寄存器映射規劃

### 狀態寄存器 (400-419) - 只讀
| 地址 | 功能 | 數值定義 | 二進制位定義 |
|------|------|----------|--------------|
| 400 | 主狀態寄存器 | bit0=Ready, bit1=Running, bit2=Alarm, bit3=Initialized | 狀態機核心控制位 |
| 401 | 機械臂狀態 | 0=離線, 1=閒置, 2=執行中, 3=錯誤, 4=緊急停止 | 機械臂運行狀態 |
| 402 | 當前流程ID | 0=無, 1=Flow1(取料), 2=Flow2(出料), 3=Flow3(完整) | 正在執行的流程 |
| 403 | 流程執行進度 | 0-100百分比 | 當前流程完成度 |
| 404 | 錯誤代碼 | 0=無錯誤, >0=錯誤類型 | 具體錯誤資訊 |
| 405 | 機械臂模式 | 機械臂實際模式值 | 來自GetRobotMode() |
| 406 | 當前X座標 | 位置座標(mm) | 實時位置資訊 |
| 407 | 當前Y座標 | 位置座標(mm) | 實時位置資訊 |
| 408 | 當前Z座標 | 位置座標(mm) | 實時位置資訊 |
| 409 | 當前R座標 | 旋轉角度 | 實時姿態資訊 |
| 410 | 關節J1角度 | 角度值×100 | 關節1實時角度 |
| 411 | 關節J2角度 | 角度值×100 | 關節2實時角度 |
| 412 | 關節J3角度 | 角度值×100 | 關節3實時角度 |
| 413 | 關節J4角度 | 角度值×100 | 關節4實時角度 |
| 414 | 數位輸入狀態 | DI狀態位 | 輸入IO狀態 |
| 415 | 數位輸出狀態 | DO狀態位 | 輸出IO狀態 |
| 416 | 操作計數器 | 累積成功次數 | 統計資訊 |
| 417 | 錯誤計數器 | 累積錯誤次數 | 統計資訊 |
| 418 | 運行時間 | 分鐘數 | 系統運行時間 |
| 419 | 保留狀態 | - | 未來擴展 |

### 控制寄存器 (440-449) - 讀寫
| 地址 | 功能 | 數值定義 | 說明 |
|------|------|----------|------|
| 440 | VP視覺取料控制 | 0=清空, 1=啟動Flow1 | VP取料流程觸發 |
| 441 | 出料控制 | 0=清空, 1=啟動Flow2 | 出料流程觸發 |
| 442 | 清除警報控制 | 0=無動作, 1=清除Alarm | 錯誤重置功能 |
| 443 | 緊急停止控制 | 0=正常, 1=緊急停止 | 安全停止功能 |
| 444 | 手動指令 | Flow ID或特殊指令 | Web端手動控制 |
| 445-449 | 保留控制 | - | 未來擴展使用 |

## 狀態機核心邏輯 (地址400二進制位)

### 狀態位定義
```
地址400的二進制位元控制:
bit0 = Ready      (系統準備就緒，可接受新指令)
bit1 = Running    (系統執行中，拒絕新指令)
bit2 = Alarm      (系統警報狀態，需要重置)
bit3 = Initialized (系統已完成初始化)
bit4-15 = 保留   (未來擴展使用)
```

### 狀態組合解讀
| 二進制 | 十進制 | Ready | Running | Alarm | 狀態說明 |
|--------|--------|-------|---------|-------|----------|
| 00001001 | 9 | 1 | 0 | 0 | 系統Ready，可接受指令 |
| 00001010 | 10 | 0 | 1 | 0 | 系統執行中，拒絕新指令 |
| 00001100 | 12 | 0 | 0 | 1 | 系統警報，需要重置 |
| 00001000 | 8 | 0 | 0 | 0 | 系統空閒，等待狀態更新 |

## 交握協議流程設計

### Flow1 (VP視覺取料) 完整流程
```
1. PLC檢查: 讀取400寄存器 = 9 (Ready=1)
2. PLC觸發: 寫入440=1 (VP視覺取料控制)
3. 系統響應: 400寄存器 = 10 (Ready=0, Running=1)
4. 系統執行: 402寄存器 = 1 (Flow1執行中)
5. Flow1完成: 
   - 成功: 400寄存器 = 8 (Ready=0, Running=0, 等待清零)
   - 失敗: 400寄存器 = 12 (Alarm=1)
6. PLC清零: 寫入440=0
7. 系統恢復: 400寄存器 = 9 (Ready=1)
```

### Flow2 (出料流程) 聯動控制
```
前提條件: 400寄存器bit0=1 (Ready=1) 且 440=0 (取料控制已清零)

1. PLC檢查: 讀取400寄存器 = 9 (Ready=1)
2. PLC觸發: 寫入441=1 (出料控制)
3. 系統響應: 400寄存器 = 10 (Ready=0, Running=1)
4. 系統執行: 402寄存器 = 2 (Flow2執行中)
5. Flow2完成:
   - 成功: 400寄存器 = 8 (Ready=0, Running=0)
   - 失敗: 400寄存器 = 12 (Alarm=1)
6. PLC清零: 寫入441=0
7. 系統恢復: 400寄存器 = 9 (Ready=1)
```

### 警報處理流程
```
當400寄存器bit2=1 (Alarm=1)時:
1. PLC檢測: 400寄存器 = 12 (Alarm=1)
2. PLC重置: 寫入442=1 (清除警報控制)
3. 系統清除: 400寄存器bit2=0 (Alarm清除)
4. PLC確認: 寫入442=0
5. 系統恢復: 400寄存器 = 9 (Ready=1)
```

## 錯誤觸發條件 (Alarm=1)

### Flow執行錯誤
- 機械臂連接失敗
- 夾爪操作失敗
- 運動控制異常
- 點位不存在

### CCD檢測錯誤
- CCD1視覺檢測無物體
- CCD3角度檢測失敗
- 通訊超時

### 系統錯誤
- Modbus通訊異常
- 設備初始化失敗
- 緊急停止觸發

## 實現代碼修改重點

### 1. DobotStateMachine類擴展
```python
class DobotStateMachine:
    def __init__(self, modbus_client: ModbusTcpClient):
        self.modbus_client = modbus_client
        self.status_register = 0b1001  # 初始: Ready=1, Initialized=1
        
    def set_ready(self, ready: bool):
        if ready:
            self.status_register |= (1 << 0)   # 設置Ready位
            self.status_register &= ~(1 << 1)  # 清除Running位
        else:
            self.status_register &= ~(1 << 0)  # 清除Ready位
    
    def set_running(self, running: bool):
        if running:
            self.status_register |= (1 << 1)   # 設置Running位
            self.status_register &= ~(1 << 0)  # 清除Ready位
        else:
            self.status_register &= ~(1 << 1)  # 清除Running位
    
    def set_alarm(self, alarm: bool):
        if alarm:
            self.status_register |= (1 << 2)   # 設置Alarm位
            self.status_register &= ~(1 << 0)  # 清除Ready位
            self.status_register &= ~(1 << 1)  # 清除Running位
        else:
            self.status_register &= ~(1 << 2)  # 清除Alarm位
```

### 2. 握手循環修改
```python
def _handshake_loop(self):
    last_vp_command = 0
    last_unload_command = 0
    last_clear_alarm = 0
    
    while self.is_running:
        try:
            # 讀取控制寄存器
            vp_control = self.read_register(440)      # VP視覺取料控制
            unload_control = self.read_register(441)  # 出料控制
            clear_alarm = self.read_register(442)     # 清除警報控制
            
            # 處理VP視覺取料指令
            if vp_control == 1 and last_vp_command != 1:
                if self.is_ready():
                    self.execute_flow1_async()
                last_vp_command = 1
            elif vp_control == 0:
                last_vp_command = 0
            
            # 處理出料指令 (需要Ready且VP控制已清零)
            if unload_control == 1 and last_unload_command != 1:
                if self.is_ready() and vp_control == 0:
                    self.execute_flow2_async()
                last_unload_command = 1
            elif unload_control == 0:
                last_unload_command = 0
            
            # 處理警報清除
            if clear_alarm == 1 and last_clear_alarm != 1:
                self.clear_alarm_state()
                last_clear_alarm = 1
            elif clear_alarm == 0:
                last_clear_alarm = 0
            
            # 更新狀態到PLC
            self.update_status_to_plc()
            
            time.sleep(0.05)  # 50ms循環
            
        except Exception as e:
            print(f"握手循環錯誤: {e}")
            time.sleep(1)
```

### 3. Flow執行完成處理
```python
def execute_flow1_async(self):
    """異步執行Flow1"""
    threading.Thread(target=self._flow1_execution_thread, daemon=True).start()

def _flow1_execution_thread(self):
    try:
        self.set_running(True)
        self.set_current_flow(1)  # 更新402寄存器
        
        # 執行Flow1
        result = self.flows[1].execute()
        
        if result.success:
            # Flow1成功完成，設置為等待清零狀態
            self.set_running(False)  # Running=0, Ready保持=0
            print("Flow1執行成功，等待PLC清零VP控制指令")
        else:
            # Flow1失敗，設置警報
            self.set_alarm(True)
            print(f"Flow1執行失敗: {result.error_message}")
            
    except Exception as e:
        print(f"Flow1執行異常: {e}")
        self.set_alarm(True)
    finally:
        self.set_current_flow(0)  # 清除402寄存器

def _flow2_execution_thread(self):
    """Flow2執行線程 - 類似Flow1"""
    try:
        self.set_running(True)
        self.set_current_flow(2)
        
        result = self.flows[2].execute()
        
        if result.success:
            self.set_running(False)
            print("Flow2執行成功，等待PLC清零出料控制指令")
        else:
            self.set_alarm(True)
            print(f"Flow2執行失敗: {result.error_message}")
            
    except Exception as e:
        print(f"Flow2執行異常: {e}")
        self.set_alarm(True)
    finally:
        self.set_current_flow(0)
```

## Web端相容性保證

### 手動控制寄存器 (444)
```python
# Web端可以使用444寄存器進行手動控制
# 不影響440/441的自動化流程控制
def handle_web_manual_command(self, command):
    if command == 1:  # 手動Flow1
        if self.is_ready():
            self.execute_flow1_async()
    elif command == 2:  # 手動Flow2
        if self.is_ready():
            self.execute_flow2_async()
```

## PLC操作指南

### VP視覺取料操作 (Flow1)
```
1. 檢查準備: 讀取400 = 9 (00001001)
2. 啟動取料: 寫入440 = 1
3. 監控執行: 讀取400 = 10 (00001010), 402 = 1
4. 等待完成: 讀取400 = 8 (00001000), 402 = 0
5. 清零指令: 寫入440 = 0
6. 確認恢復: 讀取400 = 9 (00001001)
```

### 出料流程操作 (Flow2)
```
前提: 400 = 9 且 440 = 0 (取料已完成且已清零)
1. 檢查準備: 讀取400 = 9, 440 = 0
2. 啟動出料: 寫入441 = 1
3. 監控執行: 讀取400 = 10, 402 = 2
4. 等待完成: 讀取400 = 8, 402 = 0
5. 清零指令: 寫入441 = 0
6. 確認恢復: 讀取400 = 9
```

### 警報處理操作
```
1. 檢測警報: 讀取400 = 12 (00001100)
2. 清除警報: 寫入442 = 1
3. 確認清除: 讀取400 = 9 (Alarm位清除)
4. 清零指令: 寫入442 = 0
```

## 關鍵特點

1. **聯動控制**: Flow2只有在Flow1完成且控制指令清零後才能啟動
2. **狀態隔離**: Ready位控制整體可用性，Running位防止並發執行
3. **錯誤保護**: 任何執行失敗都會觸發Alarm，需要手動重置
4. **Web相容**: 通過444寄存器實現Web端手動控制，不影響自動化流程
5. **完整握手**: 確保PLC與機械臂系統的可靠通訊和狀態同步