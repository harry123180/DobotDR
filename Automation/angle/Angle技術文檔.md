# 角度調整系統技術文檔

## 架構概述

角度調整系統實現CCD3視覺檢測與馬達驅動器RTU控制的整合橋接，採用分離式架構設計。

```
主服務器 (ModbusTCP:502)
    |
    |-- TCP --> angle_main.py (TCP Client)
                    |
                    |-- RTU --> 馬達驅動器 (COM5, slave 3)
                    |-- TCP --> CCD3模組 (基地址800)
    |
    |-- TCP --> angle_app.py (獨立TCP Client)
```

## 實現組件

### angle_main.py - 主模組
- TCP Client連接主服務器 (127.0.0.1:502)
- RTU Serial連接馬達驅動器 (COM5, 115200, slave 3)
- 整合CCD3模組 (基地址800) 進行角度檢測
- 寄存器映射基地址: 700
- 狀態機交握處理，50ms輪詢間隔

### angle_app.py - Web控制應用
- 獨立TCP Client連接主服務器 (127.0.0.1:502)
- Flask Web介面 (localhost:5009)
- SocketIO即時通訊支援
- 純ModbusTCP Client實現

### templates/angle_index.html - Web介面
- 簡約白藍漸層設計風格
- 即時狀態監控與控制
- 角度校正結果顯示
- 系統日誌記錄

## 核心功能流程

### 角度校正完整流程
1. **接收控制指令**: PLC發送指令1到寄存器740
2. **CCD3拍照檢測**: 觸發CCD3模組執行角度檢測
3. **角度數據讀取**: 從CCD3寄存器843-844讀取32位角度數據
4. **位置計算**: 按公式 `馬達位置 = 9000 - (CCD3角度 × 10)` 計算
5. **馬達運動**: 發送位置到寄存器6147，發送移動指令8到寄存器125
6. **狀態監控**: 等待馬達運動完成，自動清除指令狀態
7. **結果輸出**: 更新校正結果到寄存器720-739

### 角度計算實例
```
範例1: CCD3檢測角度 = 83.03度
計算: 9000 - (83.03 × 10) = 9000 - 830 = 8170
馬達目標位置: 8170

範例2: CCD3檢測角度 = 87.45度  
計算: 9000 - (87.45 × 10) = 9000 - 875 = 8125
馬達目標位置: 8125
```

## 寄存器映射 (基地址700)

### 狀態寄存器 (只讀 700-714)
| 地址 | 功能 | 數值定義 |
|------|------|----------|
| 700 | 狀態寄存器 | bit0=Ready, bit1=Running, bit2=Alarm, bit3=Initialized, bit4=CCD檢測中, bit5=馬達運動中 |
| 701 | Modbus連接狀態 | 0=斷開, 1=已連接 |
| 702 | 馬達連接狀態 | 0=斷開, 1=已連接 |
| 703 | 錯誤代碼 | 錯誤編號，0=無錯誤 |
| 704 | 操作計數低位 | 32位操作計數低16位 |
| 705 | 操作計數高位 | 32位操作計數高16位 |
| 706 | 錯誤計數 | 累積錯誤次數 |
| 707-714 | 保留 | 未來擴展使用 |

### 檢測結果寄存器 (只讀 720-739)
| 地址 | 功能 | 數值定義 | 說明 |
|------|------|----------|------|
| 720 | 成功標誌 | 0=失敗, 1=成功 | 角度校正結果有效性 |
| 721 | 原始角度高位 | 32位角度高16位 | CCD3檢測角度×100存儲 |
| 722 | 原始角度低位 | 32位角度低16位 | CCD3檢測角度×100存儲 |
| 723 | 角度差高位 | 32位角度差高16位 | 與90度差值×100存儲 |
| 724 | 角度差低位 | 32位角度差低16位 | 與90度差值×100存儲 |
| 725 | 馬達位置高位 | 32位位置高16位 | 計算出的馬達目標位置 |
| 726 | 馬達位置低位 | 32位位置低16位 | 計算出的馬達目標位置 |
| 727-729 | 保留 | - | 未來擴展使用 |
| 730 | 成功次數低位 | 32位成功次數低16位 | 統計資訊 |
| 731 | 成功次數高位 | 32位成功次數高16位 | 統計資訊 |
| 732 | 錯誤次數 | 累積錯誤次數 | 統計資訊 |
| 733 | 運行時間 | 秒單位 | 系統運行時間 |
| 734-739 | 保留 | - | 未來擴展使用 |

### 控制指令寄存器 (讀寫 740)
| 地址 | 功能 | 數值定義 |
|------|------|----------|
| 740 | 控制指令 | 0=清空, 1=角度校正, 2=馬達重置, 7=錯誤重置 |

## 指令映射

| 代碼 | 指令 | 功能說明 |
|------|------|----------|
| 0 | CLEAR | 清除控制指令 |
| 1 | ANGLE_CORRECTION | 執行完整角度校正流程 |
| 2 | MOTOR_RESET | 馬達重置，清除馬達指令狀態 |
| 7 | ERROR_RESET | 錯誤重置，清除系統錯誤狀態 |

## CCD3模組整合

### CCD3觸發流程
```python
def trigger_ccd3_detection(self) -> bool:
    # 發送拍照+角度檢測指令 (CCD3寄存器800, 值16)
    result = self.modbus_client.write_register(
        address=self.ccd3_base_address, value=16, slave=1
    )
    return not result.isError()
```

### CCD3角度讀取
```python
def read_ccd3_angle(self) -> Optional[float]:
    # 讀取CCD3檢測結果寄存器 (843-844: 32位角度)
    result = self.modbus_client.read_holding_registers(
        address=self.ccd3_base_address + 43, count=3, slave=1
    )
    
    registers = result.registers
    success_flag = registers[0]  # 840: 檢測成功標誌
    
    if success_flag == 1:
        # 合併32位角度數據
        angle_high = registers[1]  # 843: 角度高位
        angle_low = registers[2]   # 844: 角度低位
        angle_int = (angle_high << 16) | angle_low
        
        # 處理有符號數值並恢復精度
        if angle_int >= 2**31:
            angle_int -= 2**32
        angle_degrees = angle_int / 100.0
        
        return angle_degrees
    return None
```

## 馬達驅動器RTU控制

### ModbusRTU通訊類
基於paste.txt實現的完整RTU通訊封裝：
- CRC16校驗計算
- 單個寄存器寫入 (功能碼06h)
- 保持寄存器讀取 (功能碼03h)
- 完整的錯誤處理機制

### 馬達控制寄存器
| 寄存器 | 功能 | 說明 |
|--------|------|------|
| 6147 | 目標位置 | 設置馬達目標位置 |
| 125 | 移動指令 | 8=絕對位置移動 |
| 127 | 狀態寄存器 | bit13=運動中, bit5=準備就緒 |

### 馬達指令執行
```python
def send_motor_command(self, position: int) -> bool:
    # 步驟1: 設置目標位置 (寄存器6147)
    success1 = self.motor_rtu.write_single_register(self.motor_slave_id, 6147, position)
    
    # 步驟2: 發送移動指令 (寄存器125, 值8)
    success2 = self.motor_rtu.write_single_register(self.motor_slave_id, 125, 8)
    
    return success1 and success2
```

### 馬達狀態監控
```python
def wait_motor_complete(self, timeout: float = 30.0) -> bool:
    while time.time() - start_time < timeout:
        # 讀取馬達狀態寄存器127
        status = self.motor_rtu.read_holding_registers(self.motor_slave_id, 127, 1)
        status_word = status[0]
        
        moving = bool(status_word & (1 << 13))  # bit 13: 運動中
        ready = bool(status_word & (1 << 5))    # bit 5: 準備就緒
        
        if not moving and ready:
            # 清除馬達指令寄存器
            self.motor_rtu.write_single_register(self.motor_slave_id, 125, 0)
            return True
    return False
```

## 狀態機設計

### AngleSystemStateMachine類
擴展的6位狀態控制：
- **Ready** (bit0): 系統準備接受新指令
- **Running** (bit1): 系統正在執行操作
- **Alarm** (bit2): 系統異常或錯誤
- **Initialized** (bit3): 系統已完全初始化
- **CCD_Detecting** (bit4): CCD正在檢測角度
- **Motor_Moving** (bit5): 馬達正在運動

### 狀態機交握流程
1. **初始狀態**: Ready=1, 其他位=0
2. **接收指令**: PLC寫入740=1
3. **開始執行**: Ready=0, Running=1
4. **CCD檢測階段**: CCD_Detecting=1
5. **馬達運動階段**: CCD_Detecting=0, Motor_Moving=1
6. **執行完成**: Motor_Moving=0, Running=0
7. **等待清零**: PLC寫入740=0
8. **恢復Ready**: Ready=1

### 握手同步實現
```python
def _handshake_sync_loop(self):
    """握手同步循環 - 50ms輪詢"""
    while not self.stop_handshake:
        if self.modbus_client and self.modbus_client.connected:
            # 更新狀態寄存器
            self.write_status_registers()
            
            # 處理控制指令
            self._process_control_commands()
        
        time.sleep(0.05)  # 50ms循環
```

## 32位數值處理

### 角度精度處理
```python
# 角度存儲到寄存器 (保留2位小數)
angle_int = int(angle * 100)
angle_high = (angle_int >> 16) & 0xFFFF  # 高16位 → 721/723寄存器
angle_low = angle_int & 0xFFFF           # 低16位 → 722/724寄存器

# 寄存器恢復為角度值
angle_int = (angle_high << 16) | angle_low
if angle_int >= 2**31:  # 處理有符號數值
    angle_int -= 2**32
final_angle = angle_int / 100.0  # 恢復2位小數精度
```

### 馬達位置處理
```python
# 馬達位置32位存儲
position_high = (position >> 16) & 0xFFFF  # 高16位 → 725寄存器
position_low = position & 0xFFFF           # 低16位 → 726寄存器

# 恢復馬達位置
position = (position_high << 16) | position_low
```

### 操作計數處理
```python
# 操作計數32位存儲
count_high = (operation_count >> 16) & 0xFFFF  # 高16位 → 705/731寄存器
count_low = operation_count & 0xFFFF           # 低16位 → 704/730寄存器
```

## 配置檔案

### angle_config.json (angle_main.py)
```json
{
  "module_id": "Angle_Adjustment_System",
  "modbus_tcp": {
    "host": "127.0.0.1",
    "port": 502,
    "unit_id": 1,
    "timeout": 3.0
  },
  "motor_rtu": {
    "port": "COM5",
    "baudrate": 115200,
    "parity": "N",
    "stopbits": 1,
    "bytesize": 8,
    "timeout": 1.0,
    "slave_id": 3
  },
  "modbus_mapping": {
    "base_address": 700,
    "ccd3_base_address": 800
  },
  "angle_calculation": {
    "target_angle": 90.0,
    "motor_base_position": 9000,
    "angle_multiplier": 10
  },
  "timing": {
    "handshake_interval": 0.05,
    "motor_settle_delay": 0.1,
    "ccd_timeout": 10.0,
    "motor_timeout": 30.0
  }
}
```

### angle_app_config.json (angle_app.py)
```json
{
  "module_id": "Angle_Adjustment_Web_App",
  "modbus_tcp": {
    "host": "127.0.0.1",
    "port": 502,
    "unit_id": 1,
    "timeout": 3.0
  },
  "web_server": {
    "host": "0.0.0.0",
    "port": 5009,
    "debug": false
  },
  "modbus_mapping": {
    "base_address": 700,
    "ccd3_base_address": 800
  },
  "ui_settings": {
    "refresh_interval": 2.0,
    "auto_refresh": true
  }
}
```

## 錯誤處理機制

### 連接管理
- **Modbus TCP自動重連**: 主服務器斷線檢測與重連
- **馬達RTU連接監控**: COM5端口狀態檢測
- **CCD3模組狀態檢查**: 基於寄存器狀態判斷
- **超時控制**: CCD檢測10秒，馬達運動30秒

### 異常處理
- **通訊錯誤計數**: 統計並記錄到寄存器706
- **Alarm狀態設置**: 異常時自動設置Alarm=1
- **錯誤重置機制**: 指令7清除所有錯誤狀態
- **資源釋放**: 異常時確保CCD和馬達狀態清除

### 狀態恢復
```python
def _execute_command_async(self, command: int):
    try:
        if command == ControlCommand.ANGLE_CORRECTION.value:
            result = self.execute_angle_correction()
            self.write_result_registers(result)
        # ... 其他指令處理
    except Exception as e:
        print(f"指令執行錯誤: {e}")
        self.error_count += 1
        self.state_machine.set_alarm(True)
    finally:
        # 確保狀態清除
        self.command_processing = False
        self.state_machine.set_running(False)
        self.state_machine.set_ccd_detecting(False)
        self.state_machine.set_motor_moving(False)
```

## Flask Web介面

### 核心API路由
- **GET /** - Web介面首頁
- **POST /api/modbus/connect** - 連接Modbus服務器
- **POST /api/modbus/disconnect** - 斷開Modbus連接
- **GET /api/status** - 獲取系統狀態和結果
- **POST /api/command/angle_correction** - 執行角度校正
- **POST /api/command/motor_reset** - 馬達重置
- **POST /api/command/error_reset** - 錯誤重置
- **POST /api/command/clear** - 清除指令

### SocketIO即時通訊
- **connect/disconnect** - 連接管理事件
- **status_update** - 狀態資訊推送 (2秒間隔)
- **command_response** - 指令執行結果回應
- **get_status** - 手動狀態查詢

### Web介面特色
- **簡約白藍漸層設計**: 符合專案風格要求
- **即時狀態監控**: 6個狀態位即時顯示
- **角度校正結果**: 檢測角度、角度差、馬達位置顯示
- **計算公式顯示**: 馬達位置計算公式與範例
- **系統日誌**: 即時操作日誌記錄與顯示
- **響應式設計**: 支援桌面和移動設備訪問

## 線程架構

### 主要線程
- **主線程**: Flask Web應用服務
- **握手同步線程**: _handshake_sync_loop (50ms輪詢)
- **指令執行線程**: _execute_command_async (異步執行)
- **狀態監控線程**: _monitor_loop (2秒更新，Web應用)

### 線程安全機制
- **AngleSystemStateMachine**: 使用self.lock保護狀態寄存器
- **指令執行控制**: command_processing標誌防重入
- **ModbusRTU操作**: 序列化RTU通訊，避免衝突
- **daemon模式**: 線程自動回收，程序退出時清理

## 性能特性

### 執行時序
- **CCD3檢測時間**: 通常2-5秒
- **角度計算時間**: <1毫秒
- **馬達運動時間**: 依距離而定，通常5-20秒
- **總流程時間**: 通常10-30秒
- **狀態輪詢頻率**: 50ms (主循環)，2秒 (Web更新)

### 通訊特性
- **Modbus TCP**: 主服務器連接，3秒超時
- **Modbus RTU**: COM5, 115200, 1秒超時
- **CCD3整合**: 基於TCP寄存器操作
- **網路延遲**: <10ms (本地TCP連接)

### 記憶體管理
- **配置檔案**: 按需載入，執行檔同層目錄
- **狀態資料**: 固定大小結構，無動態分配
- **日誌緩存**: Web介面限制100條，自動清理
- **線程資源**: daemon模式自動回收

## 測試驗證

### 連接測試
1. **Modbus TCP連接**: 127.0.0.1:502主服務器
2. **馬達RTU連接**: COM5, 115200, slave 3測試
3. **CCD3模組通訊**: 基地址800寄存器讀寫測試
4. **Web介面訪問**: localhost:5009功能測試

### 功能測試
1. **角度校正流程**: 完整流程端到端測試
2. **角度計算精度**: 數學公式驗證
3. **馬達運動控制**: 位置精度與速度測試
4. **狀態機交握**: Ready/Running/Alarm狀態驗證
5. **錯誤處理機制**: 異常情況恢復測試

### 性能測試
1. **響應時間**: 指令執行到完成的總時間
2. **通訊穩定性**: 長時間運行穩定性
3. **併發處理**: 多指令序列執行測試
4. **記憶體使用**: 長期運行記憶體洩漏檢查

## 部署配置

### 運行順序
1. **啟動主Modbus TCP Server** (端口502)
2. **啟動CCD3角度辨識模組** (必須先運行)
3. **啟動angle_main.py** (角度調整主程序)
4. **啟動angle_app.py** (Web控制介面)
5. **訪問Web介面** (http://localhost:5009)

### 硬體依賴
- **CCD3模組**: 192.168.1.10相機，必須正常運行
- **馬達驅動器**: COM5端口，115200波特率，slave 3
- **主服務器**: 127.0.0.1:502 Modbus TCP服務
- **網路連接**: 本地TCP通訊正常

### 軟體依賴
- **PyModbus 3.9.2**: Modbus TCP/RTU通訊
- **Flask + SocketIO**: Web介面框架
- **PySerial**: RTU串口通訊
- **Python 3.8+**: 運行環境

### 檔案結構
```
Angle/
├── angle_main.py              # 主程序
├── angle_app.py               # Web應用
├── templates/                 # Web模板目錄
│   └── angle_index.html       # Web介面
├── angle_config.json          # 主程序配置 (自動生成)
├── angle_app_config.json      # Web應用配置 (自動生成)
└── README.md                  # 模組說明
```

## 與其他模組的關係

### CCD3模組依賴 (基地址800)
- **必要前置條件**: CCD3模組必須正常運行
- **通訊方式**: 透過主服務器寄存器操作
- **數據流向**: CCD3 → 角度調整系統
- **觸發方式**: 角度調整系統主動觸發CCD3檢測

### 馬達驅動器整合
- **通訊協議**: 獨立RTU連接，不依賴其他模組
- **控制方式**: 直接RTU指令控制
- **位置精度**: 整數位置值，適用於步進/伺服馬達
- **狀態反饋**: 即時運動狀態監控

### 主服務器整合
- **基地址分配**: 700-749，避免衝突
- **寄存器映射**: 完全遵循專案寄存器規範
- **狀態機協議**: 標準Ready/Running/Alarm交握
- **併發支援**: 與其他模組並行運行

## 故障排除

### 常見問題
1. **CCD3連接失敗**: 檢查CCD3模組(基地址800)是否正常運行
2. **馬達RTU連接失敗**: 檢查COM5端口權限和波特率設置
3. **角度檢測失敗**: 確認CCD3能正常檢測到Ring物件
4. **馬達運動異常**: 檢查馬達驅動器電源和連接線
5. **Web介面無法訪問**: 確認端口5009未被占用

### 除錯方法
1. **主程序日誌**: angle_main.py輸出詳細執行日誌
2. **Web狀態監控**: localhost:5009即時狀態顯示
3. **寄存器檢查**: 透過主服務器直接讀取寄存器700-740
4. **分步測試**: 分別測試CCD3檢測和馬達運動功能

### 錯誤恢復
1. **手動錯誤重置**: Web介面「錯誤重置」按鈕
2. **系統重啟**: 重新啟動angle_main.py程序
3. **馬達重置**: Web介面「馬達重置」清除馬達狀態
4. **指令清除**: Web介面「清除指令」手動清零

## 開發擴展

### 預留擴展點
1. **角度計算公式**: 配置檔案中可調整計算參數
2. **多馬達支援**: 架構支援擴展到多個馬達控制
3. **角度精度**: 已支援0.01度精度，可進一步提升
4. **檢測模式**: 可整合更多CCD3檢測模式

### 模組化設計
- **ModbusRTU類**: 獨立的RTU通訊封裝，可重用
- **AngleSystemStateMachine**: 獨立的狀態機實現
- **配置檔案管理**: 標準化配置載入機制
- **Web UI框架**: 可複用的Web介面設計模式

### API擴展
```python
# 未來可擴展的API接口
class AngleAdjustmentService:
    def set_angle_target(self, target_angle: float)  # 可調整目標角度
    def set_motor_speed(self, speed: int)            # 可調整馬達速度
    def get_angle_history(self) -> List[float]       # 角度歷史記錄
    def calibrate_angle_offset(self, offset: float)  # 角度校準偏移
```

## 版本歷史

### v1.0 (2024-12-XX) - 初始版本
- **基礎功能**: CCD3拍照 → 角度計算 → 馬達補正完整流程
- **寄存器映射**: 700-749完整寄存器定義
- **狀態機**: 6位狀態機設計與實現
- **Web介面**: 簡約白藍漸層設計
- **硬體整合**: CCD3模組 + 馬達驅動器整合
- **計算公式**: 馬達位置 = 9000 - (CCD3角度 × 10)
- **精度支援**: 角度保留2位小數精度

### 開發原則
- **無空泛內容**: 技術文檔基於實際代碼實現
- **實事求是**: 僅描述已實現功能，不誇大
- **技術導向**: 專注技術實現細節和架構設計
- **維護性**: 代碼結構清晰，便於後續維護擴展

---

**文檔版本**: v1.0  
**創建日期**: 2024-12-XX  
**最後更新**: 2024-12-XX  
**負責人**: 開發團隊

**相關文檔**:
- README.MD (系統整體架構)
- CCD3技術文檔.md (CCD3角度辨識模組)
- angle_main.py (主程序源碼)
- angle_app.py (Web應用源碼)