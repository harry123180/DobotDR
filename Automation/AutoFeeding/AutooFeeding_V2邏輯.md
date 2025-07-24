# AutoFeeding模組增強功能技術文檔

## 修改概要

基於現有AutoFeeding模組，新增外部控制能力、參數動態調整、自動參數初始化和記憶體監控功能。

## 新增功能清單

### 1. 外部控制功能
- **控制地址**: 920寄存器
- **控制邏輯**: 0=停止AutoFeeding, 1=啟動AutoFeeding
- **實現方式**: 主循環中檢查920寄存器值變化，動態啟動/停止檢測

### 2. VP震動參數可調
- **參數地址**: 930-949寄存器
- **包含參數**: 兩組震動動作+停止參數
- **實現方式**: 每次VP操作前從寄存器讀取最新參數

### 3. Flow4直振參數可調
- **參數地址**: 950-959寄存器
- **包含參數**: 脈衝持續時間、脈衝間隔
- **實現方式**: 每次Flow4操作前從寄存器讀取最新參數

### 4. 系統參數可調
- **參數地址**: 980-999寄存器
- **包含參數**: 檢測週期、超時設定、閾值設定等
- **實現方式**: 每個檢測週期開始時讀取最新參數

### 5. 自動參數初始化
- **觸發條件**: 檢測到關鍵寄存器(930-939, 950-951, 980-989)全為0時
- **初始化內容**: 硬編碼預設參數寫入對應寄存器
- **執行時機**: 連接Modbus服務器成功後

### 6. 記憶體使用量監控
- **監控地址**: 906寄存器
- **更新頻率**: 每10分鐘
- **數值格式**: MB單位，無條件捨去小數點後的整數
- **實現方式**: 獨立執行緒使用psutil獲取記憶體使用量

## 核心程式碼修改

### 地址重新分配
```python
# 座標交握寄存器調整為960-979
self.COORD_START = 960
# DR_F可用標誌: 960
# X座標: 961(高位), 962(低位)  
# Y座標: 963(高位), 964(低位)
# 已讀取標誌: 965
```

### 硬編碼預設參數
```python
# VP震動參數(兩組動作)
DEFAULT_VP_PARAMS = {
    "action1": {"action_code": 4, "strength": 45, "frequency": 43, "duration": 800},
    "action2": {"action_code": 11, "strength": 98, "frequency": 64, "duration": 800},
    "stop": {"command_code": 3, "delay": 100}
}

# Flow4直振參數
DEFAULT_FLOW4_PARAMS = {
    "pulse_duration": 100,    # 0.1秒
    "pulse_interval": 50      # 0.05秒
}

# 系統運行參數
DEFAULT_SYSTEM_PARAMS = {
    "cycle_interval": 1000,         # 1秒檢測週期
    "ccd1_timeout": 5000,           # 5秒CCD1超時
    "progress_threshold": 44,       # 進度阻擋閾值
    "large_count_threshold": 5,     # 異常檢測大量閾值
    "sudden_drop_threshold": 1,     # 異常檢測驟減閾值
    # ... 其他參數
}
```

### 初始化檢測邏輯
```python
def check_need_initialization(self) -> bool:
    """檢查關鍵寄存器是否全為0"""
    vp_params_zero = all(self.read_register(930 + i) == 0 for i in range(10))
    flow4_params_zero = all(self.read_register(950 + i) == 0 for i in range(2))
    system_params_zero = all(self.read_register(980 + i) == 0 for i in range(10))
    
    return vp_params_zero and flow4_params_zero and system_params_zero
```

### 參數動態讀取
```python
def read_vp_params(self) -> Dict[str, Any]:
    """從寄存器讀取VP參數並進行範圍驗證"""
    params = {
        "action1": {
            "action_code": max(0, min(11, self.read_register(930) or 4)),
            "strength": max(0, min(100, self.read_register(931) or 45)),
            # ... 其他參數
        }
    }
    return params
```

### 記憶體監控執行緒
```python
def _memory_monitor_worker(self):
    """記憶體監控工作執行緒"""
    while self.memory_monitor_running:
        try:
            process = psutil.Process()
            memory_bytes = process.memory_info().rss
            memory_mb = int(memory_bytes / (1024 * 1024))  # 無條件捨去小數點
            
            self.write_register(906, memory_mb)
            
            # 等待10分鐘
            for _ in range(600):
                if not self.memory_monitor_running:
                    break
                time.sleep(1)
        except Exception as e:
            self.logger.error(f"記憶體監控異常: {e}")
            time.sleep(60)
```

### 外部控制邏輯
```python
def check_external_control(self) -> bool:
    """檢查外部控制狀態"""
    control_value = self.read_register(920)
    should_run = (control_value == 1)
    
    if should_run != self.external_control_running:
        self.external_control_running = should_run
        if should_run:
            self.logger.info("外部控制啟動AutoFeeding (920=1)")
        else:
            self.logger.info("外部控制停止AutoFeeding (920=0)")
    
    return should_run
```

## 操作流程變更

### 原有流程
1. 程序啟動自動開始檢測
2. 參數來自配置檔案
3. 無外部控制能力

### 新增流程  
1. 程序啟動後檢查參數初始化需求
2. 自動寫入硬編碼預設參數(如需要)
3. 等待外部控制信號(920=1)啟動檢測
4. 運行過程中參數可動態調整
5. 記憶體使用量持續監控

## 寄存器使用規劃

### 狀態寄存器 (900-919) - 只讀
| 地址 | 功能 | 原有/新增 |
|------|------|----------|
| 900-905 | 原有狀態統計 | 原有 |
| 906 | 記憶體使用量(MB) | 新增 |
| 907-910 | 原有狀態 | 原有 |

### 控制寄存器 (920-929) - 讀寫
| 地址 | 功能 | 說明 |
|------|------|------|
| 920 | 運行控制 | 0=停止, 1=啟動 |
| 921-929 | 保留 | 預留擴展 |

### VP參數寄存器 (930-949) - 讀寫
| 地址 | 功能 | 預設值 |
|------|------|--------|
| 930-933 | 第一組震動參數 | 4,45,43,800 |
| 934-937 | 第二組震動參數 | 11,98,64,800 |
| 938-939 | 停止參數 | 3,100 |

### Flow4參數寄存器 (950-959) - 讀寫
| 地址 | 功能 | 預設值 |
|------|------|--------|
| 950 | 脈衝持續時間(ms) | 100 |
| 951 | 脈衝間隔(ms) | 50 |

### 座標交握寄存器 (960-979) - 混合讀寫
| 地址 | 功能 | 說明 |
|------|------|------|
| 960 | DR_F可用標誌 | 調整自原940 |
| 961-964 | DR_F座標 | 調整自原941-944 |
| 965 | 座標已讀取標誌 | 調整自原945 |

### 系統參數寄存器 (980-999) - 讀寫
| 地址 | 功能 | 預設值 |
|------|------|--------|
| 980 | 檢測週期間隔(ms) | 1000 |
| 981 | CCD1超時(ms) | 5000 |
| 982-989 | 其他系統參數 | 各種閾值設定 |

## 使用範例

### 外部控制AutoFeeding
```python
from pymodbus.client import ModbusTcpClient

client = ModbusTcpClient('127.0.0.1', port=502)
client.connect()

# 啟動AutoFeeding
client.write_register(920, 1, slave=1)

# 檢查運行狀態
status = client.read_holding_registers(900, 1, slave=1).registers[0]
print(f"AutoFeeding狀態: {status}")

# 調整VP第一組強度為70
client.write_register(931, 70, slave=1)

# 調整檢測週期為2秒
client.write_register(980, 2000, slave=1)

# 檢查記憶體使用量
memory_mb = client.read_holding_registers(906, 1, slave=1).registers[0]
print(f"記憶體使用量: {memory_mb} MB")

# 停止AutoFeeding
client.write_register(920, 0, slave=1)

client.close()
```

### Flow1座標讀取
```python
# Flow1程序中讀取座標(座標地址已調整)
def read_autofeeding_coordinates():
    dr_f_available = read_register(960)  # 調整自940
    if dr_f_available == 1:
        x = read_32bit_register(961, 962)  # 調整自941,942
        y = read_32bit_register(963, 964)  # 調整自943,944
        
        write_register(965, 1)  # 調整自945
        return (x, y)
    return None
```

## 重要技術細節

### 參數驗證機制
所有從寄存器讀取的參數都進行範圍驗證，超出範圍時使用預設值並記錄警告。

### 記憶體監控精度
使用`int(memory_bytes / (1024 * 1024))`確保無條件捨去小數點，符合要求。

### 執行緒安全
記憶體監控執行緒使用daemon模式，程序退出時自動清理。

### 錯誤處理
所有新增功能都包含完整的異常處理，不影響原有檢測邏輯穩定性。

### 向後相容性
原有的檢測邏輯和API保持不變，新功能為額外增強，不破壞既有整合。

## 部署注意事項

1. **依賴套件**: 需要安裝`psutil`套件進行記憶體監控
2. **寄存器地址**: 座標交握地址從940-945調整為960-965，Flow1程序需要相應修改
3. **初始化檢查**: 首次運行會自動初始化參數，後續運行保持寄存器值
4. **外部控制**: 預設狀態為停止(920=0)，需要外部設置920=1才會開始檢測
5. **記憶體監控**: 獨立執行緒，不影響主要檢測功能

## 測試驗證

### 功能測試項目
1. 外部控制啟動/停止功能
2. 參數動態調整生效驗證
3. 自動初始化功能測試
4. 記憶體監控準確性
5. 原有檢測邏輯不受影響
6. 座標交握功能正常

### 性能測試
1. 記憶體使用量是否穩定
2. 參數讀取對檢測週期影響
3. 執行緒資源消耗檢查

這些增強功能提供了完整的外部控制能力，同時保持了系統的穩定性和可維護性。