# DR版本AutoFeeding獨立模組協調邏輯

## 核心設計理念

### **設計目標**
- **持續供料**: 確保DR保護區域內始終有DR_F可用
- **主動監控**: 直接監控Flow1控制地址(1201)，無需被動等待暫停指令
- **簡化交握**: Flow1直接讀取座標(940-944)，不需要透過AutoProgram中轉
- **自動運行**: 啟動後自動開始檢測，無需外部觸發

### **主要改進**
- **持續檢測**: 找到DR_F後不暫停，繼續檢測確保保護區域隨時有料件
- **Flow1狀態監控**: 監控1201寄存器，當值為1時暫停自動進料程序
- **矩形保護區域**: 適應DR專案的矩形保護區域判斷
- **YOLO檢測適配**: 支援DR專案的三分類YOLO檢測系統

## DR保護區域定義

### **保護區域座標**
```python
# DR保護區域四點矩形座標
x_min = -112.0  # 左邊界
x_max = -4.0    # 右邊界  
y_min = 243.0   # 下邊界
y_max = 339.21  # 上邊界
```

### **保護區域判斷邏輯**
```python
def is_point_in_rect(x: float, y: float) -> bool:
    """判斷點是否在DR保護區域矩形內"""
    return x_min <= x <= x_max and y_min <= y <= y_max
```

## Flow1監控邏輯

### **Flow1狀態監控**
```python
def check_flow1_status(self) -> bool:
    """主動監控Flow1控制狀態"""
    current_motion_flow = self.read_register(1201)  # 當前運動Flow寄存器
    flow1_now_active = (current_motion_flow == 1)
    
    if flow1_now_active != self.flow1_active:
        if flow1_now_active:
            print("檢測到Flow1啟動，暫停檢測")
            self.status = AutoFeedingStatus.FLOW1_PAUSED
        else:
            print("檢測到Flow1停止，恢復檢測")
            self.status = AutoFeedingStatus.RUNNING
```

### **監控時序**
```
AutoFeeding監控循環 (0.1秒間隔):
1. 檢查1201寄存器值
2. 1201=1 → 立即暫停檢測
3. 1201=0 → 立即恢復檢測
4. 全程無需外部控制
```

## YOLO檢測適配

### **DR專案二分類檢測**
- **檢測對象**: DR_F (正面物件) 和 STACK (堆疊物件)
- **檢測模式**: YOLOv11二分類檢測
- **目標物件**: 僅關注DR_F，STACK用於總數量統計

### **DR專案檢測結果寄存器**
| 地址 | 功能 | 說明 |
|------|------|------|
| 240 | DR_F檢測數量 | 0-5個DR_F |
| 242 | STACK檢測數量 | 0-255個STACK |
| 243 | 總檢測數量 | DR_F + STACK |
| 261-280 | DR_F世界座標 | 每個DR_F佔4個寄存器(X高位、X低位、Y高位、Y低位) |

### **檢測流程**
```python
def trigger_ccd1_detection(self) -> CCD1DetectionResult:
    """觸發CCD1檢測 - 適配DR專案的YOLO檢測"""
    # 1. 發送拍照+檢測指令
    self.write_register(200, 16)
    
    # 2. 等待檢測完成
    while timeout_not_reached:
        if capture_complete and detect_complete and operation_success:
            break
    
    # 3. 讀取DR YOLO檢測結果
    dr_f_count = self.read_register(240)        # DR_F數量
    stack_count = self.read_register(242)       # STACK數量
    total_detections = self.read_register(243)  # 總檢測數量 (DR_F + STACK)
    
    # 4. 提取DR_F世界座標 (261-280)
    for i in range(min(dr_f_count, 5)):
        base_addr = 261 + (i * 4)
        world_x = self.read_32bit_register(base_addr, base_addr + 1)
        world_y = self.read_32bit_register(base_addr + 2, base_addr + 3)
```

## 簡化交握協議

### **新版直接交握**
```python
# AutoFeeding設置DR_F可用
def set_dr_f_available(self, coords: Tuple[float, float]):
    self.dr_f_available = True
    self.dr_f_coords = coords
    
    # 立即更新寄存器讓Flow1直接讀取
    self.write_register(940, 1)  # DR_F可用標誌
    self.write_32bit_register(941, 942, coords[0])  # X座標
    self.write_32bit_register(943, 944, coords[1])  # Y座標

# Flow1直接讀取座標並確認
def check_coords_taken(self):
    coords_taken = self.read_register(945)  # Flow1設置此標誌
    if coords_taken == 1:
        # 清除DR_F狀態，繼續檢測新的
        self.dr_f_available = False
        self.write_register(940, 0)
        self.write_register(945, 0)
```

### **交握流程**
```
AutoFeeding ↔ Flow1 直接交握:
1. AF找到DR_F → 設置940=1, 941-944=座標
2. Flow1檢查940=1 → 讀取941-944座標
3. Flow1讀取完成 → 設置945=1確認
4. AF檢測到945=1 → 清除狀態，繼續檢測
```

## 寄存器映射

### **DR_F狀態寄存器 (940-959)**
| 地址 | 功能 | 讀寫方 | 說明 |
|------|------|--------|------|
| 940 | DR_F可用標誌 | AF寫入, Flow1讀取 | 0=無DR_F, 1=有DR_F可取 |
| 941 | X座標高位 | AF寫入, Flow1讀取 | 32位世界座標X高16位 |
| 942 | X座標低位 | AF寫入, Flow1讀取 | 32位世界座標X低16位 |
| 943 | Y座標高位 | AF寫入, Flow1讀取 | 32位世界座標Y高16位 |
| 944 | Y座標低位 | AF寫入, Flow1讀取 | 32位世界座標Y低16位 |
| 945 | 座標已讀取標誌 | Flow1寫入, AF讀取 | Flow1確認已讀取座標 |

### **Flow1控制寄存器**
| 地址 | 功能 | 控制方 | 說明 |
|------|------|--------|------|
| 1201 | 當前執行Flow | Dobot_main | AF主動監控此地址 |

### **AutoFeeding狀態寄存器 (900-919)**
| 地址 | 功能 | 說明 |
|------|------|------|
| 900 | 模組狀態 | 0=停止, 1=運行, 2=Flow1暫停, 3=檢測中, 4=VP震動, 5=錯誤 |
| 901 | 週期計數 | 累積檢測週期數 |
| 902 | DR_F找到次數 | 累積找到DR_F次數 |
| 903 | Flow4觸發次數 | 累積送料次數 |
| 904 | VP震動次數 | 累積震動次數 |
| 907 | 錯誤代碼 | 0=無錯誤, >0=錯誤類型 |
| 908 | 操作狀態 | 0=閒置, 1=CCD檢測, 2=VP控制, 3=Flow4觸發 |
| 909 | Flow1監控狀態 | 0=Flow1未執行, 1=Flow1執行中 |

## 檢測邏輯流程

### **入料檢測週期**
```python
def feeding_cycle(self) -> bool:
    """執行一次入料檢測週期"""
    # 1. 檢查模組狀態
    if not self.check_modules_status():
        return False
    
    # 2. CCD1檢測
    detection_result = self.trigger_ccd1_detection()
    
    # 3. 尋找保護區域內的DR_F
    target_coords = self.find_dr_f_in_protection_zone(detection_result)
    
    if target_coords:
        # 找到DR_F - 設置可用狀態，繼續檢測
        self.set_dr_f_available(target_coords)
        
    elif detection_result.total_detections < 4:
        # 料件不足 - 觸發Flow4送料
        self.trigger_flow4_feeding()
        
    else:
        # 料件充足但無正面 - VP震動重檢
        self.trigger_vp_vibration()
        # 震動後立即重新檢測
        retry_result = self.trigger_ccd1_detection()
        retry_coords = self.find_dr_f_in_protection_zone(retry_result)
        if retry_coords:
            self.set_dr_f_available(retry_coords)
```

### **檢測決策邏輯**
```
檢測結果分析:
├── 保護區內有DR_F → 設置可用狀態，繼續檢測
├── 總料件數量<4 → Flow4送料
└── 料件充足但無正面 → VP震動重檢
    └── 震動後重檢 → 找到DR_F則設置可用
```

## 執行時序圖

### **正常運作流程**
```
時間軸  DR AutoFeeding          Flow1              Dobot_main
  |         |                     |                     |
  t1    啟動檢測循環                |                     |
  |         |                     |                     |
  t2    找到DR_F                  |                     |
  |     設置940=1,941-944         |                     |
  |         |                     |                     |
  t3        |              檢查940=1                   |
  |         |              讀取941-944                 |
  |         |              設置945=1                   |
  |         |                     |                     |
  t4    檢測到945=1               |           寫入1201=1
  |     清除DR_F狀態              |                     |
  |         |                     |                     |
  t5    檢測到1201=1              |                     |
  |     暫停檢測                    |              Flow1執行中
  |         |                     |                     |
  t6        |                     |           寫入1201=0
  |         |                     |                     |
  t7    檢測到1201=0              |                     |
  |     恢復檢測                    |                     |
  |         |                     |                     |
  t8    繼續檢測新DR_F            |                     |
```

## 配置參數

### **檢測參數配置**
```json
{
  "autofeeding": {
    "cycle_interval": 1.0,        // 檢測週期1秒
    "ccd1_timeout": 5.0,          // CCD1超時5秒
    "flow4_consecutive_limit": 5,  // 連續直振限制
    "vp_empty_check_count": 3,    // VP空盤檢查次數
    "auto_start": true            // 自動啟動檢測
  }
}
```

### **VP震動參數配置**
```json
{
  "vp_params": {
    "spread_action_code": 11,     // 震動動作碼
    "spread_strength": 60,        // 震動強度60
    "spread_frequency": 50,       // 震動頻率50Hz
    "spread_duration": 0.3,       // 震動持續0.3秒
    "stop_command_code": 3,       // 停止指令碼
    "stop_delay": 0.1             // 停止延遲
  }
}
```

### **時序控制配置**
```json
{
  "timing": {
    "command_delay": 0.05,        // 指令延遲
    "status_check_interval": 0.05, // 狀態檢查間隔
    "register_clear_delay": 0.02,  // 寄存器清除延遲
    "vp_stabilize_delay": 0.15,    // VP穩定延遲
    "flow1_check_interval": 0.1    // Flow1監控間隔
  }
}
```

## 使用方式

### **啟動AutoFeeding**
```bash
# 進入DR AutoFeeding目錄
cd Automation/AutoFeeding/

# 啟動DR AutoFeeding獨立模組
python AutoFeeding_main.py
```

### **AutoFeeding自動行為**
```
啟動後自動執行:
✓ 連接Modbus服務器 (127.0.0.1:502)
✓ 初始化寄存器
✓ 自動開始檢測循環
✓ 主動監控Flow1狀態
✓ 持續確保保護區域有DR_F
```

### **Flow1使用方式**
```python
# Flow1程序中讀取座標
def read_autofeeding_coordinates():
    # 檢查DR_F是否可用
    dr_f_available = read_register(940)
    if dr_f_available == 1:
        # 讀取32位座標
        x = read_32bit_register(941, 942)
        y = read_32bit_register(943, 944)
        
        # 確認已讀取
        write_register(945, 1)
        
        return (x, y)
    return None
```

## 主要改進功能

### **持續檢測策略**
- 找到DR_F後設置可用狀態，但不暫停檢測
- 確保保護區域內始終有DR_F可用
- 避免因單次檢測失敗造成的供料中斷

### **主動Flow1監控**
- 每0.1秒檢查1201寄存器狀態
- Flow1執行時立即暫停檢測避免衝突
- Flow1完成後立即恢復檢測

### **自動啟動機制**
- 程序啟動後自動開始檢測
- 無需外部觸發指令
- 支援配置檔案控制啟動行為

### **錯誤處理機制**
- 模組狀態檢查和自動重連
- CCD1初始化狀態容錯處理
- VP異常時的緊急停止功能

## 狀態監控

### **AutoFeeding狀態**
- **0**: 停止
- **1**: 運行中 (正常檢測)
- **2**: Flow1暫停 (1201=1時)
- **3**: 檢測中 (CCD1作業)
- **4**: VP震動中
- **5**: 錯誤

### **Flow1監控狀態**
- **0**: Flow1未執行
- **1**: Flow1執行中

### **DR_F狀態追蹤**
```python
self.dr_f_available     # 是否有DR_F可用
self.dr_f_coords        # 當前DR_F座標
self.dr_f_taken         # 座標是否已被讀取
```

## 使用優勢

### **對開發者**
- **簡化整合**: 不需要複雜的AutoProgram協調邏輯
- **直接交握**: Flow1直接讀取座標，邏輯清晰
- **自動運行**: 啟動即可工作，無需手動觸發
- **狀態透明**: 所有狀態都有寄存器對應

### **對系統運行**
- **持續供料**: 保護區域隨時有DR_F可用
- **快速響應**: 0.1秒間隔監控Flow1狀態
- **無死鎖**: 主動監控機制避免等待卡住
- **高可靠**: 獨立進程，異常不影響其他模組

### **對維護調試**
- **清晰日誌**: 所有狀態變化都有日誌記錄
- **狀態可見**: 寄存器狀態可實時監控
- **獨立運行**: 可單獨啟動測試
- **簡單重啟**: 重啟不影響其他模組

## 注意事項

### **啟動順序**
```
建議啟動順序:
1. 主Modbus TCP Server (端口502)
2. CCD1視覺檢測模組
3. VP震動盤模組  
4. AutoFeeding_main.py (自動啟動檢測)
5. Dobot_main (包含Flow1)
```

### **Flow1程序修改**
```
Flow1需要包含座標讀取邏輯:
- 檢查940=1
- 讀取941-944座標
- 設置945=1確認
- 執行後續動作
```

### **AutoProgram角色變化**
```
AutoProgram不再需要:
❌ 啟動/停止AutoFeeding
❌ 暫停/恢復AutoFeeding
❌ 座標中轉處理
❌ 複雜交握協議

AutoProgram現在只需要:
✅ 監控系統整體狀態
✅ Flow5完成後的狀態重置
✅ 異常處理和恢復
```

## 總結

DR版本的AutoFeeding實現了真正的**背景服務**模式：
- **持續不斷**的檢測確保DR_F供應
- **主動監控**Flow1避免衝突
- **直接交握**簡化整合難度
- **自動運行**降低操作複雜度
- **矩形保護區域**適應DR專案需求
- **YOLO檢測適配**支援二分類檢測

這個設計讓AutoFeeding變成一個可靠的DR_F供應服務，Flow1可以隨時取得所需的座標，整個系統更加穩定和高效。