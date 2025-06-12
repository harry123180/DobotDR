# CCD視覺檢測模組技術文檔 v4.0

## 架構概述

CCD視覺檢測模組實現握手式狀態機控制，採用Modbus TCP Client架構，v4.0版本新增世界座標轉換功能。

```
主服務器 (ModbusTCP:502)
    |
    |-- TCP --> CCD1VisionCode_Enhanced.py (TCP Client)
                    |
                    |-- 直連 --> 相機設備 (192.168.1.8)
                    |-- 檔案 --> 內外參NPY檔案 (同層目錄)
```

## 實現組件

### CCD1VisionCode_Enhanced.py - 主模組 (v4.0)
- TCP Client連接主服務器 (默認127.0.0.1:502)
- 相機直連控制 (192.168.1.8)
- 寄存器映射基地址: 200
- 握手式狀態機交握
- 50ms輪詢間隔
- **新增**: 內外參檔案管理
- **新增**: 世界座標轉換功能

### camera_manager.py - 相機管理API
- 海康威視SDK封裝
- 優化相機管理器
- 幀數據處理
- 性能監控

### CalibrationManager - 標定管理器 (新增)
- 內外參NPY檔案掃描
- 檔案格式驗證
- 標定數據載入
- 座標轉換器管理

### CameraCoordinateTransformer - 座標轉換器 (新增)
- 像素座標到世界座標轉換
- 基於OpenCV實現
- Z=0平面投影
- 畸變校正處理

## 寄存器映射 (基地址200) - v4.0擴展版本

### 核心控制握手寄存器 (200-201)
| 地址 | 功能 | 數值定義 |
|------|------|----------|
| 200 | 控制指令 | 0=清空, 8=拍照, 16=拍照+檢測, 32=重新初始化 |
| 201 | 狀態寄存器 | bit0=Ready, bit1=Running, bit2=Alarm, bit3=Initialized |

### 檢測參數寄存器 (210-219)
| 地址 | 功能 | 說明 |
|------|------|------|
| 210 | 最小面積高位 | 32位面積設定高16位 |
| 211 | 最小面積低位 | 32位面積設定低16位 |
| 212 | 最小圓度 | 圓度設定值乘以1000 |
| 213 | 高斯核大小 | 圖像處理參數 |
| 214 | Canny低閾值 | 邊緣檢測參數 |
| 215 | Canny高閾值 | 邊緣檢測參數 |

### 像素座標檢測結果寄存器 (240-255)
| 地址 | 功能 | 說明 |
|------|------|------|
| 240 | 檢測圓形數量 | 最多5個 |
| 241-243 | 圓形1像素座標半徑 | X, Y, Radius |
| 244-246 | 圓形2像素座標半徑 | X, Y, Radius |
| 247-249 | 圓形3像素座標半徑 | X, Y, Radius |
| 250-252 | 圓形4像素座標半徑 | X, Y, Radius |
| 253-255 | 圓形5像素座標半徑 | X, Y, Radius |

### 世界座標檢測結果寄存器 (256-276) - v4.0新增
| 地址 | 功能 | 說明 |
|------|------|------|
| 256 | 世界座標有效標誌 | 0=無效, 1=有效 |
| 257 | 圓形1世界X座標高位 | 32位世界X座標高16位 (×100) |
| 258 | 圓形1世界X座標低位 | 32位世界X座標低16位 (×100) |
| 259 | 圓形1世界Y座標高位 | 32位世界Y座標高16位 (×100) |
| 260 | 圓形1世界Y座標低位 | 32位世界Y座標低16位 (×100) |
| 261-262 | 圓形2世界X座標 | 高位/低位 (×100) |
| 263-264 | 圓形2世界Y座標 | 高位/低位 (×100) |
| 265-266 | 圓形3世界X座標 | 高位/低位 (×100) |
| 267-268 | 圓形3世界Y座標 | 高位/低位 (×100) |
| 269-270 | 圓形4世界X座標 | 高位/低位 (×100) |
| 271-272 | 圓形4世界Y座標 | 高位/低位 (×100) |
| 273-274 | 圓形5世界X座標 | 高位/低位 (×100) |
| 275-276 | 圓形5世界Y座標 | 高位/低位 (×100) |

### 統計資訊寄存器 (280-299)
| 地址 | 功能 | 說明 |
|------|------|------|
| 280 | 最後拍照耗時 | 毫秒單位 |
| 281 | 最後處理耗時 | 毫秒單位 |
| 282 | 最後總耗時 | 毫秒單位 |
| 283 | 操作計數器 | 累積操作次數 |
| 284 | 錯誤計數器 | 累積錯誤次數 |
| 285 | 連接計數器 | 累積連接次數 |
| 290 | 軟體版本主號 | 版本4 (新增世界座標功能) |
| 291 | 軟體版本次號 | 版本0 |
| 292 | 運行時間小時 | 系統運行時間 |
| 293 | 運行時間分鐘 | 系統運行時間 |

## 世界座標轉換功能 (v4.0新增)

### 內外參檔案管理

#### 檔案格式要求
```python
# 內參檔案 (規範命名)
camera_matrix_YYYYMMDD_HHMMSS.npy    # 3×3內參矩陣
dist_coeffs_YYYYMMDD_HHMMSS.npy      # 畸變係數陣列

# 外參檔案 (較寬鬆命名)
extrinsic_*.npy                      # 包含rvec和tvec的字典格式
*extrinsic*.npy                      # 其他包含extrinsic的檔案
```

#### 檔案格式驗證
```python
# 內參驗證
camera_matrix.shape == (3, 3)        # 內參矩陣必須為3×3
len(dist_coeffs) >= 4                # 畸變係數至少4個參數

# 外參驗證
extrinsic_data = {
    'rvec': np.array([[rx], [ry], [rz]]),    # 3×1旋轉向量
    'tvec': np.array([[tx], [ty], [tz]])     # 3×1平移向量
}
```

### 座標轉換實現

#### CameraCoordinateTransformer類
```python
class CameraCoordinateTransformer:
    def __init__(self, camera_matrix, dist_coeffs, rvec, tvec):
        self.K = camera_matrix
        self.D = dist_coeffs
        self.rvec = rvec.reshape(3, 1)
        self.tvec = tvec.reshape(3, 1)
        self.R, _ = cv2.Rodrigues(self.rvec)
```

#### 轉換流程
1. **去畸變處理**: `cv2.undistortPoints()`
2. **歸一化座標**: `K^(-1) * [u, v, 1]`
3. **深度計算**: `s = -t_z / (R3 · normalized_coords)`
4. **世界座標**: `R^(-1) * (s * normalized_coords - t)`

#### 轉換數學模型
```python
def pixel_to_world(self, pixel_coords):
    # 步驟1: 去畸變
    undistorted_uv = cv2.undistortPoints(
        pixel_coords.reshape(1, 1, 2), self.K, self.D, P=self.K
    ).reshape(-1)
    
    # 步驟2: 歸一化座標
    normalized_coords = np.linalg.inv(self.K) @ [u, v, 1]
    
    # 步驟3: 計算深度係數 (Z=0平面)
    s = (0 - self.tvec[2, 0]) / (self.R[2] @ normalized_coords)
    
    # 步驟4: 計算世界座標
    camera_point = s * normalized_coords
    world_point = np.linalg.inv(self.R) @ (camera_point - self.tvec.ravel())
    
    return world_point[:2]  # 返回X,Y座標
```

### 精度處理

#### 世界座標精度設計
- **存儲格式**: 32位有符號整數
- **精度係數**: ×100 (保留2位小數)
- **數值範圍**: ±21474.83mm
- **寄存器分配**: 每個座標使用2個16位寄存器 (高位+低位)

#### 數值轉換實現
```python
# 世界座標轉寄存器值
world_x_int = int(world_x * 100)        # 123.45mm → 12345
world_x_high = (world_x_int >> 16) & 0xFFFF
world_x_low = world_x_int & 0xFFFF

# 寄存器值恢復世界座標
world_x_int = (world_x_high << 16) | world_x_low
world_x = world_x_int / 100.0           # 12345 → 123.45mm
```

## 握手協議實現 (v4.0更新)

### 狀態機定義
SystemStateMachine類實現4位狀態控制:
- Ready (bit0): 系統準備接受新指令
- Running (bit1): 系統正在執行操作
- Alarm (bit2): 系統異常或錯誤
- Initialized (bit3): 系統已完全初始化

### 握手交握流程
1. 系統初始化完成 → 狀態寄存器固定值1 (Ready=1)
2. PLC下達控制指令 → 檢查Ready=1
3. 開始執行 → Ready=0, Running=1
4. 執行完成 → Running=0
5. PLC清零指令 → Ready=1 (準備下次)
6. 異常發生 → Alarm=1, Initialized=0

### 控制指令映射
| 指令碼 | 功能 | 執行內容 |
|--------|------|----------|
| 0 | 清空控制 | 狀態機恢復Ready |
| 8 | 拍照 | 單純圖像捕獲 |
| 16 | 拍照+檢測 | 圖像捕獲與圓形檢測 (含世界座標) |
| 32 | 重新初始化 | 相機重新初始化 |

### 世界座標檢測流程 (v4.0新增)
```python
def capture_and_detect(self):
    # 1. 圓形檢測 (像素座標)
    circles, annotated_image = self.detector.detect_circles(image)
    
    # 2. 世界座標轉換 (如果標定數據有效)
    if self.calibration_manager.transformer.is_valid():
        for circle in circles:
            pixel_coords = [circle['center']]
            world_coords = self.transformer.pixel_to_world(pixel_coords)
            circle['world_coords'] = (world_coords[0], world_coords[1])
    
    # 3. 更新Modbus寄存器
    result.has_world_coords = can_transform
    self.modbus_client.update_detection_results(result)
```

## 圓形檢測演算法

### CircleDetector類實現
- 高斯濾波預處理
- Canny邊緣檢測
- 輪廓分析與圓度計算
- 最多輸出5個檢測結果

### 檢測參數配置
```python
class DetectionParams:
    min_area: float = 50000.0
    min_roundness: float = 0.8
    gaussian_kernel_size: int = 9
    gaussian_sigma: float = 2.0
    canny_low: int = 20
    canny_high: int = 60
```

### 結果數據結構 (v4.0擴展)
```python
class VisionResult:
    circle_count: int
    circles: List[Dict[str, Any]]        # 包含world_coords
    processing_time: float
    capture_time: float
    total_time: float
    timestamp: str
    success: bool
    has_world_coords: bool = False       # 新增: 世界座標有效性
    error_message: Optional[str] = None
```

## 相機管理實現

### OptimizedCamera類
- 海康威視SDK封裝
- 單相機完整生命週期管理
- 幀緩存機制
- 性能監控統計

### CameraConfig配置
```python
class CameraConfig:
    name: str = "cam_1"
    ip: str = "192.168.1.8"
    exposure_time: float = 20000.0
    gain: float = 200.0
    frame_rate: float = 30.0
    pixel_format: PixelFormat = BAYER_GR8
    width: int = 2592
    height: int = 1944
```

## 線程架構

### 主要線程
- **主線程**: Flask Web應用
- **握手同步線程**: _handshake_sync_loop (50ms輪詢)
- **指令執行線程**: _execute_command_async (異步執行)

### 線程安全機制
- SystemStateMachine使用self.lock保護狀態
- 指令執行狀態command_processing防重入
- 相機操作使用threading.RLock

## Flask Web介面 (v4.0更新)

### 核心API路由
- POST /api/modbus/set_server - 設置Modbus服務器地址
- POST /api/modbus/connect - 連接Modbus服務器
- GET /api/modbus/registers - 讀取所有寄存器即時數值 (含世界座標)
- POST /api/modbus/manual_command - 手動發送控制指令
- POST /api/initialize - 初始化相機連接
- POST /api/capture_and_detect - 執行拍照檢測 (含世界座標)
- **新增**: GET /api/calibration/scan - 掃描標定檔案
- **新增**: POST /api/calibration/load - 載入標定數據
- **新增**: GET /api/calibration/status - 獲取標定狀態

### Web介面功能
- 運行在localhost:5051
- SocketIO即時通訊
- 狀態監控顯示
- 參數調整介面
- 手動控制功能
- **新增**: 標定檔案管理介面
- **新增**: 世界座標狀態顯示
- **新增**: 世界座標結果顯示

### UI界面更新 (v4.0)
```html
<!-- 標定管理面板 -->
<div class="calibration-panel">
    <h2>🌍 相機標定與世界座標轉換</h2>
    <div class="calibration-status">
        <!-- 內參狀態、外參狀態、轉換器狀態、工作目錄 -->
    </div>
    <button onclick="scanCalibrationFiles()">🔍 掃描檔案</button>
    <button onclick="loadCalibrationData()">✅ 確認導入</button>
</div>

<!-- 檢測結果顯示 (含世界座標) -->
<div class="circle-coords">
    <div class="coord-group pixel-coord">
        <div class="coord-label">像素座標</div>
        <div class="coord-value">X: 320 px, Y: 240 px</div>
    </div>
    <div class="coord-group world-coord">
        <div class="coord-label">世界座標</div>
        <div class="coord-value">X: 123.45 mm, Y: 678.90 mm</div>
    </div>
</div>
```

## 依賴模組版本

### 核心依賴
- PyModbus 3.9.2 (Modbus TCP Client)
- OpenCV (圖像處理 + 座標轉換)
- NumPy (數值計算)
- Flask + SocketIO (Web介面)

### 海康威視SDK
- MvCameraControl_class
- CameraParams_const
- CameraParams_header
- MvErrorDefine_const
- PixelType_header

## 錯誤處理機制 (v4.0擴展)

### 連接管理
- Modbus TCP自動重連 (reconnect_delay: 5秒)
- 相機連接異常處理
- 超時控制 (read_timeout: 3秒)

### 標定檔案錯誤處理 (新增)
- 檔案格式驗證
- 檔案缺失提示
- 轉換失敗處理
- 詳細錯誤信息顯示

### 狀態異常處理
- 通訊錯誤計數統計
- Alarm狀態自動設置
- 錯誤資訊記錄與回報

### 異常恢復
- 重新初始化指令 (32)
- 狀態機重置機制
- 資源釋放與重建

## 配置管理

### 模組配置
- 無獨立配置檔案
- 硬編碼預設參數
- 運行時動態調整

### 預設配置值
```python
# Modbus連接
server_ip = "192.168.1.100"
server_port = 502
base_address = 200

# 相機配置  
camera_ip = "192.168.1.8"
exposure_time = 20000.0
gain = 200.0

# 檢測參數
min_area = 50000.0
min_roundness = 0.8

# 標定檔案 (新增)
working_dir = os.path.dirname(os.path.abspath(__file__))
```

## 部署配置 (v4.0更新)

### 運行順序
1. 啟動主Modbus TCP Server (端口502)
2. 準備內外參NPY檔案 (放入程式同層目錄)
3. 啟動CCD1VisionCode_Enhanced.py
4. 訪問Web介面 (localhost:5051)
5. 掃描並載入標定檔案
6. 設置Modbus服務器地址並連接
7. 初始化相機連接
8. 系統進入握手模式等待PLC指令

### 檔案結構 (v4.0)
```
Vision/
├── CCD1VisionCode_Enhanced.py    # 主程序 (v4.0)
├── camera_manager.py             # 相機管理API
├── templates/
│   └── ccd_vision_enhanced_world_coord.html  # Web介面 (v4.0)
├── camera_matrix_20241210_143022.npy         # 內參矩陣 (範例)
├── dist_coeffs_20241210_143022.npy           # 畸變係數 (範例)
└── extrinsic_20241210_143530.npy             # 外參數據 (範例)
```

## 測試驗證 (v4.0擴展)

### 連接測試
1. Modbus TCP連接狀態檢查
2. 相機設備連接驗證
3. 寄存器讀寫功能測試 (含世界座標寄存器)

### 標定功能測試 (新增)
1. NPY檔案掃描功能
2. 檔案格式驗證
3. 標定數據載入測試
4. 座標轉換精度驗證

### 功能測試
1. 握手協議狀態機驗證
2. 控制指令執行確認
3. 圓形檢測演算法準確性
4. 世界座標轉換準確性 (新增)
5. 錯誤處理機制測試

### 性能監控
1. 拍照耗時統計
2. 檢測處理時間
3. 座標轉換耗時 (新增)
4. 握手響應延遲
5. 系統穩定性驗證

## 已知限制 (v4.0更新)

### 硬體依賴
- 需要海康威視SDK完整安裝
- 相機必須支援TCP/IP連接
- 網路延遲影響響應時間

### 世界座標限制 (新增)
- 僅支援Z=0平面投影
- 精度受標定品質影響
- 需要高品質內外參數據
- 座標範圍限制: ±21474.83mm

### 軟體限制
- 最多檢測5個圓形
- 固定圓形檢測演算法
- 無持久化配置機制
- NPY檔案格式限制

### 系統限制
- 單相機支援
- 同步執行模式
- 無負載平衡機制

## 開發問題修正記錄

### v4.0世界座標功能開發記錄

#### 1. 寄存器地址分配 (新增)
**設計**: 世界座標寄存器區域規劃
**實現**: 採用方案A，256-276地址區間
```python
# 寄存器分配
像素座標: 241-255 (現有)
世界座標: 256-276 (新增，20個寄存器)
統計資訊: 280-299 (保持不變)
```

#### 2. 精度處理實現 (新增)
**需求**: 世界座標保留2位小數精度
**實現**: ×100整數存儲方案
```python
world_x_int = int(world_x * 100)  # 123.45 → 12345
world_x = world_x_int / 100.0     # 12345 → 123.45
```

#### 3. 檔案管理功能 (新增)
**功能**: 自動掃描內外參NPY檔案
**實現**: CalibrationManager類
- 規範內參檔案命名檢查
- 寬鬆外參檔案命名支援
- 檔案格式自動驗證

#### 4. UI界面整合 (新增)
**功能**: 標定管理與世界座標顯示
**實現**: Web界面擴展
- 標定狀態面板
- 世界座標指示器
- 檔案掃描與導入按鈕
- 檢測結果雙座標顯示

#### 5. 向下兼容處理 (新增)
**需求**: 無標定數據時系統正常工作
**實現**: 條件式世界座標轉換
```python
has_world_coords = (標定數據有效 and 轉換成功)
result.has_world_coords = has_world_coords
```

### 原有問題修正記錄

#### 1. 狀態機初始值問題 (已修正)
**錯誤**: 狀態寄存器初始值不固定
**修正**: 強制設置初始值為1 (Ready=1, 其他位=0)
```python
# 修正實現
initial_status = 0b0001
self.state_machine.status_register = initial_status
```

#### 2. 握手協議時序錯誤 (已修正)
**錯誤**: Running狀態未正確清除
**修正**: 指令執行完成後確保Running=0，等待PLC清零指令恢復Ready

#### 3. 相機連接異常處理 (已修正)
**錯誤**: 相機斷線未設置Alarm狀態
**修正**: 相機異常時自動設置Alarm=1, Initialized=0

#### 4. 線程同步問題 (已修正)
**錯誤**: 多線程訪問狀態機競爭條件
**修正**: 使用threading.Lock保護狀態機操作
```python
def set_bit(self, bit_pos: StatusBits, value: bool):
    with self.lock:
        # 原子化操作
```

#### 5. 指令重複執行問題 (已修正)
**錯誤**: 同一指令被重複執行
**修正**: 實現command_processing標誌防重入
- last_control_command追蹤已處理指令
- command_processing防止重複執行

## 狀態機交握實現細節 (v4.0更新)

### EnhancedModbusTcpClientService核心方法
```python
def _handshake_sync_loop(self):
    # 50ms高頻輪詢
    # 1. 更新狀態寄存器到PLC
    # 2. 讀取控制指令並處理握手邏輯
    # 3. 定期更新統計資訊
    # 4. 更新世界座標有效性標誌 (新增)
```

### 指令執行流程
```python
def _handle_action_command(self, command: ControlCommand):
    # 1. 檢查Ready狀態
    # 2. 設置Running狀態，清除Ready
    # 3. 異步執行指令 (含世界座標轉換)
    # 4. 完成後清除Running狀態
```

### 異常處理機制
```python
def _update_initialization_status(self):
    # 檢查Modbus和相機連接狀態
    # 檢查標定數據有效性 (新增)
    # 設置Initialized和Alarm位
    # 確保狀態一致性
```

## API介面文檔 (v4.0擴展)

### CCD1VisionController關鍵方法
```python
def set_modbus_server(self, ip: str, port: int) -> Dict[str, Any]
def connect_modbus(self) -> Dict[str, Any]  
def initialize_camera(self, ip_address: str) -> Dict[str, Any]
def capture_and_detect(self) -> VisionResult  # 含世界座標
def update_detection_params(self, **kwargs)
def get_status(self) -> Dict[str, Any]       # 含標定狀態

# 新增標定相關方法
def scan_calibration_files(self) -> Dict[str, Any]
def load_calibration_data(self, intrinsic_file=None, extrinsic_file=None) -> Dict[str, Any]
def get_calibration_status(self) -> Dict[str, Any]
```

### SystemStateMachine狀態控制
```python
def is_ready(self) -> bool
def is_running(self) -> bool
def is_alarm(self) -> bool
def is_initialized(self) -> bool
def set_ready(self, ready: bool)
def set_running(self, running: bool)
def reset_to_idle(self)
```

### CalibrationManager標定管理 (新增)
```python
def scan_calibration_files(self) -> Dict[str, Any]
def load_calibration_data(self, intrinsic_file=None, extrinsic_file=None) -> Dict[str, Any]
def get_status(self) -> Dict[str, Any]
```

### CameraCoordinateTransformer座標轉換 (新增)
```python
def pixel_to_world(self, pixel_coords) -> Optional[np.ndarray]
def is_valid(self) -> bool
```

## 運行狀態輸出 (v4.0更新)

### 系統啟動輸出
```
CCD1 視覺控制系統啟動中 (運動控制握手版本 + 世界座標轉換)
系統架構: Modbus TCP Client - 運動控制握手模式 + 世界座標轉換
連接模式: 主動連接外部PLC/HMI設備
握手協議: 指令/狀態模式，50ms高頻輪詢
新功能: 內外參管理 + 像素座標到世界座標轉換
Web介面啟動中... http://localhost:5051
```

### 握手協議運行日誌
```
收到新控制指令: 16 (上次: 0)
開始處理控制指令: 16
執行拍照+檢測指令
檢測成功，找到 3 個圓形 (包含世界座標)
控制指令 16 執行完成
恢復Ready狀態
```

### 世界座標轉換日誌 (新增)
```
掃描結果: 找到1組內參檔案, 1個外參檔案
標定數據載入成功
轉換器已啟用，載入時間: 2024-12-10 14:30:22
像素座標 [320, 240] 對應世界座標 [123.45, 678.90] mm
世界座標寄存器更新: 256=1, 257-260=[1, 23400, 6, 78900]
```

## 記憶體管理

### 線程生命週期
- daemon模式線程自動回收
- 握手同步線程正確退出機制
- 相機資源釋放管理

### 幀數據管理
- 固定大小幀緩存隊列
- 舊幀自動丟棄機制
- numpy數組記憶體回收

### 標定數據管理 (新增)
- NPY檔案按需載入
- 標定數據快取管理
- 座標轉換器資源釋放

### 資源釋放
```python
def disconnect(self):
    # 停止握手同步線程
    # 斷開相機連接
    # 關閉Modbus連接
    # 釋放標定數據 (新增)
    # 釋放所有資源
```

## 版本歷史

### v4.0 (2024-12-XX) - 世界座標轉換功能
- **新增**: 內外參NPY檔案管理
- **新增**: 像素座標到世界座標轉換 (Z=0平面)
- **新增**: 世界座標寄存器映射 (256-276)
- **新增**: 標定檔案掃描與載入API
- **新增**: Web界面標定管理功能
- **新增**: 世界座標結果顯示 (保留2位小數)
- **更新**: 軟體版本號升級到4.0
- **擴展**: Modbus寄存器總數增加到21個

### v3.0 (原版本) - 運動控制握手
- 握手式狀態機控制
- 50ms高頻輪詢
- 圓形檢測演算法
- Web界面控制
- Modbus TCP Client架構