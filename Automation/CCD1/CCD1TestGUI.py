import numpy as np
import psutil
import gc
import time

def get_memory_mb():
    """獲取當前程式記憶體使用量(MB)"""
    process = psutil.Process()
    return process.memory_info().rss / 1024 / 1024

def test_numpy_gc():
    print(f"初始記憶體: {get_memory_mb():.1f}MB")
    
    # 創建大型數組列表
    arrays = []
    for i in range(10):
        # 每個數組約5MB
        arr = np.zeros((1944, 2592), dtype=np.uint8)
        arrays.append(arr)
        print(f"創建第{i+1}個數組，記憶體: {get_memory_mb():.1f}MB")
    
    print(f"創建完成，記憶體: {get_memory_mb():.1f}MB")
    
    # 刪除引用
    del arrays
    print(f"刪除引用後，記憶體: {get_memory_mb():.1f}MB")
    
    # 等待自動垃圾回收
    for i in range(10):
        time.sleep(1)
        memory_now = get_memory_mb()
        print(f"等待{i+1}秒，記憶體: {memory_now:.1f}MB")
    
    # 手動觸發垃圾回收
    collected = gc.collect()
    print(f"手動GC回收了{collected}個對象，記憶體: {get_memory_mb():.1f}MB")

# 執行測試
test_numpy_gc()
def enhanced_memory_test():
    """增強版記憶體測試"""
    import psutil
    import os
    
    process = psutil.Process(os.getpid())
    
    def print_memory_info():
        mem = process.memory_info()
        return {
            'rss_mb': mem.rss / 1024 / 1024,
            'vms_mb': mem.vms / 1024 / 1024,
            'percent': process.memory_percent()
        }
    
    print("=== 增強版記憶體測試 ===")
    start_mem = print_memory_info()
    print(f"初始記憶體: RSS={start_mem['rss_mb']:.1f}MB, VMS={start_mem['vms_mb']:.1f}MB")
    
    # 創建數組
    arrays = []
    for i in range(5):  # 減少到5個以便觀察
        arr = np.zeros((1944, 2592), dtype=np.uint8)
        arrays.append(arr)
        
        current_mem = print_memory_info()
        growth = current_mem['rss_mb'] - start_mem['rss_mb']
        print(f"創建第{i+1}個數組: RSS={current_mem['rss_mb']:.1f}MB (+{growth:.1f}MB)")
    
    # 檢查數組大小
    total_size = sum(arr.nbytes for arr in arrays) / 1024 / 1024
    print(f"數組總大小: {total_size:.1f}MB")
    
    # 刪除引用
    del arrays
    
    after_del = print_memory_info()
    print(f"刪除後: RSS={after_del['rss_mb']:.1f}MB")
    
    # 手動GC
    import gc
    collected = gc.collect()
    
    final_mem = print_memory_info()
    final_growth = final_mem['rss_mb'] - start_mem['rss_mb']
    print(f"GC後: RSS={final_mem['rss_mb']:.1f}MB (+{final_growth:.1f}MB), 回收{collected}個對象")

# 執行增強版測試
enhanced_memory_test()

def check_numpy_allocation():
    """檢查numpy分配方式"""
    import numpy as np
    
    print("=== numpy分配檢查 ===")
    
    # 檢查numpy配置
    print(f"numpy版本: {np.__version__}")
    print(f"numpy配置: {np.show_config()}")
    
    # 測試實際記憶體分配
    arr = np.zeros((1944, 2592), dtype=np.uint8)
    print(f"數組大小: {arr.nbytes / 1024 / 1024:.1f}MB")
    print(f"數組形狀: {arr.shape}")
    print(f"數組dtype: {arr.dtype}")
    print(f"數組flags: {arr.flags}")
    
    # 檢查是否使用記憶體映射
    print(f"是否記憶體映射: {hasattr(arr, 'filename')}")
    if hasattr(arr, 'filename'):
        print(f"映射檔案: {arr.filename}")

check_numpy_allocation()

def test_real_memory_usage():
    """測試實際記憶體使用"""
    import psutil
    import os
    
    def get_detailed_memory():
        process = psutil.Process(os.getpid())
        mem = process.memory_info()
        
        # 嘗試獲取更多記憶體信息
        try:
            full_info = process.memory_full_info()
            return {
                'rss': mem.rss / 1024 / 1024,
                'vms': mem.vms / 1024 / 1024,
                'uss': full_info.uss / 1024 / 1024,  # 獨佔記憶體
                'pss': full_info.pss / 1024 / 1024,  # 按比例分配記憶體
                'shared': (mem.rss - full_info.uss) / 1024 / 1024
            }
        except:
            return {
                'rss': mem.rss / 1024 / 1024,
                'vms': mem.vms / 1024 / 1024,
                'uss': 'N/A',
                'pss': 'N/A', 
                'shared': 'N/A'
            }
    
    print("=== 詳細記憶體分析 ===")
    start_mem = get_detailed_memory()
    print(f"初始: RSS={start_mem['rss']:.1f}MB, VMS={start_mem['vms']:.1f}MB")
    print(f"      USS={start_mem['uss']}, PSS={start_mem['pss']}")
    
    # 創建並實際寫入數據（強制分配）
    arrays = []
    for i in range(3):
        arr = np.zeros((1944, 2592), dtype=np.uint8)
        # 強制實際分配記憶體
        arr.fill(255)  # 寫入數據強制分配
        arrays.append(arr)
        
        current_mem = get_detailed_memory()
        rss_growth = current_mem['rss'] - start_mem['rss']
        print(f"創建並填充第{i+1}個數組: RSS={current_mem['rss']:.1f}MB (+{rss_growth:.1f}MB)")
    
    print(f"總數組大小: {sum(arr.nbytes for arr in arrays) / 1024 / 1024:.1f}MB")

test_real_memory_usage()



def check_python_environment():
    """檢查Python環境"""
    import sys
    import platform
    
    print("=== Python環境檢查 ===")
    print(f"Python版本: {sys.version}")
    print(f"平台: {platform.platform()}")
    print(f"架構: {platform.architecture()}")
    print(f"執行檔: {sys.executable}")
    
    # 檢查是否在特殊環境中
    print(f"是否在虛擬環境: {hasattr(sys, 'real_prefix') or (hasattr(sys, 'base_prefix') and sys.base_prefix != sys.prefix)}")
    
    # 檢查記憶體分配器
    try:
        import tracemalloc
        print(f"tracemalloc可用: True")
        tracemalloc.start()
        
        # 測試記憶體追蹤
        arr = np.zeros((1000, 1000), dtype=np.uint8)
        current, peak = tracemalloc.get_traced_memory()
        print(f"tracemalloc追蹤: 當前={current/1024/1024:.1f}MB, 峰值={peak/1024/1024:.1f}MB")
        
        tracemalloc.stop()
    except:
        print(f"tracemalloc不可用")

check_python_environment()