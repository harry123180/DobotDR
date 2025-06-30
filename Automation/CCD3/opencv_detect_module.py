import cv2
import numpy as np
import math
import os
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import ttk

#影像前處理
def get_pre_treatment_image(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    return thresh

## 角度檢測
def get_main_contour(image, min_area_size_rate=0.05, sequence=False):
    min_area = image.shape[0] * image.shape[1] * min_area_size_rate
    contours, _ = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
    if not contours:
        return None
    # 找出最大內輪廓
    if sequence:
        contour = contours[-1]
    else:
        contour = contours[0]
    return contour

def get_obj_angle(image, mode=0):
    #mode->0 case ,mode->1 dr
    contour = None
    rst_contour = None

    pt_img = get_pre_treatment_image(image)
    
    if mode == 0:
        contour = get_main_contour(pt_img, sequence=True)
        if contour is None:
            return None, None
        mask_1 = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
        mask_2 = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
        cv2.drawContours(mask_1, [contour], -1, (255, 255, 255), -1)
        ellipse = cv2.fitEllipse(contour)

        # 取得中心與角度
        (x, y), (MA, ma), angle = ellipse
        print(f"橢圓長軸: {MA}")
        center = (int(x), int(y))
        
        # 畫橢圓與方向
        cv2.ellipse(mask_1, ellipse, (0, 0, 0), -1)
        
        center_circle, radius = cv2.minEnclosingCircle(contour)
        center_circle = (int(center_circle[0]), int(center_circle[1]))
        
        cv2.circle(mask_2, center_circle, int(radius), (255, 255, 255), -1)
        kernel = np.ones((11, 11), np.uint8)
        mask_1 = cv2.dilate(mask_1, kernel, iterations=1)
        mask_1 = cv2.bitwise_not(mask_1)
        rst = cv2.bitwise_and(mask_1, mask_1, mask=mask_2)
        rst_contour = get_main_contour(rst)

    else:
        rst_contour = get_main_contour(pt_img)
        if rst_contour is None:
            return None, None
    
    rect = cv2.minAreaRect(rst_contour)
    box = cv2.boxPoints(rect)
    box = np.int_(box)
    angle = rect[2]

    # 視覺化
    cv2.drawContours(image, [box], 0, (0, 255, 0), 2)
    center = tuple(np.int_(rect[0]))
    
    return center, angle

def show_results_with_pil(results):
    """使用PIL顯示檢測結果"""
    root = tk.Tk()
    root.title("CCD3角度檢測結果對比")
    root.geometry("1200x800")
    
    # 創建主框架
    main_frame = ttk.Frame(root)
    main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
    
    # 標題
    title_label = ttk.Label(main_frame, text="CCD3角度檢測結果對比", font=("Arial", 16, "bold"))
    title_label.pack(pady=(0, 20))
    
    # 創建結果顯示區域
    results_frame = ttk.Frame(main_frame)
    results_frame.pack(fill=tk.BOTH, expand=True)
    
    for i, (mode_name, frame_bgr, center, angle) in enumerate(results):
        # 創建每個模式的框架
        mode_frame = ttk.LabelFrame(results_frame, text=mode_name, padding=10)
        mode_frame.grid(row=0, column=i, padx=10, pady=10, sticky="nsew")
        
        # 配置列權重
        results_frame.columnconfigure(i, weight=1)
        results_frame.rowconfigure(0, weight=1)
        
        # 轉換OpenCV圖像為PIL格式
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        
        # 調整圖像大小以適應顯示
        height, width = frame_rgb.shape[:2]
        max_size = 400
        if max(height, width) > max_size:
            scale = max_size / max(height, width)
            new_width = int(width * scale)
            new_height = int(height * scale)
            frame_rgb = cv2.resize(frame_rgb, (new_width, new_height))
        
        # 轉換為PIL Image
        pil_image = Image.fromarray(frame_rgb)
        photo = ImageTk.PhotoImage(pil_image)
        
        # 顯示圖像
        image_label = ttk.Label(mode_frame, image=photo)
        image_label.image = photo  # 保持引用
        image_label.pack(pady=(0, 10))
        
        # 顯示檢測結果文字
        if center is not None and angle is not None:
            result_text = f"中心座標: {center}\n角度: {angle:.2f}°"
            result_color = "green"
        else:
            result_text = "檢測失敗\n未找到有效輪廓"
            result_color = "red"
        
        result_label = ttk.Label(mode_frame, text=result_text, 
                               font=("Arial", 12), foreground=result_color)
        result_label.pack()
    
    # 添加關閉按鈕
    close_button = ttk.Button(main_frame, text="關閉", command=root.destroy)
    close_button.pack(pady=(20, 0))
    
    # 顯示視窗
    root.mainloop()

def test_debug_image():
    # 獲取執行檔案的目錄
    script_dir = os.path.dirname(os.path.abspath(__file__))
    debug_image_path = os.path.join(script_dir, "debug_images", "1_original.jpg")
    
    print(f"執行檔案目錄: {script_dir}")
    print(f"尋找圖像路徑: {debug_image_path}")
    
    if not os.path.exists(debug_image_path):
        print(f"錯誤: 找不到圖像檔案 {debug_image_path}")
        print("請確認CCD3系統已運行並產生調試圖像")
        return
    
    print(f"讀取圖像: {debug_image_path}")
    frame = cv2.imread(debug_image_path)
    
    if frame is None:
        print(f"錯誤: 無法讀取圖像 {debug_image_path}")
        return
    
    print(f"圖像尺寸: {frame.shape}")
    
    # 測試兩種模式
    modes = [0, 1]
    mode_names = ["CASE模式(橢圓擬合)", "DR模式(最小外接矩形)"]
    
    results = []
    
    for mode in modes:
        print(f"\n=== 測試{mode_names[mode]} ===")
        frame_copy = frame.copy()
        
        result = get_obj_angle(frame_copy, mode=mode)
        
        if result[0] is None:
            print(f"{mode_names[mode]}: 檢測失敗，未找到有效輪廓")
            center, angle = None, None
        else:
            center, angle = result
            print(f"{mode_names[mode]}: 中心座標={center}, 角度={angle:.2f}度")
            
            # 添加文字標註
            cv2.putText(frame_copy, f"Mode: {mode_names[mode]}", (50, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(frame_copy, f"Angle: {angle:.2f} deg", (center[0] - 70, center[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame_copy, f"Center: {center}", (center[0] - 50, center[1] + 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            # 標記中心點
            cv2.circle(frame_copy, center, 5, (255, 0, 0), -1)
        
        # 保存結果到列表
        results.append((mode_names[mode], frame_copy, center, angle))
        
        # 保存結果圖像
        debug_dir = os.path.join(script_dir, "debug_images")
        result_path = os.path.join(debug_dir, f"test_result_mode_{mode}.jpg")
        cv2.imwrite(result_path, frame_copy)
        print(f"結果已保存: {result_path}")
    
    # 使用PIL顯示結果
    print("\n顯示PIL結果視窗...")
    show_results_with_pil(results)
    
    print("測試完成")

if __name__ == "__main__":
    print("CCD3調試圖像角度檢測測試")
    print("=== 測試debug_images/1_original.jpg ===")
    test_debug_image()