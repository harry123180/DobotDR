#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
system.py - 自動化系統主控制器
- 物件管理器執行緒：初始化所有子系統
- 參數設定事件：設定CCD1模型、夾爪開度、震動盤參數
- 自動進料事件：CCD1檢測物料並控制震動進料
- 機械臂上料事件：執行Flow1流程(直接實作)
- 機械臂組裝事件：執行Flow2流程(直接實作)
"""

import os
import sys
import time
import threading
import logging
import json
from typing import Dict, Any, Optional, List

# 添加專案路徑
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(os.path.dirname(current_dir), 'Robot'))

# 導入機器人系統
from Robot import Robot, RobotConfig

# 全域變數 - 物件和狀態
robot = None

# 全域變數 - 物料狀態
material_available = True          # 原物料狀態
label_0_objects = []              # 標籤0物件座標列表
label_1_objects = []              # 標籤1物件座標列表
label_2_objects = []              # 標籤2物件座標列表
flow1_execution_count = 0         # Flow1執行次數

# 全域變數 - 角度數據
current_angle = 0.0               # CCD3檢測角度
command_angle = 45.0              # 指令角度(檢測角度+補償值)

# 全域變數 - 系統參數
ccd1_model_id = 1                 # CCD1模型ID
gripper_opening = 360             # 夾爪開度
vibration_intensity = 60          # 震動強度
vibration_frequency = 100         # 震動頻率

# 全域變數 - 點位數據
robot_points = {}                 # 機械臂點位數據

# 全域鎖和事件
system_lock = threading.Lock()
object_manager_event = threading.Event()
parameter_setup_event = threading.Event()
auto_feeding_event = threading.Event()
robot_pickup_event = threading.Event()
robot_assembly_event = threading.Event()

# 常數
PICKUP_HEIGHT = 147.52
ANGLE_OFFSET = 45.0

# 日誌設置
def setup_logging():
    log_dir = os.path.join(current_dir, 'logs')
    os.makedirs(log_dir, exist_ok=True)
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s',
        handlers=[
            logging.FileHandler(os.path.join(log_dir, 'system.log'), encoding='utf-8'),
            logging.StreamHandler()
        ]
    )

def load_robot_points():
    """載入機械臂點位數據"""
    global robot_points
    
    points_file = os.path.join(current_dir, "saved_points", "robot_points.json")
    
    try:
        with open(points_file, "r", encoding="utf-8") as f:
            points_list = json.load(f)
        
        robot_points.clear()
        for point_data in points_list:
            # 支援pose或cartesian格式
            if "pose" in point_data:
                pose_data = point_data["pose"]
            elif "cartesian" in point_data:
                pose_data = point_data["cartesian"]
            else:
                continue
                
            if "joint" not in point_data:
                continue
                
            joint_data = point_data["joint"]
            
            robot_points[point_data["name"]] = {
                'x': float(pose_data["x"]),
                'y': float(pose_data["y"]), 
                'z': float(pose_data["z"]),
                'r': float(pose_data["r"]),
                'j1': float(joint_data["j1"]),
                'j2': float(joint_data["j2"]),
                'j3': float(joint_data["j3"]),
                'j4': float(joint_data["j4"])
            }
        
        logging.info(f"載入機械臂點位成功，共{len(robot_points)}個點位")
        return True
        
    except Exception as e:
        logging.error(f"載入機械臂點位失敗: {e}")
        return False

def object_manager_thread():
    """物件管理器執行緒 - 初始化所有子系統"""
    global robot
    
    try:
        system_lock.acquire()
        logging.info("物件管理器啟動 - 開始初始化子系統")
        
        # 創建機器人配置
        config = RobotConfig(
            dobot_ip="192.168.1.6",
            ccd1_ip="192.168.1.8", 
            ccd3_ip="192.168.1.10",
            vp_ip="192.168.1.7",
            auto_initialize=True,
            enable_logging=True
        )
        
        # 初始化機器人系統
        robot = Robot(config)
        if not robot.initialize_all_systems():
            logging.error("機器人系統初始化失敗")
            return False
        
        # 機械臂連接和使能
        if robot.Dobot and robot.Dobot.is_connected():
            if not robot.dobot_enable():
                logging.error("機械臂使能失敗")
                return False
            logging.info("機械臂連接並使能成功")
        else:
            logging.error("機械臂連接失敗")
            return False
        
        # 夾爪連接和快速關閉
        if robot.Gripper and robot.Gripper.is_connected():
            robot.gripper_quick_close()
            logging.info("夾爪連接並關閉成功")
        else:
            logging.warning("夾爪連接失敗，系統將繼續運作")
        
        # CCD1連接和設定模型1
        if robot.CCD1:
            robot.switch_ccd1_model(1)
            logging.info("CCD1連接並設定模型1成功")
        else:
            logging.error("CCD1連接失敗")
            return False
        
        # CCD3連接和初始化
        if robot.CCD3:
            logging.info("CCD3連接並初始化成功")
        else:
            logging.error("CCD3連接失敗")
            return False
        
        # 載入機械臂點位
        if not load_robot_points():
            logging.error("載入機械臂點位失敗")
            return False
        
        logging.info("物件管理器初始化完成")
        return True
        
    except Exception as e:
        logging.error(f"物件管理器初始化異常: {e}")
        return False
    finally:
        system_lock.release()

def parameter_setup_thread():
    """參數設定事件執行緒"""
    global robot, ccd1_model_id, gripper_opening, vibration_intensity, vibration_frequency
    
    try:
        logging.info("參數設定事件啟動")
        
        # 等待物件管理器完成
        if not object_manager_event.wait(timeout=30):
            logging.error("等待物件管理器超時")
            return False
        
        system_lock.acquire()
        
        # 設定CCD1模型
        if robot.CCD1:
            robot.switch_ccd1_model(ccd1_model_id)
            logging.info(f"CCD1模型切換到: {ccd1_model_id}")
        
        # 設定夾爪開度
        if robot.Gripper and robot.Gripper.is_connected():
            robot.gripper_smart_release(gripper_opening)
            logging.info(f"夾爪開度設定為: {gripper_opening}")
        
        # 設定震動盤參數
        if robot.VP:
            robot.vp_set_backlight(True, 50)
            logging.info(f"震動盤參數設定 - 強度:{vibration_intensity}, 頻率:{vibration_frequency}")
        
        logging.info("參數設定完成")
        return True
        
    except Exception as e:
        logging.error(f"參數設定異常: {e}")
        return False
    finally:
        system_lock.release()

def auto_feeding_thread():
    """自動進料事件執行緒"""
    global robot, material_available, label_0_objects, label_1_objects, label_2_objects, flow1_execution_count
    
    try:
        logging.info("自動進料事件啟動")
        
        # 等待參數設定完成
        if not parameter_setup_event.wait(timeout=30):
            logging.error("等待參數設定超時")
            return False
        
        retry_count = 0
        max_retries = 10
        
        while retry_count < max_retries:
            try:
                system_lock.acquire()
                
                # CCD1拍照檢測
                detections = robot.get_detections()
                if not detections:
                    logging.error("CCD1檢測失敗")
                    return False
                
                # 獲取各標籤物件數量和座標
                coords = robot.get_ccd1_coordinates()
                world_coords = coords.get('world_coordinates', {})
                
                label_0_count = len(world_coords.get(0, []))
                label_1_count = len(world_coords.get(1, []))  
                label_2_count = len(world_coords.get(2, []))
                
                logging.info(f"檢測結果 - 標籤0:{label_0_count}, 標籤1:{label_1_count}, 標籤2:{label_2_count}")
                
                # 判斷標籤0數量
                if label_0_count >= 1:
                    # 更新全域變數
                    label_0_objects = world_coords.get(0, [])
                    label_1_objects = world_coords.get(1, [])
                    label_2_objects = world_coords.get(2, [])
                    flow1_execution_count = 0  # 重置Flow1執行次數
                    
                    logging.info(f"物料準備完成 - 標籤0數量:{len(label_0_objects)}")
                    return True
                
                # 標籤0數量為0，檢查標籤1+標籤2數量
                total_other_count = label_1_count + label_2_count
                
                if total_other_count < 5:
                    # 數量少於5，執行震動進料
                    logging.info("執行震動進料控制")
                    robot.vibration_feed()
                else:
                    # 數量大於5，開始震動盤震動直到標籤0出現
                    logging.info("開始震動盤震動")
                    robot.vp_start_vibration("vertical", vibration_intensity, vibration_frequency, 0)
                    
                    # 持續檢測直到標籤0出現
                    shake_retry = 0
                    while shake_retry < 10:
                        time.sleep(1)
                        detections = robot.get_detections()
                        if detections:
                            coords = robot.get_ccd1_coordinates()
                            world_coords = coords.get('world_coordinates', {})
                            new_label_0_count = len(world_coords.get(0, []))
                            
                            if new_label_0_count >= 1:
                                robot.vp_stop_vibration()
                                label_0_objects = world_coords.get(0, [])
                                label_1_objects = world_coords.get(1, [])
                                label_2_objects = world_coords.get(2, [])
                                flow1_execution_count = 0
                                logging.info(f"震動成功 - 標籤0數量:{len(label_0_objects)}")
                                return True
                        
                        shake_retry += 1
                    
                    robot.vp_stop_vibration()
                
                retry_count += 1
                logging.info(f"重試第{retry_count}次")
                
            finally:
                system_lock.release()
            
            time.sleep(0.5)
        
        # 連續10次都沒有標籤0，設定沒料狀態
        material_available = False
        logging.error("連續10次檢測無標籤0物件，設定原物料狀態為沒料")
        return False
        
    except Exception as e:
        logging.error(f"自動進料異常: {e}")
        return False

def robot_pickup_thread():
    """機械臂上料事件執行緒 - Flow1流程直接實作"""
    global robot, label_0_objects, flow1_execution_count, current_angle, command_angle
    
    try:
        logging.info("機械臂上料事件啟動(Flow1流程)")
        
        # 等待自動進料完成
        if not auto_feeding_event.wait(timeout=60):
            logging.error("等待自動進料超時")
            return False
        
        # 檢查是否有可用的標籤0物件
        if flow1_execution_count >= len(label_0_objects):
            logging.info("所有標籤0物件已處理完成，需要重新進料")
            # 觸發自動進料事件
            auto_feeding_event.clear()
            threading.Thread(target=auto_feeding_thread, daemon=True).start()
            auto_feeding_event.wait(timeout=60)
        
        system_lock.acquire()
        
        try:
            # 獲取當前要處理的物件座標
            if flow1_execution_count >= len(label_0_objects):
                logging.error("沒有可用的標籤0物件座標")
                return False
            
            target_coord = label_0_objects[flow1_execution_count]
            logging.info(f"處理第{flow1_execution_count + 1}個物件，座標:({target_coord[0]:.2f}, {target_coord[1]:.2f})")
            
            # Flow1流程開始
            # 1. 切換到左手手勢
            if hasattr(robot.Dobot, 'dashboard_api') and robot.Dobot.dashboard_api:
                try:
                    robot.Dobot.dashboard_api.SetArmOrientation(1)
                    logging.info("切換到左手手勢")
                except:
                    logging.warning("手勢切換失敗，繼續執行")
            
            # 2. 移動到standby
            if "standby" in robot_points:
                point = robot_points["standby"]
                robot.dobot_movj_coord(point['x'], point['y'], point['z'], point['r'])
                robot.dobot_sync()
                logging.info("移動到standby完成")
            
            # 3. 移動到VP_TOPSIDE
            if "VP_TOPSIDE" in robot_points:
                point = robot_points["VP_TOPSIDE"]
                robot.dobot_movj_coord(point['x'], point['y'], point['z'], point['r'])
                robot.dobot_sync()
                logging.info("移動到VP_TOPSIDE完成")
            
            # 4. 移動到標籤0的XY座標(VP_TOPSIDE同高)
            vp_point = robot_points["VP_TOPSIDE"]
            robot.dobot_movl_coord(target_coord[0], target_coord[1], vp_point['z'], vp_point['r'])
            robot.dobot_sync()
            logging.info("移動到檢測位置上方完成")
            
            # 5. 移動到標籤0的XY座標(PICKUP_HEIGHT高度)
            robot.dobot_movl_coord(target_coord[0], target_coord[1], PICKUP_HEIGHT, vp_point['r'])
            robot.dobot_sync()
            logging.info("移動到夾取位置完成")
            
            # 6. 撐開夾爪
            robot.gripper_smart_release(gripper_opening)
            time.sleep(0.3)
            logging.info("夾爪撐開完成")
            
            # 7. 返回VP_TOPSIDE
            robot.dobot_movl_coord(vp_point['x'], vp_point['y'], vp_point['z'], vp_point['r'])
            robot.dobot_sync()
            logging.info("返回VP_TOPSIDE完成")
            
            # 8. 返回standby
            standby_point = robot_points["standby"]
            robot.dobot_movj_coord(standby_point['x'], standby_point['y'], standby_point['z'], standby_point['r'])
            robot.dobot_sync()
            logging.info("返回standby完成")
            
            # 9. 移動到Rotate_top
            if "Rotate_top" in robot_points:
                point = robot_points["Rotate_top"]
                robot.dobot_movj_coord(point['x'], point['y'], point['z'], point['r'])
                robot.dobot_sync()
                logging.info("移動到Rotate_top完成")
            
            # 10. 移動到Rotate_down
            if "Rotate_down" in robot_points:
                point = robot_points["Rotate_down"]
                robot.dobot_movj_coord(point['x'], point['y'], point['z'], point['r'])
                robot.dobot_sync()
                logging.info("移動到Rotate_down完成")
            
            # 11. 夾爪關閉
            robot.gripper_quick_close()
            time.sleep(0.3)
            logging.info("夾爪關閉完成")
            
            # 12. 返回Rotate_top
            rotate_top_point = robot_points["Rotate_top"]
            robot.dobot_movj_coord(rotate_top_point['x'], rotate_top_point['y'], rotate_top_point['z'], rotate_top_point['r'])
            robot.dobot_sync()
            logging.info("返回Rotate_top完成")
            
            # 13. 返回standby
            robot.dobot_movj_coord(standby_point['x'], standby_point['y'], standby_point['z'], standby_point['r'])
            robot.dobot_sync()
            logging.info("返回standby完成")
            
            # 14. CCD3獲取角度
            try:
                angle_result = robot.get_angle()
                if angle_result is not None:
                    current_angle = angle_result
                    command_angle = current_angle + ANGLE_OFFSET
                    logging.info(f"CCD3角度檢測成功 - 檢測角度:{current_angle:.2f}°, 指令角度:{command_angle:.2f}°")
                else:
                    current_angle = 0.0
                    command_angle = ANGLE_OFFSET
                    logging.warning("CCD3角度檢測失敗，使用預設值")
            except Exception as e:
                logging.error(f"CCD3角度檢測異常: {e}")
                current_angle = 0.0
                command_angle = ANGLE_OFFSET
            
            # 15. 移動到Rotate_top
            robot.dobot_movj_coord(rotate_top_point['x'], rotate_top_point['y'], rotate_top_point['z'], rotate_top_point['r'])
            robot.dobot_sync()
            
            # 16. 移動到Rotate_down
            rotate_down_point = robot_points["Rotate_down"]
            robot.dobot_movj_coord(rotate_down_point['x'], rotate_down_point['y'], rotate_down_point['z'], rotate_down_point['r'])
            robot.dobot_sync()
            
            # 17. 撐開夾爪
            robot.gripper_smart_release(gripper_opening)
            time.sleep(0.3)
            
            # 18. 返回Rotate_top
            robot.dobot_movj_coord(rotate_top_point['x'], rotate_top_point['y'], rotate_top_point['z'], rotate_top_point['r'])
            robot.dobot_sync()
            
            # 19. 移動到put_asm_top
            if "put_asm_top" in robot_points:
                point = robot_points["put_asm_top"]
                robot.dobot_movj_coord(point['x'], point['y'], point['z'], point['r'])
                robot.dobot_sync()
                logging.info("移動到put_asm_top完成")
            
            # 更新Flow1執行次數
            flow1_execution_count += 1
            logging.info(f"Flow1流程完成，執行次數: {flow1_execution_count}")
            
            return True
            
        except Exception as e:
            logging.error(f"Flow1流程執行異常: {e}")
            return False
            
    finally:
        system_lock.release()

def robot_assembly_thread():
    """機械臂組裝事件執行緒 - Flow2流程直接實作"""
    global robot, command_angle
    
    try:
        logging.info("機械臂組裝事件啟動(Flow2流程)")
        
        # 等待機械臂上料完成
        if not robot_pickup_event.wait(timeout=60):
            logging.error("等待機械臂上料超時")
            return False
        
        # 等待5秒
        time.sleep(5)
        
        system_lock.acquire()
        
        try:
            # Flow2流程開始
            logging.info(f"開始組裝流程，使用角度: {command_angle:.2f}°")
            
            # 1. 移動到put_asm_top (帶J4角度)
            if "put_asm_top" in robot_points:
                point = robot_points["put_asm_top"]
                robot.Dobot.joint_movj(point['j1'], point['j2'], point['j3'], command_angle)
                robot.dobot_sync()
                logging.info(f"移動到put_asm_top完成 (J4角度:{command_angle:.2f}°)")
            
            # 2. 移動到put_asm_down (帶J4角度)
            if "put_asm_down" in robot_points:
                point = robot_points["put_asm_down"]
                robot.Dobot.joint_movj(point['j1'], point['j2'], point['j3'], command_angle)
                robot.dobot_sync()
                logging.info(f"移動到put_asm_down完成 (J4角度:{command_angle:.2f}°)")
            
            # 3. 夾爪關閉
            robot.gripper_quick_close()
            time.sleep(0.3)
            logging.info("夾爪關閉完成")
            
            # 4. 返回put_asm_top
            put_asm_top_point = robot_points["put_asm_top"]
            robot.Dobot.joint_movj(put_asm_top_point['j1'], put_asm_top_point['j2'], put_asm_top_point['j3'], command_angle)
            robot.dobot_sync()
            logging.info("返回put_asm_top完成")
            
            # 5. 返回standby
            if "standby" in robot_points:
                point = robot_points["standby"]
                robot.dobot_movj_coord(point['x'], point['y'], point['z'], point['r'])
                robot.dobot_sync()
                logging.info("返回standby完成")
            
            logging.info("Flow2組裝流程完成")
            return True
            
        except Exception as e:
            logging.error(f"Flow2流程執行異常: {e}")
            return False
            
    finally:
        system_lock.release()

def main():
    """主程式入口"""
    setup_logging()
    logging.info("自動化系統啟動")
    
    try:
        # 1. 啟動物件管理器執行緒
        object_manager_thread_handle = threading.Thread(target=object_manager_thread, daemon=True)
        object_manager_thread_handle.start()
        object_manager_thread_handle.join()
        object_manager_event.set()
        
        # 2. 啟動參數設定事件
        parameter_setup_thread_handle = threading.Thread(target=parameter_setup_thread, daemon=True)
        parameter_setup_thread_handle.start()
        parameter_setup_thread_handle.join()
        parameter_setup_event.set()
        
        # 3. 啟動自動進料事件
        auto_feeding_thread_handle = threading.Thread(target=auto_feeding_thread, daemon=True)
        auto_feeding_thread_handle.start()
        auto_feeding_thread_handle.join()
        auto_feeding_event.set()
        
        # 4. 主循環 - 重複執行上料和組裝
        while material_available:
            # 機械臂上料事件
            robot_pickup_thread_handle = threading.Thread(target=robot_pickup_thread, daemon=True)
            robot_pickup_thread_handle.start()
            robot_pickup_thread_handle.join()
            robot_pickup_event.set()
            
            # 機械臂組裝事件
            robot_assembly_thread_handle = threading.Thread(target=robot_assembly_thread, daemon=True)
            robot_assembly_thread_handle.start()
            robot_assembly_thread_handle.join()
            robot_assembly_event.set()
            
            # 重置事件以便下次循環
            robot_pickup_event.clear()
            robot_assembly_event.clear()
            
            logging.info("一輪生產循環完成")
            
            # 檢查是否需要重新進料
            if flow1_execution_count >= len(label_0_objects):
                auto_feeding_event.clear()
                auto_feeding_thread_handle = threading.Thread(target=auto_feeding_thread, daemon=True)
                auto_feeding_thread_handle.start()
                auto_feeding_thread_handle.join()
                auto_feeding_event.set()
        
        logging.info("系統停止 - 原物料不足")
        
    except KeyboardInterrupt:
        logging.info("收到中斷信號，正在關閉系統")
    except Exception as e:
        logging.error(f"系統運行異常: {e}")
    finally:
        # 清理資源
        if robot:
            robot.emergency_stop()
            robot.disconnect_all()
        logging.info("系統已安全關閉")

if __name__ == "__main__":
    main()