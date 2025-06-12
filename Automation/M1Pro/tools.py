import sys
import os
import json
import time
import threading
from datetime import datetime
from threading import Thread
import numpy as np
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType
from pymodbus.client import ModbusTcpClient

# 機械臂模式定義
LABEL_ROBOT_MODE = {
    1: "ROBOT_MODE_INIT",
    2: "ROBOT_MODE_BRAKE_OPEN", 
    3: "",
    4: "ROBOT_MODE_DISABLED",
    5: "ROBOT_MODE_ENABLE",
    6: "ROBOT_MODE_BACKDRIVE",
    7: "ROBOT_MODE_RUNNING",
    8: "ROBOT_MODE_RECORDING",
    9: "ROBOT_MODE_ERROR",
    10: "ROBOT_MODE_PAUSE",
    11: "ROBOT_MODE_JOG"
}

class StatusUpdateSignal(QObject):
    """狀態更新信號"""
    feedback_update = pyqtSignal(dict)
    log_update = pyqtSignal(str)
    error_update = pyqtSignal(str)

class PointEditDialog(QDialog):
    """點位編輯對話框"""
    def __init__(self, point_data=None, parent=None):
        super().__init__(parent)
        self.point_data = point_data or {}
        self.setupUI()
        self.load_data()
        
    def setupUI(self):
        self.setWindowTitle("編輯點位")
        self.setFixedSize(400, 350)
        layout = QVBoxLayout()
        
        # 名稱輸入
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel("點位名稱:"))
        self.name_edit = QLineEdit()
        name_layout.addWidget(self.name_edit)
        layout.addLayout(name_layout)
        
        # 笛卡爾座標
        cart_group = QGroupBox("笛卡爾座標 (mm/度)")
        cart_layout = QGridLayout()
        self.x_spin = QDoubleSpinBox()
        self.x_spin.setRange(-1000, 1000)
        self.x_spin.setDecimals(2)
        self.y_spin = QDoubleSpinBox()
        self.y_spin.setRange(-1000, 1000)
        self.y_spin.setDecimals(2)
        self.z_spin = QDoubleSpinBox()
        self.z_spin.setRange(-100, 800)
        self.z_spin.setDecimals(2)
        self.r_spin = QDoubleSpinBox()
        self.r_spin.setRange(-180, 180)
        self.r_spin.setDecimals(2)
        
        cart_layout.addWidget(QLabel("X:"), 0, 0)
        cart_layout.addWidget(self.x_spin, 0, 1)
        cart_layout.addWidget(QLabel("Y:"), 0, 2)
        cart_layout.addWidget(self.y_spin, 0, 3)
        cart_layout.addWidget(QLabel("Z:"), 1, 0)
        cart_layout.addWidget(self.z_spin, 1, 1)
        cart_layout.addWidget(QLabel("R:"), 1, 2)
        cart_layout.addWidget(self.r_spin, 1, 3)
        cart_group.setLayout(cart_layout)
        layout.addWidget(cart_group)
        
        # 關節座標
        joint_group = QGroupBox("關節座標 (度)")
        joint_layout = QGridLayout()
        self.j1_spin = QDoubleSpinBox()
        self.j1_spin.setRange(-180, 180)
        self.j1_spin.setDecimals(2)
        self.j2_spin = QDoubleSpinBox()
        self.j2_spin.setRange(-135, 135)
        self.j2_spin.setDecimals(2)
        self.j3_spin = QDoubleSpinBox()
        self.j3_spin.setRange(-135, 135)
        self.j3_spin.setDecimals(2)
        self.j4_spin = QDoubleSpinBox()
        self.j4_spin.setRange(-180, 180)
        self.j4_spin.setDecimals(2)
        
        joint_layout.addWidget(QLabel("J1:"), 0, 0)
        joint_layout.addWidget(self.j1_spin, 0, 1)
        joint_layout.addWidget(QLabel("J2:"), 0, 2)
        joint_layout.addWidget(self.j2_spin, 0, 3)
        joint_layout.addWidget(QLabel("J3:"), 1, 0)
        joint_layout.addWidget(self.j3_spin, 1, 1)
        joint_layout.addWidget(QLabel("J4:"), 1, 2)
        joint_layout.addWidget(self.j4_spin, 1, 3)
        joint_group.setLayout(joint_layout)
        layout.addWidget(joint_group)
        
        # 按鈕
        button_layout = QHBoxLayout()
        self.ok_btn = QPushButton("確定")
        self.cancel_btn = QPushButton("取消")
        self.ok_btn.clicked.connect(self.accept)
        self.cancel_btn.clicked.connect(self.reject)
        button_layout.addWidget(self.ok_btn)
        button_layout.addWidget(self.cancel_btn)
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
        
    def load_data(self):
        """載入點位數據"""
        if self.point_data:
            self.name_edit.setText(self.point_data.get('name', ''))
            cartesian = self.point_data.get('cartesian', {})
            self.x_spin.setValue(cartesian.get('x', 0))
            self.y_spin.setValue(cartesian.get('y', 0))
            self.z_spin.setValue(cartesian.get('z', 0))
            self.r_spin.setValue(cartesian.get('r', 0))
            
            joint = self.point_data.get('joint', {})
            self.j1_spin.setValue(joint.get('j1', 0))
            self.j2_spin.setValue(joint.get('j2', 0))
            self.j3_spin.setValue(joint.get('j3', 0))
            self.j4_spin.setValue(joint.get('j4', 0))
    
    def get_data(self):
        """獲取編輯後的數據"""
        return {
            'name': self.name_edit.text(),
            'cartesian': {
                'x': self.x_spin.value(),
                'y': self.y_spin.value(),
                'z': self.z_spin.value(),
                'r': self.r_spin.value()
            },
            'joint': {
                'j1': self.j1_spin.value(),
                'j2': self.j2_spin.value(),
                'j3': self.j3_spin.value(),
                'j4': self.j4_spin.value()
            },
            'created_time': self.point_data.get('created_time', datetime.now().isoformat()),
            'modified_time': datetime.now().isoformat()
        }
    
    def validate_position_change(self, original, new):
        """驗證位置變化是否安全"""
        if not original:
            return True
            
        orig_cart = original.get('cartesian', {})
        new_cart = new.get('cartesian', {})
        
        # 檢查變化幅度 (10cm = 100mm)
        max_change = 100.0
        for key in ['x', 'y', 'z']:
            old_val = orig_cart.get(key, 0)
            new_val = new_cart.get(key, 0)
            if abs(new_val - old_val) > max_change:
                return False
                
        # 檢查是否正負相反
        for key in ['x', 'y', 'z']:
            old_val = orig_cart.get(key, 0)
            new_val = new_cart.get(key, 0)
            if old_val != 0 and new_val != 0:
                if (old_val > 0) != (new_val > 0):  # 正負號不同
                    return False
        return True

class RobotUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MG400/M1Pro Python Demo - PyQt版")
        self.setFixedSize(1400, 1000)
        
        # 狀態變量
        self.global_state = {
            'connect': False,
            'enable': False
        }
        
        # 連接客戶端
        self.client_dash = None
        self.client_move = None
        self.client_feed = None
        self.modbus_client = None  # PGC夾爪控制
        
        # 信號槽
        self.signals = StatusUpdateSignal()
        self.signals.feedback_update.connect(self.update_feedback_display)
        self.signals.log_update.connect(self.append_log)
        self.signals.error_update.connect(self.append_error)
        
        # 實際機械臂位置數據 (從反饋獲取)
        self.current_position = {
            'cartesian': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'r': 0.0},
            'joint': {'j1': 0.0, 'j2': 0.0, 'j3': 0.0, 'j4': 0.0}
        }
        
        # 點位數據
        self.saved_points = []
        self.points_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                                       'saved_points', 'robot_points.json')
        
        # 確保資料夾存在
        os.makedirs(os.path.dirname(self.points_file), exist_ok=True)
        
        self.setupUI()
        self.load_points()
        
    def setupUI(self):
        """建立UI界面"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 設定窗口圖標和標題
        self.setWindowTitle("MG400/M1Pro Python Demo - PyQt版 (ESC=緊急停止)")
        
        # 主布局
        main_layout = QVBoxLayout()
        
        # 連接設定區域
        main_layout.addWidget(self.create_connection_group())
        
        # 中間區域 - 左右分割
        middle_layout = QHBoxLayout()
        
        # 左側區域
        left_layout = QVBoxLayout()
        left_layout.addWidget(self.create_dashboard_group())
        left_layout.addWidget(self.create_move_group())
        left_layout.addWidget(self.create_gripper_group())  # PGC夾爪控制
        
        # 右側區域 - 點位管理
        right_layout = QVBoxLayout()
        right_layout.addWidget(self.create_points_group())
        
        # 設定左右比例
        left_widget = QWidget()
        left_widget.setLayout(left_layout)
        right_widget = QWidget()
        right_widget.setLayout(right_layout)
        
        middle_layout.addWidget(left_widget, 2)  # 左側佔2/3
        middle_layout.addWidget(right_widget, 1)  # 右側佔1/3
        
        main_layout.addLayout(middle_layout)
        
        # 底部狀態顯示區域
        bottom_layout = QHBoxLayout()
        bottom_layout.addWidget(self.create_feedback_group(), 2)
        bottom_layout.addWidget(self.create_log_group(), 1)
        
        main_layout.addLayout(bottom_layout)
        
        central_widget.setLayout(main_layout)
        
        # 設定焦點策略，確保能接收鍵盤事件
        self.setFocusPolicy(Qt.StrongFocus)
        
    def create_connection_group(self):
        """建立連接設定群組"""
        group = QGroupBox("機械臂連接設定")
        layout = QHBoxLayout()
        
        layout.addWidget(QLabel("IP:"))
        self.ip_edit = QLineEdit("192.168.1.6")
        self.ip_edit.setFixedWidth(120)
        layout.addWidget(self.ip_edit)
        
        layout.addWidget(QLabel("Dashboard:"))
        self.dash_edit = QLineEdit("29999")
        self.dash_edit.setFixedWidth(60)
        layout.addWidget(self.dash_edit)
        
        layout.addWidget(QLabel("Move:"))
        self.move_edit = QLineEdit("30003")
        self.move_edit.setFixedWidth(60)
        layout.addWidget(self.move_edit)
        
        layout.addWidget(QLabel("Feed:"))
        self.feed_edit = QLineEdit("30004")
        self.feed_edit.setFixedWidth(60)
        layout.addWidget(self.feed_edit)
        
        self.connect_btn = QPushButton("連接")
        self.connect_btn.clicked.connect(self.toggle_connection)
        layout.addWidget(self.connect_btn)
        
        layout.addStretch()
        group.setLayout(layout)
        return group
        
    def create_dashboard_group(self):
        """建立控制面板群組"""
        group = QGroupBox("機械臂控制")
        layout = QGridLayout()
        
        # 緊急停止按鈕 - 顯眼的紅色大按鈕
        self.emergency_stop_btn = QPushButton("緊急停止")
        self.emergency_stop_btn.setFixedSize(120, 60)
        self.emergency_stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #FF4444;
                color: white;
                font-size: 16px;
                font-weight: bold;
                border: 3px solid #CC0000;
                border-radius: 10px;
            }
            QPushButton:hover {
                background-color: #FF6666;
            }
            QPushButton:pressed {
                background-color: #CC0000;
            }
        """)
        self.emergency_stop_btn.clicked.connect(self.emergency_stop)
        layout.addWidget(self.emergency_stop_btn, 0, 0, 2, 1)  # 跨兩行顯示
        
        # 使能/下使能
        self.enable_btn = QPushButton("使能")
        self.enable_btn.clicked.connect(self.toggle_enable)
        self.enable_btn.setEnabled(False)
        layout.addWidget(self.enable_btn, 0, 1)
        
        # 重置機械臂
        reset_btn = QPushButton("重置機械臂")
        reset_btn.clicked.connect(self.reset_robot)
        reset_btn.setEnabled(False)
        layout.addWidget(reset_btn, 0, 2)
        
        # 清除錯誤
        clear_btn = QPushButton("清除錯誤")
        clear_btn.clicked.connect(self.clear_error)
        clear_btn.setEnabled(False)
        layout.addWidget(clear_btn, 0, 3)
        
        # 速度設定
        layout.addWidget(QLabel("速度比例:"), 1, 1)
        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(1, 100)
        self.speed_spin.setValue(50)
        self.speed_spin.setSuffix("%")
        layout.addWidget(self.speed_spin, 1, 2)
        
        speed_confirm_btn = QPushButton("確認速度")
        speed_confirm_btn.clicked.connect(self.confirm_speed)
        speed_confirm_btn.setEnabled(False)
        layout.addWidget(speed_confirm_btn, 1, 3)
        
        # DO控制
        layout.addWidget(QLabel("DO索引:"), 2, 0)
        self.do_index_spin = QSpinBox()
        self.do_index_spin.setRange(1, 24)
        layout.addWidget(self.do_index_spin, 2, 1)
        
        self.do_status_combo = QComboBox()
        self.do_status_combo.addItems(["高電平", "低電平"])
        layout.addWidget(self.do_status_combo, 2, 2)
        
        do_confirm_btn = QPushButton("設定DO")
        do_confirm_btn.clicked.connect(self.confirm_do)
        do_confirm_btn.setEnabled(False)
        layout.addWidget(do_confirm_btn, 2, 3)
        
        # 保存按鈕引用以便後續啟用/禁用 (不包含緊急停止按鈕)
        self.control_buttons = [reset_btn, clear_btn, speed_confirm_btn, do_confirm_btn]
        
        group.setLayout(layout)
        return group
        
    def create_move_group(self):
        """建立運動控制群組"""
        group = QGroupBox("運動控制")
        layout = QGridLayout()
        
        # 笛卡爾座標輸入
        layout.addWidget(QLabel("X:"), 0, 0)
        self.x_spin = QDoubleSpinBox()
        self.x_spin.setRange(-1000, 1000)
        self.x_spin.setValue(600)
        self.x_spin.setDecimals(2)
        layout.addWidget(self.x_spin, 0, 1)
        
        layout.addWidget(QLabel("Y:"), 0, 2)
        self.y_spin = QDoubleSpinBox()
        self.y_spin.setRange(-1000, 1000)
        self.y_spin.setValue(-260)
        self.y_spin.setDecimals(2)
        layout.addWidget(self.y_spin, 0, 3)
        
        layout.addWidget(QLabel("Z:"), 0, 4)
        self.z_spin = QDoubleSpinBox()
        self.z_spin.setRange(-100, 800)
        self.z_spin.setValue(380)
        self.z_spin.setDecimals(2)
        layout.addWidget(self.z_spin, 0, 5)
        
        layout.addWidget(QLabel("R:"), 0, 6)
        self.r_spin = QDoubleSpinBox()
        self.r_spin.setRange(-180, 180)
        self.r_spin.setValue(170)
        self.r_spin.setDecimals(2)
        layout.addWidget(self.r_spin, 0, 7)
        
        # 運動按鈕
        movj_btn = QPushButton("MovJ")
        movj_btn.clicked.connect(self.movj)
        movj_btn.setEnabled(False)
        layout.addWidget(movj_btn, 1, 0)
        
        movl_btn = QPushButton("MovL")
        movl_btn.clicked.connect(self.movl)
        movl_btn.setEnabled(False)
        layout.addWidget(movl_btn, 1, 1)
        
        # 速度控制
        layout.addWidget(QLabel("MovL速度:"), 1, 2)
        self.movl_speed_slider = QSlider(Qt.Horizontal)
        self.movl_speed_slider.setRange(1, 100)
        self.movl_speed_slider.setValue(50)
        layout.addWidget(self.movl_speed_slider, 1, 3, 1, 2)
        
        self.speed_label = QLabel("50%")
        self.movl_speed_slider.valueChanged.connect(
            lambda v: self.speed_label.setText(f"{v}%"))
        layout.addWidget(self.speed_label, 1, 5)
        
        # 關節座標輸入
        layout.addWidget(QLabel("J1:"), 2, 0)
        self.j1_spin = QDoubleSpinBox()
        self.j1_spin.setRange(-180, 180)
        self.j1_spin.setValue(0)
        self.j1_spin.setDecimals(2)
        layout.addWidget(self.j1_spin, 2, 1)
        
        layout.addWidget(QLabel("J2:"), 2, 2)
        self.j2_spin = QDoubleSpinBox()
        self.j2_spin.setRange(-135, 135)
        self.j2_spin.setValue(-20)
        self.j2_spin.setDecimals(2)
        layout.addWidget(self.j2_spin, 2, 3)
        
        layout.addWidget(QLabel("J3:"), 2, 4)
        self.j3_spin = QDoubleSpinBox()
        self.j3_spin.setRange(-135, 135)
        self.j3_spin.setValue(-80)
        self.j3_spin.setDecimals(2)
        layout.addWidget(self.j3_spin, 2, 5)
        
        layout.addWidget(QLabel("J4:"), 2, 6)
        self.j4_spin = QDoubleSpinBox()
        self.j4_spin.setRange(-180, 180)
        self.j4_spin.setValue(30)
        self.j4_spin.setDecimals(2)
        layout.addWidget(self.j4_spin, 2, 7)
        
        joint_movj_btn = QPushButton("JointMovJ")
        joint_movj_btn.clicked.connect(self.joint_movj)
        joint_movj_btn.setEnabled(False)
        layout.addWidget(joint_movj_btn, 3, 0)
        
        # 點動控制按鈕群組
        jog_group = QGroupBox("點動控制")
        jog_layout = QGridLayout()
        jog_layout.setSpacing(8)  # 增加按鈕間距
        
        # 關節點動
        self.create_jog_buttons(jog_layout, 0, ["J1-", "J1+"], ["j1-", "j1+"])
        self.create_jog_buttons(jog_layout, 1, ["J2-", "J2+"], ["j2-", "j2+"])
        self.create_jog_buttons(jog_layout, 2, ["J3-", "J3+"], ["j3-", "j3+"])
        self.create_jog_buttons(jog_layout, 3, ["J4-", "J4+"], ["j4-", "j4+"])
        
        # 笛卡爾點動
        self.create_jog_buttons(jog_layout, 0, ["X-", "X+"], ["x-", "x+"], col_offset=3)
        self.create_jog_buttons(jog_layout, 1, ["Y-", "Y+"], ["y-", "y+"], col_offset=3)
        self.create_jog_buttons(jog_layout, 2, ["Z-", "Z+"], ["z-", "z+"], col_offset=3)
        self.create_jog_buttons(jog_layout, 3, ["R-", "R+"], ["r-", "r+"], col_offset=3)
        
        jog_group.setLayout(jog_layout)
        layout.addWidget(jog_group, 4, 0, 1, 8)
        
        # 保存運動按鈕引用
        self.move_buttons = [movj_btn, movl_btn, joint_movj_btn]
        
        group.setLayout(layout)
        return group
        
    def create_jog_buttons(self, layout, row, labels, commands, col_offset=0):
        """建立點動按鈕"""
        for i, (label, cmd) in enumerate(zip(labels, commands)):
            btn = QPushButton(label)
            btn.setMinimumSize(60, 40)  # 設定最小尺寸
            btn.setEnabled(False)
            btn.pressed.connect(lambda c=cmd: self.start_jog(c))
            btn.released.connect(self.stop_jog)
            layout.addWidget(btn, row, i + col_offset)
            if not hasattr(self, 'jog_buttons'):
                self.jog_buttons = []
            self.jog_buttons.append(btn)
    
    def create_gripper_group(self):
        """建立PGC夾爪控制群組"""
        group = QGroupBox("PGC夾爪控制")
        layout = QGridLayout()
        
        # 連接狀態顯示
        layout.addWidget(QLabel("連接狀態:"), 0, 0)
        self.gripper_status_label = QLabel("未連接")
        self.gripper_status_label.setStyleSheet("color: red")
        layout.addWidget(self.gripper_status_label, 0, 1)
        
        # 夾爪狀態顯示
        layout.addWidget(QLabel("夾持狀態:"), 0, 2)
        self.gripper_hold_label = QLabel("未知")
        layout.addWidget(self.gripper_hold_label, 0, 3)
        
        # 位置顯示
        layout.addWidget(QLabel("當前位置:"), 0, 4)
        self.gripper_pos_label = QLabel("0")
        layout.addWidget(self.gripper_pos_label, 0, 5)
        
        # 控制按鈕
        init_btn = QPushButton("初始化")
        init_btn.clicked.connect(self.gripper_initialize)
        init_btn.setEnabled(False)
        layout.addWidget(init_btn, 1, 0)
        
        open_btn = QPushButton("快速開啟")
        open_btn.clicked.connect(self.gripper_open)
        open_btn.setEnabled(False)
        layout.addWidget(open_btn, 1, 1)
        
        close_btn = QPushButton("快速關閉")
        close_btn.clicked.connect(self.gripper_close)
        close_btn.setEnabled(False)
        layout.addWidget(close_btn, 1, 2)
        
        stop_btn = QPushButton("停止")
        stop_btn.clicked.connect(self.gripper_stop)
        stop_btn.setEnabled(False)
        layout.addWidget(stop_btn, 1, 3)
        
        # 位置控制
        layout.addWidget(QLabel("目標位置:"), 2, 0)
        self.gripper_pos_spin = QSpinBox()
        self.gripper_pos_spin.setRange(0, 1000)
        self.gripper_pos_spin.setValue(500)
        layout.addWidget(self.gripper_pos_spin, 2, 1)
        
        pos_btn = QPushButton("移動到位置")
        pos_btn.clicked.connect(self.gripper_move_to_pos)
        pos_btn.setEnabled(False)
        layout.addWidget(pos_btn, 2, 2)
        
        # 力道控制
        layout.addWidget(QLabel("夾持力道:"), 2, 3)
        self.gripper_force_spin = QSpinBox()
        self.gripper_force_spin.setRange(20, 100)
        self.gripper_force_spin.setValue(50)
        layout.addWidget(self.gripper_force_spin, 2, 4)
        
        force_btn = QPushButton("設定力道")
        force_btn.clicked.connect(self.gripper_set_force)
        force_btn.setEnabled(False)
        layout.addWidget(force_btn, 2, 5)
        
        # 保存夾爪按鈕引用
        self.gripper_buttons = [init_btn, open_btn, close_btn, stop_btn, pos_btn, force_btn]
        
        group.setLayout(layout)
        return group
    
    def create_points_group(self):
        """建立點位管理群組"""
        group = QGroupBox("點位管理")
        layout = QVBoxLayout()
        
        # 操作按鈕
        btn_layout = QHBoxLayout()
        
        save_btn = QPushButton("保存當前點位")
        save_btn.clicked.connect(self.save_current_point)
        save_btn.setEnabled(False)
        save_btn.setToolTip("保存機械臂當前實際位置為點位")
        btn_layout.addWidget(save_btn)
        
        sync_btn = QPushButton("同步到輸入框")
        sync_btn.clicked.connect(self.sync_current_position)
        sync_btn.setEnabled(False)
        sync_btn.setToolTip("將機械臂當前位置同步到移動控制輸入框")
        btn_layout.addWidget(sync_btn)
        
        edit_btn = QPushButton("編輯點位")
        edit_btn.clicked.connect(self.edit_selected_point)
        btn_layout.addWidget(edit_btn)
        
        delete_btn = QPushButton("刪除點位")
        delete_btn.clicked.connect(self.delete_selected_point)
        btn_layout.addWidget(delete_btn)
        
        layout.addLayout(btn_layout)
        
        # 點位列表
        self.points_list = QListWidget()
        self.points_list.itemDoubleClicked.connect(self.move_to_selected_point_with_dialog)
        layout.addWidget(self.points_list)
        
        # 移動控制區域
        move_control_group = QGroupBox("移動控制")
        move_control_layout = QVBoxLayout()
        
        # 運動類型選擇
        motion_type_layout = QHBoxLayout()
        motion_type_layout.addWidget(QLabel("運動類型:"))
        
        self.motion_type_combo = QComboBox()
        self.motion_type_combo.addItems([
            "直線運動(MovL) - 可能遇到手勢切換問題",
            "關節運動(MovJ) - 避免手勢切換問題", 
            "關節座標運動(JointMovJ) - 使用儲存關節角度"
        ])
        self.motion_type_combo.setCurrentIndex(1)  # 預設選擇MovJ
        motion_type_layout.addWidget(self.motion_type_combo)
        
        move_control_layout.addLayout(motion_type_layout)
        
        # 速度控制
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("移動速度:"))
        
        self.point_speed_slider = QSlider(Qt.Horizontal)
        self.point_speed_slider.setRange(1, 100)
        self.point_speed_slider.setValue(30)
        speed_layout.addWidget(self.point_speed_slider)
        
        self.point_speed_label = QLabel("30%")
        self.point_speed_slider.valueChanged.connect(
            lambda v: self.point_speed_label.setText(f"{v}%"))
        speed_layout.addWidget(self.point_speed_label)
        
        move_control_layout.addLayout(speed_layout)
        
        # 移動按鈕
        move_btn_layout = QHBoxLayout()
        
        move_btn = QPushButton("移動到選中點位")
        move_btn.clicked.connect(self.move_to_selected_point)
        move_btn.setEnabled(False)
        move_btn_layout.addWidget(move_btn)
        
        quick_move_btn = QPushButton("快速移動(使用預設類型)")
        quick_move_btn.clicked.connect(lambda: self.move_to_selected_point(use_preset=True))
        quick_move_btn.setEnabled(False)
        move_btn_layout.addWidget(quick_move_btn)
        
        move_control_layout.addLayout(move_btn_layout)
        
        move_control_group.setLayout(move_control_layout)
        layout.addWidget(move_control_group)
        
        # 保存點位按鈕引用
        self.point_buttons = [save_btn, sync_btn, move_btn, quick_move_btn]
        
        group.setLayout(layout)
        return group
        
    def create_feedback_group(self):
        """建立狀態反饋群組"""
        group = QGroupBox("狀態反饋")
        layout = QGridLayout()
        
        # 速度比例
        layout.addWidget(QLabel("當前速度比例:"), 0, 0)
        self.current_speed_label = QLabel("0%")
        layout.addWidget(self.current_speed_label, 0, 1)
        
        # 機械臂模式
        layout.addWidget(QLabel("機械臂模式:"), 0, 2)
        self.robot_mode_label = QLabel("未知")
        layout.addWidget(self.robot_mode_label, 0, 3)
        
        # 位置反饋
        layout.addWidget(QLabel("當前位置(X,Y,Z,R):"), 1, 0)
        self.position_label = QLabel("0.00, 0.00, 0.00, 0.00")
        layout.addWidget(self.position_label, 1, 1, 1, 3)
        
        # 關節角度反饋
        layout.addWidget(QLabel("關節角度(J1,J2,J3,J4):"), 2, 0)
        self.joint_label = QLabel("0.00, 0.00, 0.00, 0.00")
        layout.addWidget(self.joint_label, 2, 1, 1, 3)
        
        # 數位IO
        layout.addWidget(QLabel("數位輸入:"), 3, 0)
        self.di_label = QLabel("")
        layout.addWidget(self.di_label, 3, 1, 1, 3)
        
        layout.addWidget(QLabel("數位輸出:"), 4, 0)
        self.do_label = QLabel("")
        layout.addWidget(self.do_label, 4, 1, 1, 3)
        
        group.setLayout(layout)
        return group
        
    def create_log_group(self):
        """建立日誌群組"""
        group = QGroupBox("系統日誌")
        layout = QVBoxLayout()
        
        # 日誌顯示區域
        self.log_text = QTextEdit()
        self.log_text.setMaximumHeight(200)
        self.log_text.setReadOnly(True)
        layout.addWidget(self.log_text)
        
        # 錯誤顯示區域
        error_group = QGroupBox("錯誤信息")
        error_layout = QVBoxLayout()
        self.error_text = QTextEdit()
        self.error_text.setMaximumHeight(100)
        self.error_text.setReadOnly(True)
        error_layout.addWidget(self.error_text)
        
        clear_error_btn = QPushButton("清除錯誤信息")
        clear_error_btn.clicked.connect(self.clear_error_info)
        error_layout.addWidget(clear_error_btn)
        
        error_group.setLayout(error_layout)
        layout.addWidget(error_group)
        
        group.setLayout(layout)
        return group
    
    def toggle_connection(self):
        """切換連接狀態"""
        if self.global_state['connect']:
            self.disconnect_robot()
        else:
            self.connect_robot()
            
    def connect_robot(self):
        """連接機械臂"""
        try:
            ip = self.ip_edit.text()
            dash_port = int(self.dash_edit.text())
            move_port = int(self.move_edit.text())
            feed_port = int(self.feed_edit.text())
            
            self.append_log("正在連接機械臂...")
            
            self.client_dash = DobotApiDashboard(ip, dash_port)
            self.client_move = DobotApiMove(ip, move_port)
            self.client_feed = DobotApi(ip, feed_port)
            
            # 連接Modbus TCP用於PGC夾爪控制 - 使用PyModbus 3.x語法
            self.modbus_client = ModbusTcpClient(host='127.0.0.1', port=502)
            if self.modbus_client.connect():
                self.append_log("Modbus TCP連接成功")
                # 測試讀取PGC狀態寄存器
                result = self.modbus_client.read_holding_registers(address=500, count=1, slave=1)
                if not result.isError():
                    self.append_log("PGC夾爪寄存器讀取成功")
                else:
                    self.append_log(f"PGC夾爪寄存器讀取失敗: {result}")
            else:
                self.append_log("Modbus TCP連接失敗")
            
            self.global_state['connect'] = True
            self.connect_btn.setText("斷開")
            
            # 啟用控制按鈕
            self.enable_controls(True)
            
            # 啟動反饋線程
            self.start_feedback_thread()
            
            self.append_log("機械臂連接成功")
            
        except Exception as e:
            QMessageBox.critical(self, "連接錯誤", f"連接失敗: {str(e)}")
            self.append_log(f"連接失敗: {str(e)}")
            
    def disconnect_robot(self):
        """斷開機械臂連接"""
        try:
            if self.client_dash:
                self.client_dash.close()
            if self.client_move:
                self.client_move.close()
            if self.client_feed:
                self.client_feed.close()
            if self.modbus_client:
                self.modbus_client.close()
                
            self.global_state['connect'] = False
            self.connect_btn.setText("連接")
            
            # 禁用控制按鈕
            self.enable_controls(False)
            
            self.append_log("機械臂連接已斷開")
            
        except Exception as e:
            self.append_log(f"斷開連接錯誤: {str(e)}")
            
    def enable_controls(self, enabled):
        """啟用/禁用控制按鈕"""
        self.enable_btn.setEnabled(enabled)
        # 緊急停止按鈕始終可用，不受連接狀態影響
        for btn in self.control_buttons + self.move_buttons + self.jog_buttons + self.gripper_buttons + self.point_buttons:
            btn.setEnabled(enabled)
            
    def emergency_stop(self):
        """緊急停止功能"""
        try:
            if self.global_state['connect'] and self.client_dash:
                # 發送緊急停止指令到機械臂
                result = self.client_dash.EmergencyStop()
                self.append_log(f"緊急停止指令已發送: {result}")
                
                # 停止所有點動操作
                if self.client_move:
                    self.client_move.MoveJog("")
                
                # 發送夾爪緊急停止指令
                if self.modbus_client and self.modbus_client.connected:
                    try:
                        # PGC夾爪緊急停止 (指令2: 停止)
                        command_id = int(time.time() * 1000) % 65535
                        self.modbus_client.write_register(address=520, value=2, slave=1)
                        self.modbus_client.write_register(address=523, value=command_id, slave=1)
                        self.append_log("PGC夾爪緊急停止指令已發送")
                    except Exception as e:
                        self.append_log(f"PGC夾爪緊急停止失敗: {str(e)}")
                
                # 顯示緊急停止警告
                QMessageBox.warning(self, "緊急停止", 
                    "緊急停止已觸發！\n" +
                    "機械臂已停止所有運動。\n" +
                    "請檢查安全狀況後重新使能機械臂。")
                
                # 自動下使能機械臂
                if self.global_state['enable']:
                    self.global_state['enable'] = False
                    self.enable_btn.setText("使能")
                
            else:
                # 即使未連接也要顯示緊急停止被按下的提示
                self.append_log("緊急停止按鈕被按下 (機械臂未連接)")
                QMessageBox.information(self, "緊急停止", "緊急停止按鈕已被按下。")
                
        except Exception as e:
            error_msg = f"緊急停止執行失敗: {str(e)}"
            self.append_log(error_msg)
            QMessageBox.critical(self, "緊急停止錯誤", error_msg)
    
    def keyPressEvent(self, event):
        """鍵盤快捷鍵處理"""
        # 按 ESC 鍵觸發緊急停止
        if event.key() == Qt.Key_Escape:
            self.emergency_stop()
        # 按 Space 鍵停止所有點動
        elif event.key() == Qt.Key_Space:
            if self.global_state['connect'] and self.client_move:
                self.client_move.MoveJog("")
                self.append_log("空白鍵停止點動")
        else:
            super().keyPressEvent(event)
            
    def reset_robot(self):
        """重置機械臂"""
        self.client_dash.ResetRobot()
        self.append_log("機械臂重置")
        
    def clear_error(self):
        """清除錯誤"""
        self.client_dash.ClearError()
        self.append_log("錯誤已清除")
        
    def confirm_speed(self):
        """確認速度設定"""
        speed = self.speed_spin.value()
        self.client_dash.SpeedFactor(speed)
        self.append_log(f"速度比例設定為 {speed}%")
        
    def confirm_do(self):
        """確認DO設定"""
        index = self.do_index_spin.value()
        status = 1 if self.do_status_combo.currentText() == "高電平" else 0
        self.client_dash.DO(index, status)
        self.append_log(f"DO{index} 設定為 {'高電平' if status else '低電平'}")
        
    def movj(self):
        """關節運動"""
        x, y, z, r = self.get_cartesian_values()
        self.client_move.MovJ(x, y, z, r)
        self.append_log(f"MovJ to ({x}, {y}, {z}, {r})")
        
    def movl(self):
        """直線運動"""
        x, y, z, r = self.get_cartesian_values()
        speed = self.movl_speed_slider.value()
        speed_param = f"SpeedL={speed}"
        self.client_move.MovL(x, y, z, r, speed_param)
        self.append_log(f"MovL to ({x}, {y}, {z}, {r}) at {speed}% speed")
        
    def joint_movj(self):
        """關節移動"""
        j1, j2, j3, j4 = self.get_joint_values()
        self.client_move.JointMovJ(j1, j2, j3, j4)
        self.append_log(f"JointMovJ to ({j1}, {j2}, {j3}, {j4})")
        
    def get_cartesian_values(self):
        """獲取笛卡爾座標值"""
        return (self.x_spin.value(), self.y_spin.value(), 
                self.z_spin.value(), self.r_spin.value())
                
    def get_joint_values(self):
        """獲取關節角度值"""
        return (self.j1_spin.value(), self.j2_spin.value(),
                self.j3_spin.value(), self.j4_spin.value())
                
    def start_jog(self, command):
        """開始點動"""
        if self.global_state['connect']:
            self.client_move.MoveJog(command)
            
    def stop_jog(self):
        """停止點動"""
        if self.global_state['connect']:
            self.client_move.MoveJog("")
    
    # PGC夾爪控制功能
    def send_gripper_command(self, cmd, param1=0, param2=0):
        """發送PGC夾爪指令"""
        if not self.modbus_client or not self.modbus_client.connected:
            self.append_log("Modbus連接未建立")
            return False
            
        try:
            # PGC夾爪指令寄存器基地址 520-529
            command_id = int(time.time() * 1000) % 65535  # 生成指令ID
            
            # 使用PyModbus 3.x語法寫入指令寄存器
            result = self.modbus_client.write_register(address=520, value=cmd, slave=1)
            if result.isError():
                self.append_log(f"寫入指令代碼失敗: {result}")
                return False
                
            result = self.modbus_client.write_register(address=521, value=param1, slave=1)
            if result.isError():
                self.append_log(f"寫入參數1失敗: {result}")
                return False
                
            result = self.modbus_client.write_register(address=522, value=param2, slave=1)
            if result.isError():
                self.append_log(f"寫入參數2失敗: {result}")
                return False
                
            result = self.modbus_client.write_register(address=523, value=command_id, slave=1)
            if result.isError():
                self.append_log(f"寫入指令ID失敗: {result}")
                return False
            
            self.append_log(f"PGC夾爪指令已發送: cmd={cmd}, param1={param1}, ID={command_id}")
            return True
            
        except Exception as e:
            self.append_log(f"PGC夾爪指令發送失敗: {str(e)}")
            return False
    
    def gripper_initialize(self):
        """PGC夾爪初始化"""
        self.send_gripper_command(1)  # 指令1: 初始化
        
    def gripper_open(self):
        """PGC夾爪快速開啟"""
        self.send_gripper_command(7)  # 指令7: 快速開啟
        
    def gripper_close(self):
        """PGC夾爪快速關閉"""
        self.send_gripper_command(8)  # 指令8: 快速關閉
        
    def gripper_stop(self):
        """PGC夾爪停止"""
        self.send_gripper_command(2)  # 指令2: 停止
        
    def gripper_move_to_pos(self):
        """PGC夾爪移動到指定位置"""
        pos = self.gripper_pos_spin.value()
        self.send_gripper_command(3, pos)  # 指令3: 絕對位置
        
    def gripper_set_force(self):
        """PGC夾爪設定力道"""
        force = self.gripper_force_spin.value()
        self.send_gripper_command(5, force)  # 指令5: 設定力道
    
    def sync_current_position(self):
        """將機械臂當前位置同步到輸入框"""
        if not self.global_state['connect']:
            QMessageBox.warning(self, "警告", "請先連接機械臂")
            return
            
        # 檢查是否有有效的位置數據
        if (self.current_position['cartesian']['x'] == 0 and 
            self.current_position['cartesian']['y'] == 0 and 
            self.current_position['cartesian']['z'] == 0):
            QMessageBox.warning(self, "警告", "尚未獲取到機械臂位置反饋，請稍後再試")
            return
            
        # 同步笛卡爾座標到輸入框
        cart = self.current_position['cartesian']
        self.x_spin.setValue(cart['x'])
        self.y_spin.setValue(cart['y'])
        self.z_spin.setValue(cart['z'])
        self.r_spin.setValue(cart['r'])
        
        # 同步關節角度到輸入框
        joint = self.current_position['joint']
        self.j1_spin.setValue(joint['j1'])
        self.j2_spin.setValue(joint['j2'])
        self.j3_spin.setValue(joint['j3'])
        self.j4_spin.setValue(joint['j4'])
        
    def update_gripper_status(self):
        """更新PGC夾爪狀態顯示"""
        if not self.modbus_client or not self.modbus_client.connected:
            return
            
        try:
            # 使用PyModbus 3.x語法讀取PGC夾爪狀態寄存器 500-519
            result = self.modbus_client.read_holding_registers(address=500, count=20, slave=1)
            if result.isError():
                self.gripper_status_label.setText("讀取錯誤")
                self.gripper_status_label.setStyleSheet("color: red")
                return
                
            registers = result.registers
            
            # 解析狀態
            module_status = registers[0]  # 模組狀態
            connection_status = registers[1]  # 連接狀態
            hold_status = registers[4]  # 夾持狀態
            current_pos = registers[5]  # 當前位置
            
            # 更新顯示
            if connection_status == 1:
                self.gripper_status_label.setText("已連接")
                self.gripper_status_label.setStyleSheet("color: green")
            else:
                self.gripper_status_label.setText("未連接")
                self.gripper_status_label.setStyleSheet("color: red")
                
            # 夾持狀態映射
            hold_status_map = {0: "運動中", 1: "到達", 2: "夾住", 3: "掉落"}
            self.gripper_hold_label.setText(hold_status_map.get(hold_status, "未知"))
            
            self.gripper_pos_label.setText(str(current_pos))
            
        except Exception as e:
            self.gripper_status_label.setText("通訊錯誤")
            self.gripper_status_label.setStyleSheet("color: red")
            # 不要在這裡記錄日誌，避免頻繁錯誤訊息
    
    # 點位管理功能
    def save_current_point(self):
        """保存當前點位"""
        if not self.global_state['connect']:
            QMessageBox.warning(self, "警告", "請先連接機械臂")
            return
            
        # 檢查是否有有效的位置數據
        if (self.current_position['cartesian']['x'] == 0 and 
            self.current_position['cartesian']['y'] == 0 and 
            self.current_position['cartesian']['z'] == 0):
            QMessageBox.warning(self, "警告", "尚未獲取到機械臂位置反饋，請稍後再試")
            return
            
        name, ok = QInputDialog.getText(self, "保存點位", "輸入點位名稱:")
        if not ok or not name.strip():
            return
            
        # 使用實際機械臂反饋的位置數據
        cartesian = self.current_position['cartesian'].copy()
        joint = self.current_position['joint'].copy()
        
        point = {
            'id': len(self.saved_points),
            'name': name.strip(),
            'cartesian': cartesian,
            'joint': joint,
            'created_time': datetime.now().isoformat(),
            'modified_time': datetime.now().isoformat()
        }
        
        self.saved_points.append(point)
        self.save_points()
        self.refresh_points_list()
        
        # 更詳細的日誌信息
        self.append_log(f"點位 '{name}' 已保存 - 位置: X:{cartesian['x']:.2f}, Y:{cartesian['y']:.2f}, Z:{cartesian['z']:.2f}, R:{cartesian['r']:.2f}")
        self.append_log(f"關節角度: J1:{joint['j1']:.2f}, J2:{joint['j2']:.2f}, J3:{joint['j3']:.2f}, J4:{joint['j4']:.2f}")
        
    def edit_selected_point(self):
        """編輯選中的點位"""
        current_row = self.points_list.currentRow()
        if current_row < 0 or current_row >= len(self.saved_points):
            QMessageBox.warning(self, "警告", "請先選擇要編輯的點位")
            return
            
        point = self.saved_points[current_row]
        dialog = PointEditDialog(point, self)
        
        if dialog.exec_() == QDialog.Accepted:
            new_data = dialog.get_data()
            
            # 驗證位置變化
            if not dialog.validate_position_change(point, new_data):
                reply = QMessageBox.question(self, "安全警告", 
                    "檢測到大幅度位置變化或正負號改變，這可能導致機械臂碰撞。\n是否繼續保存？",
                    QMessageBox.Yes | QMessageBox.No)
                if reply == QMessageBox.No:
                    return
                    
            self.saved_points[current_row] = new_data
            self.save_points()
            self.refresh_points_list()
            self.append_log(f"點位 '{new_data['name']}' 已更新")
            
    def delete_selected_point(self):
        """刪除選中的點位"""
        current_row = self.points_list.currentRow()
        if current_row < 0 or current_row >= len(self.saved_points):
            QMessageBox.warning(self, "警告", "請先選擇要刪除的點位")
            return
            
        point = self.saved_points[current_row]
        reply = QMessageBox.question(self, "確認刪除", 
            f"確定要刪除點位 '{point['name']}' 嗎？",
            QMessageBox.Yes | QMessageBox.No)
            
        if reply == QMessageBox.Yes:
            del self.saved_points[current_row]
            # 重新分配ID
            for i, p in enumerate(self.saved_points):
                p['id'] = i
            self.save_points()
            self.refresh_points_list()
            self.append_log(f"點位 '{point['name']}' 已刪除")
            
    def move_to_selected_point_with_dialog(self):
        """雙擊點位列表時彈出運動類型選擇對話框"""
        current_row = self.points_list.currentRow()
        if current_row < 0 or current_row >= len(self.saved_points):
            return
        
        # 彈出運動類型選擇對話框
        motion_type, ok = QInputDialog.getItem(self, "選擇運動類型", 
            "請選擇運動方式:", 
            [
                "直線運動(MovL) - 可能遇到手勢切換問題",
                "關節運動(MovJ) - 避免手勢切換問題", 
                "關節座標運動(JointMovJ) - 使用儲存關節角度"
            ], 
            1, False)  # 預設選擇MovJ
        
        if ok:
            self.execute_point_movement(motion_type)
    
    def move_to_selected_point(self, use_preset=False):
        """移動到選中的點位"""
        current_row = self.points_list.currentRow()
        if current_row < 0 or current_row >= len(self.saved_points):
            QMessageBox.warning(self, "警告", "請先選擇要移動的點位")
            return
            
        if not self.global_state['connect']:
            QMessageBox.warning(self, "警告", "請先連接機械臂")
            return
            
        if not self.global_state['enable']:
            QMessageBox.warning(self, "警告", "請先使能機械臂")
            return
        
        if use_preset:
            # 使用下拉選單預設的運動類型
            motion_type = self.motion_type_combo.currentText()
            self.execute_point_movement(motion_type)
        else:
            # 彈出選擇對話框
            motion_type, ok = QInputDialog.getItem(self, "選擇運動類型", 
                "請選擇運動方式:", 
                [
                    "直線運動(MovL) - 可能遇到手勢切換問題",
                    "關節運動(MovJ) - 避免手勢切換問題", 
                    "關節座標運動(JointMovJ) - 使用儲存關節角度"
                ], 
                1, False)  # 預設選擇MovJ
            
            if ok:
                self.execute_point_movement(motion_type)
    
    def execute_point_movement(self, motion_type):
        """執行點位移動"""
        current_row = self.points_list.currentRow()
        point = self.saved_points[current_row]
        cartesian = point['cartesian']
        joint = point['joint']
        speed = self.point_speed_slider.value()
        
        # 檢查點位數據完整性
        required_keys = ['x', 'y', 'z', 'r']
        for key in required_keys:
            if key not in cartesian:
                QMessageBox.critical(self, "錯誤", f"點位數據不完整，缺少{key}座標")
                return
        
        # 安全範圍檢查
        x, y, z, r = cartesian['x'], cartesian['y'], cartesian['z'], cartesian['r']
        
        # M1Pro安全工作範圍檢查
        if not (-800 <= x <= 800):
            QMessageBox.warning(self, "安全警告", f"X座標 {x} 可能超出安全範圍(-800~800)")
        if not (-800 <= y <= 800):
            QMessageBox.warning(self, "安全警告", f"Y座標 {y} 可能超出安全範圍(-800~800)")
        if not (50 <= z <= 600):
            reply = QMessageBox.question(self, "安全警告", 
                f"Z座標 {z} 可能不安全(建議50~600)，是否繼續？",
                QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.No:
                return
        
        try:
            # 記錄詳細信息用於除錯
            self.append_log(f"準備移動到點位 '{point['name']}'")
            
            if "MovL" in motion_type:
                self.append_log(f"使用MovL - 目標座標: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, R={r:.2f}")
                self.append_log(f"移動速度: {speed}%")
                speed_param = f"SpeedL={speed}"
                result = self.client_move.MovL(x, y, z, r, speed_param)
                
            elif "MovJ" in motion_type and "Joint" not in motion_type:
                self.append_log(f"使用MovJ - 目標座標: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, R={r:.2f}")
                result = self.client_move.MovJ(x, y, z, r)
                
            elif "JointMovJ" in motion_type:
                # 檢查關節數據完整性
                required_joint_keys = ['j1', 'j2', 'j3', 'j4']
                for key in required_joint_keys:
                    if key not in joint:
                        QMessageBox.critical(self, "錯誤", f"關節數據不完整，缺少{key}角度")
                        return
                
                j1, j2, j3, j4 = joint['j1'], joint['j2'], joint['j3'], joint['j4']
                self.append_log(f"使用JointMovJ - 關節角度: J1={j1:.2f}, J2={j2:.2f}, J3={j3:.2f}, J4={j4:.2f}")
                result = self.client_move.JointMovJ(j1, j2, j3, j4)
            
            self.append_log(f"運動指令已發送，回應: {result}")
            
        except Exception as e:
            error_msg = f"移動失敗: {str(e)}"
            QMessageBox.critical(self, "移動錯誤", error_msg)
            self.append_log(error_msg)
            self.append_log(f"點位數據: {cartesian}")
    def toggle_enable(self):
        """切換使能狀態"""
        if self.global_state['enable']:
            self.client_dash.DisableRobot()
            self.enable_btn.setText("使能")
            self.global_state['enable'] = False
            self.append_log("機械臂已下使能")
        else:
            self.client_dash.EnableRobot()
            self.enable_btn.setText("下使能")
            self.global_state['enable'] = True
            self.append_log("機械臂已使能")
            
    def load_points(self):
        """載入點位數據"""
        try:
            if os.path.exists(self.points_file):
                with open(self.points_file, 'r', encoding='utf-8') as f:
                    self.saved_points = json.load(f)
                self.refresh_points_list()
                self.append_log(f"載入 {len(self.saved_points)} 個點位")
        except Exception as e:
            self.append_log(f"載入點位失敗: {str(e)}")
            self.saved_points = []
            
    def save_points(self):
        """保存點位數據"""
        try:
            with open(self.points_file, 'w', encoding='utf-8') as f:
                json.dump(self.saved_points, f, ensure_ascii=False, indent=2)
        except Exception as e:
            self.append_log(f"保存點位失敗: {str(e)}")
            
    def refresh_points_list(self):
        """刷新點位列表顯示"""
        self.points_list.clear()
        for point in self.saved_points:
            item_text = f"[{point['id']}] {point['name']} - " \
                       f"X:{point['cartesian']['x']:.2f} " \
                       f"Y:{point['cartesian']['y']:.2f} " \
                       f"Z:{point['cartesian']['z']:.2f}"
            self.points_list.addItem(item_text)
    
    def start_feedback_thread(self):
        """啟動反饋線程"""
        if not self.global_state['connect']:
            self.append_log("錯誤：機械臂未連接，無法啟動反饋線程")
            return
        
        self.feedback_thread = Thread(target=self.feedback_loop, daemon=True)
        self.feedback_thread.start()
        self.append_log("狀態反饋線程已啟動")
        
    def feedback_loop(self):
        """反饋循環"""
        hasRead = 0
        while self.global_state['connect']:
            try:
                data = bytes()
                while hasRead < 1440:
                    temp = self.client_feed.socket_dobot.recv(1440 - hasRead)
                    if len(temp) > 0:
                        hasRead += len(temp)
                        data += temp
                hasRead = 0
                
                a = np.frombuffer(data, dtype=MyType)
                if hex((a['test_value'][0])) == '0x123456789abcdef':
                    feedback_data = {
                        'speed_scaling': a["speed_scaling"][0],
                        'robot_mode': a["robot_mode"][0],
                        'digital_input_bits': a["digital_input_bits"][0],
                        'digital_outputs': a["digital_outputs"][0],
                        'q_actual': a["q_actual"][0],
                        'tool_vector_actual': a["tool_vector_actual"][0]
                    }
                    self.signals.feedback_update.emit(feedback_data)
                    
                    # 檢查錯誤
                    if a["robot_mode"] == 9:
                        self.handle_robot_error()
                        
                # 更新PGC夾爪狀態
                self.update_gripper_status()
                        
                time.sleep(0.1)  # 100ms更新間隔
                
            except Exception as e:
                if self.global_state['connect']:
                    self.signals.log_update.emit(f"反饋線程錯誤: {str(e)}")
                break
                
    def handle_robot_error(self):
        """處理機械臂錯誤"""
        try:
            error_info = self.client_dash.GetErrorID()
            # 解析錯誤代碼
            self.parse_and_display_error(error_info)
        except Exception as e:
            self.signals.error_update.emit(f"獲取錯誤信息失敗: {str(e)}")
    
    def parse_and_display_error(self, error_response):
        """解析並顯示錯誤信息"""
        try:
            # 解析錯誤回應格式
            if "{" in error_response:
                error_data = error_response.split("{")[1].split("}")[0]
                error_list = json.loads("{" + error_data + "}")
                
                # 錯誤代碼映射
                error_messages = {
                    22: "手勢切換錯誤 - 請嘗試使用關節運動(JointMovJ)或調整目標點位",
                    23: "直線運動過程中規劃點超出工作空間 - 重新選取運動點位",
                    24: "圓弧運動過程中規劃點超出工作空間 - 重新選取運動點位",
                    32: "運動過程逆解算奇異 - 重新選取運動點位",
                    33: "運動過程逆解算無解 - 重新選取運動點位",
                    34: "運動過程逆解算限位 - 重新選取運動點位"
                }
                
                # 檢查控制器錯誤
                if len(error_list) > 0 and error_list[0]:
                    for error_id in error_list[0]:
                        if error_id in error_messages:
                            self.signals.error_update.emit(f"錯誤 {error_id}: {error_messages[error_id]}")
                        else:
                            self.signals.error_update.emit(f"未知錯誤 {error_id}")
            else:
                self.signals.error_update.emit(f"機械臂錯誤: {error_response}")
                
        except Exception as e:
            self.signals.error_update.emit(f"錯誤解析失敗: {str(e)}")
            self.signals.error_update.emit(f"原始錯誤: {error_response}")
            
    @pyqtSlot(dict)
    def update_feedback_display(self, data):
        """更新反饋顯示"""
        # 更新速度比例
        self.current_speed_label.setText(f"{data['speed_scaling']}%")
        
        # 更新機械臂模式
        mode_text = LABEL_ROBOT_MODE.get(data['robot_mode'], "未知")
        self.robot_mode_label.setText(mode_text)
        
        # 更新位置信息並保存到實際位置變量
        pos = data['tool_vector_actual']
        self.position_label.setText(f"{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}, {pos[3]:.2f}")
        
        # 保存實際位置數據供點位記錄使用
        self.current_position['cartesian'] = {
            'x': float(pos[0]),
            'y': float(pos[1]),
            'z': float(pos[2]),
            'r': float(pos[3])
        }
        
        # 更新關節角度並保存到實際位置變量
        joints = data['q_actual']
        self.joint_label.setText(f"{joints[0]:.2f}, {joints[1]:.2f}, {joints[2]:.2f}, {joints[3]:.2f}")
        
        # 保存實際關節角度數據
        self.current_position['joint'] = {
            'j1': float(joints[0]),
            'j2': float(joints[1]),
            'j3': float(joints[2]),
            'j4': float(joints[3])
        }
        
        # 更新數位IO
        di_bits = bin(data['digital_input_bits'])[2:].rjust(64, '0')
        do_bits = bin(data['digital_outputs'])[2:].rjust(64, '0')
        self.di_label.setText(di_bits)
        self.do_label.setText(do_bits)
        
    @pyqtSlot(str)
    def append_log(self, message):
        """添加日誌"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        
    @pyqtSlot(str)
    def append_error(self, message):
        """添加錯誤信息"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.error_text.append(f"[{timestamp}] {message}")
        
    def clear_error_info(self):
        """清除錯誤信息"""
        self.error_text.clear()
        
    def closeEvent(self, event):
        """關閉事件"""
        if self.global_state['connect']:
            self.disconnect_robot()
        event.accept()

<<<<<<< HEAD
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RobotUI()
    window.show()
    sys.exit(app.exec_())
=======

def main():
    """主函數"""
    # 檢查依賴
    try:
        import customtkinter
        import pymodbus
    except ImportError as e:
        print(f"缺少依賴模組: {e}")
        print("請安裝: pip install customtkinter pymodbus")
        return
        
    # 創建並運行應用
    app = DobotM1Visualizer()
    app.run()


if __name__ == "__main__":
    main()
>>>>>>> a88a68be167a8a36cbaa4c083fac3ec5dad9dd1e
