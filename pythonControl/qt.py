import sys
import json
import socket
import time
import threading
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QWidget, QSlider, QLabel, QVBoxLayout, QApplication, 
    QHBoxLayout, QComboBox, QPushButton, QTableWidget, 
    QTableWidgetItem, QGridLayout)


class DragUI(QWidget):
    def __init__(self,socket):
        super().__init__()
        self.socket = socket
        self.setWindowTitle("UR5 Position Controller")
        self.resize(400, 300)

        self.sliders = {}
        self.labels = {}

        # self.scale = {
        #     'X': 0.01, 'Y': 0.01, 'Z': 0.01,
        #     'RX': 0.01, 'RY': 0.01, 'RZ': 0.01,
        #     'J1': 0.01, 'J2': 0.01, 'J3': 0.01,
        #     'J4': 0.01, 'J5': 0.01, 'J6': 0.01,
        # }

        # 전체 레이아웃: 좌우 나누기
        main_layout = QHBoxLayout()
        left_layout = QVBoxLayout()   # X, Y, Z, RX, RY, RZ
        right_layout = QVBoxLayout()  # J1 ~ J6

        # 왼쪽: 위치 및 자세 슬라이더
        for axis in ['X', 'Y', 'Z', 'RX', 'RY', 'RZ']:
            hlayout = QHBoxLayout()
            slider = QSlider(Qt.Horizontal)
            slider.setFixedWidth(100)
            slider.setMinimum(int(-500.0 / 0.01) if axis in ['X', 'Y', 'Z'] else int(-180.0 / 0.01))
            slider.setMaximum(int(500.0 / 0.01) if axis in ['X', 'Y', 'Z'] else int(180.0 / 0.01))
            slider.setValue(0)
            slider.setSingleStep(1)
            slider.valueChanged.connect(self.on_pose_changed)

            label = QLabel(f"{axis}: 0.00")
            label.setFixedWidth(60)
            label.setAlignment(Qt.AlignRight)

            hlayout.addWidget(label)
            hlayout.addWidget(slider)
            left_layout.addLayout(hlayout)

            self.sliders[axis] = slider
            self.labels[axis] = label

        # 오른쪽: 조인트 각도 슬라이더
        for i in range(6):
            name = f"J{i+1}"
            hlayout = QHBoxLayout()
            slider = QSlider(Qt.Horizontal)
            slider.setFixedWidth(100)
            slider.setMinimum(int(-180.0 / 0.01))
            slider.setMaximum(int(180.0 / 0.01))
            slider.setValue(0)
            slider.setSingleStep(1)
            slider.valueChanged.connect(self.on_angle_changed)

            label = QLabel(f"{name}: 0.00")
            label.setFixedWidth(60)
            label.setAlignment(Qt.AlignRight)

            hlayout.addWidget(label)
            hlayout.addWidget(slider)
            right_layout.addLayout(hlayout)

            self.sliders[name] = slider
            self.labels[name] = label

        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)
        self.setLayout(main_layout)

    def on_pose_changed(self):
        pose = []
        for axis in ['X', 'Y', 'Z', 'RX', 'RY', 'RZ']:
            val = self.sliders[axis].value() * 0.01
            self.labels[axis].setText(f"{axis}: {val}")
            pose.append(val)

        print("→ 드래그 위치:", pose)
        # TODO: math.IK + trajectory + socket.send(pose)

    def on_angle_changed(self):
        angle = []
        for axis in ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']:
            val = self.sliders[axis].value() * 0.01
            self.labels[axis].setText(f"{axis}: {val}")
            angle.append(val)
        
            # JSON 직렬화
        data = json.dumps(angle).encode('utf-8')
        self.socket.sendall(data + b'\n')  # 한 줄 단위로 구분
        time.sleep(0.001) 

        print(f"Joint : {angle}")

if __name__ == '__main__':
    
    HOST = '127.0.0.1'
    PORT = 5000

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        print(f" Unity connected on {HOST}:{PORT}")
    
        angle_thread = threading.Thread(target=receive_loop,args=(s,))
        angle_thread.start()

        app = QApplication(sys.argv)
        window = DragUI(s)
        window.show()
        sys.exit(app.exec_())
