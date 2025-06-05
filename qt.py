from PyQt5.QtWidgets import QWidget, QSlider, QLabel, QVBoxLayout, QApplication, QHBoxLayout
from PyQt5.QtCore import Qt
import sys

class DragUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("UR5 Position Controller")
        self.resize(400, 300)

        self.sliders = {}
        self.labels = {}

        # 전체 레이아웃: 좌우 나누기
        main_layout = QHBoxLayout()
        left_layout = QVBoxLayout()   # X, Y, Z, RX, RY, RZ
        right_layout = QVBoxLayout()  # J1 ~ J6

        # 왼쪽: 위치 및 자세 슬라이더
        for axis in ['X', 'Y', 'Z', 'RX', 'RY', 'RZ']:
            hlayout = QHBoxLayout()
            label = QLabel(f"{axis}: 0")
            slider = QSlider(Qt.Horizontal)
            slider.setFixedWidth(100)
            slider.setMinimum(-500 if axis in ['X', 'Y', 'Z'] else -180)
            slider.setMaximum(500 if axis in ['X', 'Y', 'Z'] else 180)
            slider.setValue(0)
            slider.setSingleStep(1)
            slider.valueChanged.connect(self.on_value_changed)

            label = QLabel(f"{axis}: 0")
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
            slider.setMinimum(-180)
            slider.setMaximum(180)
            slider.setValue(0)
            slider.setSingleStep(1)
            # slider.valueChanged.connect(...) ← 향후 제어용

            label = QLabel(f"{name}: 0")
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

    def on_value_changed(self):
        pose = []
        for axis in ['X', 'Y', 'Z', 'RX', 'RY', 'RZ']:
            val = self.sliders[axis].value()
            self.labels[axis].setText(f"{axis}: {val}")
            pose.append(val)

        print("→ 드래그 위치:", pose)
        # TODO: math.IK + trajectory + socket.send(pose)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = DragUI()
    window.show()
    sys.exit(app.exec_())
