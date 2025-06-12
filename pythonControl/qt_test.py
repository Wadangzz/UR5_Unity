import sys, socket, json, time
import Robot
import threading
import UR5_task
import MyRobotMath as math
from MyRobotMath import SE3
from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidgetItem
from MyQT import Ui_MainWindow

class MyApp(QMainWindow):
    def __init__(self, sock):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.sock = sock
        self.programs_queue = {str(i): [] for i in range(1, 10)}  # '1' ~ '9' 프로그램별 리스트

        # J1~J6 슬라이더와 라벨 딕셔너리로 정리
        self.joint_sliders = {
            f'J{i}': getattr(self.ui, f'slider_J{i}') for i in range(1, 7)}
        self.joint_labels = {
            f'J{i}': getattr(self.ui, f'label_{i+6}') for i in range(1, 7)}  # label_7 ~ label_12

        # X~RZ 슬라이더와 라벨 딕셔너리로 정리
        self.axis_sliders = {}
        self.axis_labels = {}
        for i, axis in enumerate(['X','Y','Z','RX','RY','RZ']):
            self.axis_sliders[axis] = getattr(self.ui, f'slider_{axis}')
            self.axis_labels[axis] = getattr(self.ui, f'label_{i+1}')

        self.ui.slider_Speed.setValue(5)

        self.ui.comboBox_Robot.currentIndexChanged.connect(self.on_program_changed)
        self.ui.pB_Queuepos.clicked.connect(self.queue_current_pose)
        self.ui.pB_Start.clicked.connect(self.run_trajectory_program)
        self.connect_joint_sliders()       
        self.connect_axis_sliders()

    def get_speed_sample_count(self):
        level = self.ui.slider_Speed.value()
        return max(10, level * 100)

    def update_table(self, program_id):
        pose_list = self.programs_queue[program_id]
        self.ui.tableWidget.setRowCount(len(pose_list))
        for row, pose in enumerate(pose_list):
            for col, val in enumerate(pose):
                item = QTableWidgetItem(f"{val:.2f}")
                self.ui.tableWidget.setItem(row, col, item)

    def on_program_changed(self, idx):
        program_id = self.ui.comboBox_Robot.currentText()
        self.update_table(program_id)

    def queue_current_pose(self):
        global initpos
        if initpos is None:
            print("아직 initpos 수신 전입니다.")
            return

        program_id = self.ui.comboBox_Robot.currentText()
        self.programs_queue[program_id].append(initpos.copy())
        print(f"프로그램 {program_id} 위치 저장됨:", initpos)
        self.update_table(program_id)

    def run_trajectory_program(self):
        global init, initpos
        if init is None or initpos is None:
            print("init/initpos 없음. Unity 데이터 수신 전입니다.")
            return

        program_id = self.ui.comboBox_Robot.currentText()
        pose_list = self.programs_queue.get(program_id, [])

        if not pose_list:
            print(f"프로그램 {program_id}에 저장된 pose가 없습니다.")
            return

        print(f"프로그램 {program_id} 실행 시작 (총 {len(pose_list)}개)")

        current_joint = init
        current_pose = initpos

        for i, target_pose in enumerate(pose_list):
            print(f"▶ {i+1}/{len(pose_list)} → {target_pose}")
            n = self.get_speed_sample_count()
            joint_trajectory = UR5_task.trajectory(current_joint, current_pose, target_pose, n)

            for joint in joint_trajectory:
                data = json.dumps(joint).encode('utf-8')
                try:
                    self.sock.sendall(data + b'\n')
                    time.sleep(0.001)
                except Exception as e:
                    print(f"[전송 실패] {e}")
                    return

            # 업데이트 상태를 다음 단계 기준으로 갱신
            current_joint = joint_trajectory[-1]
            matexps_b = [se3.matexp(j, B[i], joint=ur5.joints[i].type) for i, j in enumerate(current_joint)]
            T = se3.matFK(M, matexps_b)
            current_pose = T[:3, 3].tolist() + se3.CurrenntAngles(T)

    def connect_joint_sliders(self):
        for name, slider in self.joint_sliders.items():
            slider.valueChanged.connect(self.send_joint_values)

    def connect_axis_sliders(self):
        for name, slider in self.axis_sliders.items():
            slider.valueChanged.connect(self.send_axis_values)

    def send_joint_values(self):
        angle = []
        for name in self.joint_sliders:
            val = self.joint_sliders[name].value() * 0.01
            self.joint_labels[name].setText(f"{val:.2f}")
            angle.append(val)
        matexps_b = [se3.matexp(angle[i], B[i], joint=ur5.joints[i].type) for i in range(L)]
        T = se3.matFK(M, matexps_b)
        x, y ,z = T[:3,3].flatten()
        roll, pitch, yaw = se3.CurrenntAngles(T)
        pos = [x, y, z, roll, pitch, yaw]
        for i, name in enumerate(self.axis_sliders):
            self.axis_sliders[name].blockSignals(True)
            self.axis_sliders[name].setValue(int(pos[i] / 0.01))
            self.axis_sliders[name].blockSignals(False)
            self.axis_labels[name].setText(f"{pos[i]:.2f}")

        data = json.dumps(angle).encode('utf-8')
        try:
            self.sock.sendall(data + b'\n')
            print("조인트 전송:", angle)
        except Exception as e:
            print("[소켓 전송 실패]", e)

    def send_axis_values(self):
        desired = []
        for name in self.axis_sliders:
            val = self.axis_sliders[name].value() * 0.01
            self.axis_labels[name].setText(f"{val:.2f}")
            desired.append(val)

        pose,_ = math.IK(ur5,init,desired)  

        for i, name in enumerate(self.joint_sliders):
            self.joint_sliders[name].blockSignals(True)
            self.joint_sliders[name].setValue(int(pose[i] / 0.01))
            self.joint_sliders[name].blockSignals(False)
        data = json.dumps(pose).encode('utf-8')        
        try:
            self.sock.sendall(data + b'\n')
            print("자세 전송:", pose)
        except Exception as e:
            print("[소켓 전송 실패]", e)

def receive_loop(sock):
    global init, initpos
    with sock.makefile('r', encoding='utf-8') as f:
        for line in f:
            try:
                data = json.loads(line.strip())
                # print("[DATA]", data)
                init = data['joints']
                matexps_b = [se3.matexp(init[i], B[i], joint=ur5.joints[i].type) for i in range(L)]
                T_init = se3.matFK(M,matexps_b)
                initpos = data['position'] + se3.CurrenntAngles(T_init)
                # print(initpos)
                # initpos = data['position'] + data['rotation']
            except json.JSONDecodeError:
                print("[WARN] Invalid JSON")

if __name__ == '__main__':

    se3 = SE3()
    ur5 = Robot.UR5()

    M = ur5.zero
    B = ur5.B_tw
    L = len(ur5.joints)

    HOST = '127.0.0.1'
    PORT = 5000

    init = None
    initpos = None

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        print(f"✅ Unity connected to {HOST}:{PORT}")

        angle_thread = threading.Thread(target=receive_loop,args=(s,),daemon = True)
        angle_thread.start()

        app = QApplication(sys.argv)
        window = MyApp(s)
        window.show()
        sys.exit(app.exec_())