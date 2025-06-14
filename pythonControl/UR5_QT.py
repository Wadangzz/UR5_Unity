import sys, socket, json, time, sqlite3
import trajectorysqlite as ts
import Robot
import threading
import UR5_task
import MyRobotMath as math
from MyRobotMath import SE3
from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidgetItem, QMessageBox
from MyQT import Ui_MainWindow

class MyApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.lock = threading.Lock()
        self.sock = None
        self.connected = False
        self.receive_thread = None
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
            self.axis_labels[axis] = getattr(self.ui, f'label_{i+1}') # label_0 ~ label_6

        self.ui.slider_Speed.setValue(5)

        self.ui.pb_Conn.clicked.connect(self.connect_to_unity)
        self.ui.pB_Disconn.clicked.connect(self.disconnect_from_unity)
        self.ui.comboBox_Robot.currentIndexChanged.connect(self.on_program_changed) # 프로그램 번호 변경 드롭다운
        self.ui.pB_Queuepos.clicked.connect(self.queue_current_pose) # 자세 저장 버튼
        self.ui.pB_Start.clicked.connect(self.run_trajectory_program) # 프로그램 실행 버튼
        self.ui.slider_Speed.valueChanged.connect(self.speed_level_changed)
        self.connect_joint_sliders()       
        self.connect_axis_sliders()


    def connect_to_unity(self):
        if self.connected:
            QMessageBox.information(self, "Alarm", "이미 연결되어 있습니다.")
            return
        
        ip = self.ui.lineEdit_IP.text()
        port = self.ui.lineEdit_PORT.text()
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((ip, int(port)))
            self.connected = True
            self.receive_thread = threading.Thread(target=self.receive_loop, daemon=True)
            self.receive_thread.start()
            QMessageBox.information(self, "연결 성공", f"Unity에 연결되었습니다: {ip}:{port}")
            print(f"Unity connected to {ip}:{port}")
        except Exception as e:
            QMessageBox.critical(self, "연결 실패", f"연결 중 오류 발생: {e}")
            self.sock = None
            self.connected = False

    def disconnect_from_unity(self):
        if self.connected and self.sock:
            try:
                self.sock.close()
                QMessageBox.information(self, "연결 해제", "Unity와의 연결이 해제되었습니다.")
                print("Unity 연결 해제됨")
            except Exception as e:
                QMessageBox.warning(self, "해제 실패", f"해제 중 오류 발생: {e}")
        else:
            QMessageBox.information(self, "알림", "이미 연결 해제 상태입니다.")
            print("이미 연결 해제 상태입니다.")
        self.sock = None
        self.connected = False


    # 속도 레벨에 따라 샘플 수 조정 (최대 5000)
    # 샘플 수가 적을 수록 속도가 빨라짐
    def get_speed_sample_count(self):
        level = self.ui.slider_Speed.value()
        return min(int(1000 / level), 1000) 
    
    # 프로그램 ID에 해당하는 테이블 업데이트
    def update_table(self, program_id):
        # pose_list = self.programs_queue[program_id]
        pose_list = ts.load_poses_from_db(program_id)
        self.ui.tableWidget.setRowCount(len(pose_list))
        for row, pose in enumerate(pose_list):
            for col, val in enumerate(pose):
                item = QTableWidgetItem(f"{val:.2f}")
                self.ui.tableWidget.setItem(row, col, item)

    # 프로그램 변경 시 테이블 업데이트
    def on_program_changed(self, idx):
        program_id = self.ui.comboBox_Robot.currentText()
        self.update_table(program_id)

    # 현재 자세를 프로그램에 저장
    def queue_current_pose(self):
        global initpos
        if initpos is None:
            print("아직 initpos 수신 전입니다.")
            return QMessageBox.warning(self, "Error", "연결되지 않았습니다.")

        program_id = self.ui.comboBox_Robot.currentText()
        ts.save_pose_to_db(program_id, initpos)
        # self.programs_queue[program_id].append(initpos.copy())
        print(f"프로그램 {program_id} 위치 저장됨:", initpos)
        self.update_table(program_id)

    # 현재 프로그램의 자세를 실행하는 함수
    # def run_trajectory_program(self):
    #     global init, initpos
    #     if init is None or initpos is None:
    #         print("init/initpos 없음. Unity 데이터 수신 전입니다.")
    #         return QMessageBox.warning(self, "Error", "연결되지 않았습니다.")

    #     program_id = self.ui.comboBox_Robot.currentText()
    #     pose_list = self.programs_queue.get(program_id, [])

    #     if not pose_list:
    #         print(f"프로그램 {program_id}에 저장된 pose가 없습니다.")
    #         return QMessageBox.warning(self, "Error", f"프로그램 {program_id}에 저장된 pose가 없습니다.")

    #     print(f"프로그램 {program_id} 실행 시작 (총 {len(pose_list)}개)")

    #     try:
    #         send_thread = threading.Thread(target=self.calc_send,args=(init,initpos,pose_list),daemon=True)
    #         send_thread.start()
    #     except Exception as e:
    #         QMessageBox.critical(self, "Error", f"프로그램 실행 중 오류 발생: {e}")

    def run_trajectory_program(self):
        global init, initpos
        if init is None or initpos is None:
            return QMessageBox.warning(self, "Error", "Unity 데이터가 수신되지 않았습니다.")

        program_id = self.ui.comboBox_Robot.currentText()
        row_count = self.ui.tableWidget.rowCount()
        if row_count == 0:
            return QMessageBox.warning(self, "Error", "테이블에 저장된 pose가 없습니다.")

        pose_list = []
        for row in range(row_count):
            pose = []
            for col in range(7):  # X, Y, Z, quaternion
                item = self.ui.tableWidget.item(row, col)
                if item is None:
                    pose.append(0.0)
                else:
                    try:
                        pose.append(float(item.text()))
                    except ValueError:
                        pose.append(0.0)  # or handle error
            pose_list.append(pose)

        print(f"프로그램 {program_id} 실행 시작 (총 {len(pose_list)}개)")
        try:
            send_thread = threading.Thread(target=self.calc_send,args=(init,initpos,pose_list),daemon=True)
            send_thread.start()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"프로그램 실행 중 오류 발생: {e}")
    
    def calc_send(self,init,initpos,pose_list):

        current_joint = init
        current_pose = initpos

        for i, target_pose in enumerate(pose_list):
            print(f"▶ {i+1}/{len(pose_list)} → {target_pose}")
            n = self.get_speed_sample_count()
            trajectory = UR5_task.trajectory(current_joint, current_pose, target_pose, n)

            interval = 0.005  # 5ms
            next_time = time.perf_counter()

            for joint in trajectory:
                now = time.perf_counter()
                if now < next_time:
                    time.sleep(next_time - now)
                data = json.dumps(joint).encode('utf-8')
                try:
                    with self.lock:
                        self.sock.sendall(data + b'\n')
                except Exception as e:
                    print(f"[전송 실패] {e}")

                next_time += interval
            
            # 업데이트 상태를 다음 단계 기준으로 갱신
            current_joint = trajectory[-1]
            matexps_b = [se3.matexp(j, B[i], joint=ur5.joints[i].type) for i, j in enumerate(current_joint)]
            T = se3.matFK(M, matexps_b)
            current_pose = T[:3, 3].tolist() + se3.CurrentQuaternion(T)[0]

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
        x, y, z = T[:3,3].flatten()
        _, euler = se3.CurrentQuaternion(T)
        pos = [x, y, z] + euler
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
        pos = []
        try:
            for name in self.axis_sliders:
                val = self.axis_sliders[name].value() * 0.01
                self.axis_labels[name].setText(f"{val:.2f}")
                pos.append(val)

            desired = se3.euler_to_quat(pos)
            pose,_ = math.IK(ur5,init,desired)

            for i, name in enumerate(self.joint_sliders):
                self.joint_sliders[name].blockSignals(True)
                self.joint_sliders[name].setValue(int(pose[i] / 0.01))
                self.joint_sliders[name].blockSignals(False)
                self.joint_labels[name].setText(f"{pose[i]:.2f}")
            data = json.dumps(pose).encode('utf-8')
            
        except Exception as e:
            print(f"[오류] 자세 계산 실패: {e}")
            return
        try:
            self.sock.sendall(data + b'\n')
            print("자세 전송:", pose)
        except Exception as e:
            print("[소켓 전송 실패]", e)
    
    def speed_level_changed(self):
        level = self.ui.slider_Speed.value()
        self.ui.label_0.setText(f"{level}")
        print(f"속도 레벨 변경됨: {level}")


    def receive_loop(self):
        global init, initpos
        with self.sock.makefile('r', encoding='utf-8') as f:
            for line in f:
                try:
                    data = json.loads(line.strip())
                    # print("[DATA]", data)
                    init = data['joints']
                    matexps_b = [se3.matexp(init[i], B[i], joint=ur5.joints[i].type) for i in range(L)]
                    # T_init = se3.matFK(M,matexps_b)
                    initpos = data['position'] + data['rotation']
                except json.JSONDecodeError:
                    # JSON 파싱 실패 시 경고 출력
                    print("[WARN] Invalid JSON received from Unity")

if __name__ == '__main__':

    se3 = SE3()
    ur5 = Robot.UR5()

    M = ur5.zero
    B = ur5.B_tw
    L = len(ur5.joints)

    init = None
    initpos = None

    app = QApplication(sys.argv)
    window = MyApp()
    window.show()
    sys.exit(app.exec_())