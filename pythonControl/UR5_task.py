import numpy as np
import Robot
import socket
import json
import threading
import time
import MyRobotMath as math

se3 = math.SE3()

HOST = '127.0.0.1'
PORT = 5000

ur5 = Robot.UR5()
M = ur5.zero

init = None
initpos = None
coodinate = ['x','y','z','RX','RY','RZ']

def receive_loop(sock):
    global init, initpos
    with sock.makefile('r', encoding='utf-8') as f:
        for line in f:
            try:
                data = json.loads(line.strip())
                # print("[DATA]", data)
                init = data['joints']
                initpos = data['position'] + data['rotation']
            except json.JSONDecodeError:
                print("[WARN] Invalid JSON")

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    print(f" Unity connected on {HOST}:{PORT}")

    angle_thread = threading.Thread(target=receive_loop,args=(s,))
    angle_thread.start()

    while init is None:
        print("[INFO] Waiting for joint angle data from Unity...")
        time.sleep(0.1)

    while True:

        desired = []
        for i, comp in enumerate(coodinate):
            desired.append(float(input(f"{comp} : ")))

        X_s = math.task_trajectory(initpos,desired,samples=200)
        print(X_s[-1])
        task_trajectory = []

        for i in range(len(X_s)):
            x0, y0 ,z0 = X_s[i][:3,3].flatten()
            roll0, pitch0, yaw0 = se3.CurrenntAngles(X_s[i])
            pos_d = [x0, y0, z0, roll0, pitch0, yaw0]
            theta = math.IK(ur5,init,pos_d)
            task_trajectory.append(theta)
            init = theta


        # visual.plot_trajectory(start,d_theta,L)

        for angles in task_trajectory:
            # JSON 직렬화
            data = json.dumps(angles).encode('utf-8')
            s.sendall(data + b'\n')  # 한 줄 단위로 구분
            time.sleep(0.001) 

        