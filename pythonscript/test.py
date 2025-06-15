import numpy as np
import Robot
import socket
import json
import threading
import time
import MyRobotMath as math
from MyRobotMath import SE3

se3 = SE3()
ur5 = Robot.UR5()
M = ur5.zero
B = ur5.B_tw
L = len(ur5.joints)

init = [0,0,0,0,0,0]
initpos = None
coodinate = ['x','y','z','roll','pitch','yaw']

# def receive_loop(sock):
#     global init
#     with sock.makefile('r', encoding='utf-8') as f:
#         for line in f:
#             try:
#                 data = json.loads(line.strip())
#                 # print("[DATA]", data)
#                 init = data['joints']
#                 initpos = data['position'] + data['rotation']
#             except json.JSONDecodeError:
#                 print("[WARN] Invalid JSON")

# with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
#     s.connect((HOST, PORT))
#     print(f" Unity connected on {HOST}:{PORT}")

#     angle_thread = threading.Thread(target=receive_loop,args=(s,))
#     angle_thread.start()

#     while init is None:
#         print("[INFO] Waiting for joint angle data from Unity...")
#         time.sleep(0.1)

while True:

    desired = []
    for i, comp in enumerate(coodinate):
        desired.append(float(input(f"{comp} : ")))

    end = math.IK(ur5,init,desired)

    if np.abs(end[1]) > 95:
        phi = sum(end[1:4])
        k_1 = ur5.links[2]+ur5.links[4]*np.cos(np.deg2rad(end[2]))
        k_2 = -ur5.links[4]*np.sin(np.deg2rad(end[2]))
        
        end[2] = -end[2]
        end[1] = end[1] -np.rad2deg(2*np.arctan2(k_2,k_1))
        end[3] = phi-(end[1]+end[2])

    matexps_b = [se3.matexp(end[i], B[i], joint=ur5.joints[i].type) for i in range(L)]
    T_d = se3.matFK(M, matexps_b)

    print(se3.CurrenntAngles(T_d))

        # start = np.array(init)
        # end = np.array(end)

        # trajectory = math.joint_trajectory(start,end,times=1.0,samples=200)

        # visual.plot_trajectory(start,d_theta,L)

        # for angles in trajectory:
        #     # JSON 직렬화
        #     data = json.dumps(angles.tolist()).encode('utf-8')
        #     s.sendall(data + b'\n')  # 한 줄 단위로 구분
        #     time.sleep(0.001) 

        