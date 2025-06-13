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
B = ur5.B_tw
L = len(ur5.joints)

init = None
initpos = None
impos_task = False
coodinate = ['x','y','z','RX','RY','RZ']

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
                # initpos = data['position'] + data['rotation']
            except json.JSONDecodeError:
                print("[WARN] Invalid JSON")

def trajectory(init,initpos,desired,N):

    impos_task = False
    X_s = math.task_trajectory(ur5,initpos,desired,samples=N)
    task_trajectory = []
    current = init

    for i in range(len(X_s)):
        x0, y0 ,z0 = X_s[i][:3,3].flatten()
        roll0, pitch0, yaw0 = se3.CurrenntAngles(X_s[i])
        pos_d = [x0, y0, z0, roll0, pitch0, yaw0]
        theta,count = math.IK(ur5,current,pos_d)
        # task trajectoy 발산할 때 or elbow down 자세일 때 joint trajectory로 전환
        if count == 50 or np.abs(theta[1]) > 90: 
            impos_task = True
            task_trajectory.clear()
            break

        task_trajectory.append(theta)
        current = theta
        
    if impos_task:
        
        end,_ = math.IK(ur5,init,desired)

        if np.abs(end[1]) > 90: # elbow down 자세 바닥에 밖히는거 방지
        
            phi = sum(end[1:4])
            k_1 = ur5.links[2]+ur5.links[4]*np.cos(np.deg2rad(end[2]))
            k_2 = -ur5.links[4]*np.sin(np.deg2rad(end[2]))
            
            end[2] = -end[2]
            end[1] = end[1] - np.rad2deg(2*np.arctan2(k_2,k_1))
            end[3] = phi-(end[1]+end[2])

        start = np.array(init)
        end = np.array(end)

        d_theta, task_trajectory = math.joint_trajectory(start,end,samples=2*N)

    return task_trajectory
    
if __name__ == "__main__":

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        print(f" Unity connected on {HOST}:{PORT}")

        angle_thread = threading.Thread(target=receive_loop,args=(s,))
        angle_thread.start()

        while init is None:
            print("[INFO] Waiting for joint angle data from Unity...")
            time.sleep(0.1)

        while True:

            print(initpos)
            print(init)
            desired = []
            try:
                for i, comp in enumerate(coodinate):
                    desired.append(float(input(f"{comp} : ")))
            except Exception as e:
                print(f'입력오류 : {e}')
                continue

            X_s = math.task_trajectory(ur5,initpos,desired,samples=100)
            task_trajectory = []
            current = init

            for i in range(len(X_s)):
                x0, y0 ,z0 = X_s[i][:3,3].flatten()
                roll0, pitch0, yaw0 = se3.CurrenntAngles(X_s[i])
                pos_d = [x0, y0, z0, roll0, pitch0, yaw0]
                theta,count = math.IK(ur5,current,pos_d)
                # task trajectoy 발산할 때 or elbow down 자세일 때 joint trajectory로 전환
                if count == 50 or np.abs(theta[1]) > 90: 
                    impos_task = True
                    task_trajectory.clear()
                    break

                task_trajectory.append(theta)
                current = theta
                
            if impos_task:
            
                print('관절 공간 경로')
                end,_ = math.IK(ur5,init,desired)

                if np.abs(end[1]) > 90: # elbow down 자세 바닥에 밖히는거 방지
                
                    phi = sum(end[1:4])
                    k_1 = ur5.links[2]+ur5.links[4]*np.cos(np.deg2rad(end[2]))
                    k_2 = -ur5.links[4]*np.sin(np.deg2rad(end[2]))
                    
                    end[2] = -end[2]
                    end[1] = end[1] - np.rad2deg(2*np.arctan2(k_2,k_1))
                    end[3] = phi-(end[1]+end[2])

                start = np.array(init)
                end = np.array(end)

                d_theta, task_trajectory = math.joint_trajectory(start,end,samples=200)

            # visual.plot_trajectory(start,d_theta,L)

            for angles in task_trajectory:
                # JSON 직렬화
                data = json.dumps(angles).encode('utf-8')
                s.sendall(data + b'\n')  # 한 줄 단위로 구분
                time.sleep(0.001) 