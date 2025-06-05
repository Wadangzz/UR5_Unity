import numpy as np
import Robot
import socket
import json
import time
import MyRobotMath as math
from MyRobotMath import SE3
import matplotlib.pyplot as plt

HOST = '127.0.0.1'
PORT = 5000


se3 = SE3()
ur5 = Robot.UR5()

M = ur5.zero

desired = [440,-431,575,-45,131,102]
# desired = [350,-250,400,0,0,45]
# theta_0 = [0,0,0,0,0,0]
# initpos = M[:3,3].tolist()
# init = [initpos[0],initpos[1],initpos[2],0,0,0]
init = [674.0001, 500.0001, 399.9999, 12.20001, 36.20602, 36.20602]
theta_0 = [-46.04139, -88.33513, 41.61081, 11.45547, 90.85027, 14.39871] 


X_s = math.task_trajectory(init,desired,samples=200)
print(X_s[-1])
task_trajectory = []


for i in range(len(X_s)):
    x0, y0 ,z0 = X_s[i][:3,3].flatten()
    roll0, pitch0, yaw0 = se3.CurrenntAngles(X_s[i])
    pos_d = [x0, y0, z0, roll0, pitch0, yaw0]
    theta = math.IK(ur5,theta_0,pos_d)
    task_trajectory.append(theta)
    theta_0 = theta


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    print(f" Unity connected on {HOST}:{PORT}")

    for angles in task_trajectory:
        # JSON 직렬화
        data = json.dumps(angles).encode('utf-8')
        s.sendall(data + b'\n')  # 한 줄 단위로 구분
        time.sleep(0.001) 


# task_trajectory = np.array(task_trajectory).T
# times = np.linspace(0, 1.0, 500)

# plt.figure(figsize=(10, 6))


# for i,joint in enumerate(['J1','J2','J3','J4','J5','J6']):
#     plt.plot(times, task_trajectory[i], label=joint)

# plt.xlabel("Time (s)")
# plt.ylabel("Joint Value")
# plt.title("SCARA Joint Trajectory")
# plt.grid(True)
# plt.legend()
# plt.tight_layout()
# # plt.show()