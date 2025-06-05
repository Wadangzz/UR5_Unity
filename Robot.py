import numpy as np
from dataclasses import dataclass
from MyRobotMath import SE3
from scipy.spatial.transform import Rotation as R


se3 = SE3()

# def select_init(desired, preset): # 목표 좌표에 해당하는 사분면 초기 상태 호출
#     x, y = desired[0], desired[1]
#     key = (x > 0, y > 0)
#     mapping = {
#         (True,   True): 0,
#         (False,  True): 1,
#         (False, False): 2,
#         (True,  False): 3,
#     }
#     return preset[mapping[key]]

@dataclass
class JointSpec:
    type: str  # 'R' or 'P'
    axis: str  # 'x', 'y', 'z'
    link : str # 'x', 'y', 'z'

class RobotLink:

    def __init__(self, length, joint, angle=np.zeros(3), color = 'b', parent=None):
        self.length = length
        self.joint_type = joint.type
        self.joint_axis = joint.axis # 'R' for revolute, 'P' for prismatic
        self.joint_link = joint.link  
        self.angle = angle
        self.start_point = np.array([0, 0, 0])
        self.end_point = np.array([0, 0, 0])
        self.color = color
        self.parent = parent
        
    def update_position(self, start_point, angle=np.zeros(3), prismatic=0):
        self.start_point = start_point
        self.angle = angle
        yaw, pitch, roll = self.angle
        robot_type = type(self.parent).__name__ if self.parent else 'Unknown'
        
        if self.joint_type == 'R':
            # axis_map = {
            #     'x': [0, 0, self.angle],
            #     'y': [0, self.angle, 0],
            #     'z': [self.angle, 0, 0],
            # }
            link_map = {
                'x': np.array([self.length, 0, 0]),
                'y': np.array([0, self.length, 0]),
                'z': np.array([0, 0, self.length])
            }
            # euler = axis_map.get(self.joint_axis, [0, 0, 0])
            direction = link_map.get(self.joint_link, np.array([0, 0, 0]))
            if robot_type == 'SCARA':
                rotation_matrix = R.from_euler('zyx',[yaw,pitch,roll]).as_matrix()
            elif robot_type == 'BarretWAM':
                rotation_matrix = R.from_euler('xyz',[roll,pitch,yaw]).as_matrix()
            self.end_point = self.start_point + rotation_matrix @ direction

        else:
            link_map = {
                'x': np.array([prismatic, 0, 0]),
                'y': np.array([0, prismatic, 0]),
                'z': np.array([0, 0, prismatic])
            }
            linear = link_map.get(self.joint_link, [0,0,0])
            self.end_point = self.start_point + linear

class SCARA:
    # SCARA 로봇의 Zero position, Body Twist, Space Twist, 초기값 추정 set
    def __init__(self, L1, L2, L3):
        
        self.joints = [
            JointSpec('R', 'z', 'x'),
            JointSpec('R', 'z', 'x'),
            JointSpec('P', 'z', 'z'),
            JointSpec('R', 'z', 'x'),
        ]

        # SCARA 로봇 링크 생성
        self.links = [
            RobotLink(L1, JointSpec(None, 'z', 'z'), parent=self),  # L1: J1 회전 축 관절
            RobotLink(L2, self.joints[0], color='c', parent=self),  # L2: J1 관절에 의해 회전
            RobotLink(20, JointSpec(None, 'z', 'z'), parent=self),  # J2 회전 축 관절
            RobotLink(L3, self.joints[1], color='m', parent=self),  # L3: J2 관절에 의해 회전
            RobotLink(0, self.joints[2], color='y', parent=self),  # L4: J3 직선 관절
            RobotLink(10, self.joints[3], color='r', parent=self),
            RobotLink(10, JointSpec('R', 'z', 'y'), color='g', parent=self)   # End-effector: J4 회전 관절
        ]
        
        self.zero = np.array([[1, 0, 0, L2+L3],
                              [0, 1, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])

        self.B_tw = [[0, 0, 1 ,0, L2+L3 ,0],
                     [0, 0, 1, 0, L2, 0],
                     [0, 0, 0, 0, 0, 1],
                     [0, 0, 1, 0, 0, 0]]
        
        self.S_tw = (se3.adjoint(self.zero) @ np.array(self.B_tw).T).T.tolist()

    def update_angles(self, angles):
        current_point = np.array([0, 0, 0])
        
        # L1 (중앙 Z축)
        self.links[0].update_position(current_point, prismatic=self.links[0].length)
        current_point = self.links[0].end_point
        
        # L2 (J1 회전 관절)
        self.links[1].update_position(current_point, angle=np.deg2rad([angles[0],0,0]))
        current_point = self.links[1].end_point

        # J2 회전 축
        self.links[2].update_position(current_point, prismatic=self.links[2].length)
        current_point = self.links[2].end_point
        
        # L3 (J2 회전 관절)
        self.links[3].update_position(current_point, angle=np.deg2rad([angles[0]+angles[1],0,0]))
        current_point = self.links[3].end_point
        
        # L4 (J3 직선 관절)
        self.links[4].update_position(current_point, prismatic=-self.links[0].length-self.links[2].length+angles[2])
        current_point = self.links[4].end_point
        
        # End-effector (J4 회전 관절)
        self.links[5].update_position(current_point, angle=np.deg2rad([angles[0]+angles[1]+angles[3],0,0]))
        self.links[6].update_position(current_point, angle=np.deg2rad([angles[0]+angles[1]+angles[3],0,0]))

class BarretWAM:

    def __init__(self,L1,L2,L3,W):
        
        self.joints = [
            JointSpec('R', 'z', 'z'),
            JointSpec('R', 'y', 'z'),
            JointSpec('R', 'z', 'z'),
            JointSpec('R', 'y', 'z'),
            JointSpec('R', 'z', 'z'),
            JointSpec('R', 'y', 'z'),
            JointSpec('R', 'z', 'z')
        ]

        self.links = [
            RobotLink(0, self.joints[0], color='c', parent=self),
            RobotLink(L1, self.joints[1], color='c', parent=self), # L1 링크
            RobotLink(0, self.joints[2], color='c', parent=self),  
            RobotLink(W, JointSpec('R','y','x'), color='c', parent=self), 
            RobotLink(-W, JointSpec('R','y','x'), color='y', parent=self),
            RobotLink(L2, self.joints[3], color='y', parent=self),    
            RobotLink(0, self.joints[4], color='k', parent=self),
            RobotLink(L3, self.joints[5], color='k', parent=self), 
            RobotLink(0, self.joints[6], color='g', parent=self),
            RobotLink(4, JointSpec('R', 'z', 'x'), color='r', parent=self),
            RobotLink(4, JointSpec('R', 'z', 'y'), color='b', parent=self),
            RobotLink(4, JointSpec('R', 'z', 'z'), color='g', parent=self),
            RobotLink(4, JointSpec('R', 'z', 'x'), color='r', parent=self), # J1 회전 시각화
            RobotLink(4, JointSpec('R', 'z', 'x'), color='b', parent=self) # J3 회전 시각화
        ]
        self.zero = np.array([[1, 0, 0, 0],
                              [0, 1, 0, 0],
                              [0, 0, 1, L1+L2+L3],
                              [0, 0, 0, 1]])
        
        self.B_tw = [[0, 0, 1 ,0, 0 ,0],
                     [0, 1, 0, L1+L2+L3, 0, 0],
                     [0, 0, 1, 0, 0, 0],
                     [0, 1, 0, L2+L3, 0, W],
                     [0, 0, 1, 0, 0, 0],
                     [0, 1, 0, L3, 0, 0],
                     [0, 0, 1, 0, 0, 0]]

        self.S_tw = (se3.adjoint(self.zero) @ np.array(self.B_tw).T).T.tolist()

    def update_angles(self, angles):
        current_point = np.array([0, 0, 0])
        
        self.links[12].update_position(current_point, angle=np.deg2rad([angles[0],0,0]))
        self.links[13].update_position(current_point, angle=np.deg2rad([angles[2],0,0]))

        self.links[1].update_position(current_point, angle=np.deg2rad([angles[0]+angles[2],
                                                                       angles[1],0])) # L1 링크 회전
        current_point = self.links[1].end_point
        
        self.links[3].update_position(current_point, angle=np.deg2rad([angles[0]+angles[2],
                                                                       angles[1],
                                                                       0])) # L1 + W 링크 회전
        current_point = self.links[3].end_point

        self.links[4].update_position(current_point, angle=np.deg2rad([angles[0]+angles[2],
                                                                       angles[1]+angles[3],
                                                                       0])) # -W 링크 회전
        current_point = self.links[4].end_point

        self.links[5].update_position(current_point, angle=np.deg2rad([angles[0]+angles[2],
                                                                       angles[1]+angles[3],
                                                                       0])) # -W + L2 링크 회전
        current_point = self.links[5].end_point


        self.links[7].update_position(current_point, angle=np.deg2rad([angles[0]+angles[2]+angles[4]+angles[6],
                                                                       angles[1]+angles[3]+angles[5], 
                                                                       0])) # L3 링크 회전
        current_point = self.links[7].end_point
        
        # End-Effector 표시

        self.links[9].update_position(current_point, angle=np.deg2rad([angles[0]+angles[2]+angles[4]+angles[6],
                                                                       angles[1]+angles[3]+angles[5],
                                                                       0]))
        self.links[10].update_position(current_point, angle=np.deg2rad([angles[0]+angles[2]+angles[4]+angles[6],
                                                                       angles[1]+angles[3]+angles[5],
                                                                       0]))
        self.links[11].update_position(current_point, angle=np.deg2rad([angles[0]+angles[2]+angles[4]+angles[6],
                                                                       angles[1]+angles[3]+angles[5],
                                                                       0]))
class UR5:

    def __init__(self,L1=89,L2=135.85,L3=425,L4=119.7,L5=392.25,L6=93,L7=94.65,L8=82.3):
        
        self.joints = [
            JointSpec('R', 'z', 'z'),
            JointSpec('R', 'x', 'z'),
            JointSpec('R', 'x', 'z'),
            JointSpec('R', 'x', 'z'),
            JointSpec('R', 'z', 'z'),
            JointSpec('R', 'x', 'z'),
        ]
        self.links = [L1,L2,L3,L4,L5,L6,L7,L8]

        self.zero = np.array([[1, 0, 0, L2-L4+L6+L8],
                              [0, 1, 0, 0],
                              [0, 0, 1, L1+L3+L5+L7],
                              [0, 0, 0, 1]])
        
        self.B_tw = [[0, 0, 1, 0,(L2-L4+L6+L8),0],
                     [1, 0, 0, 0, -(L3+L5+L7), 0],
                     [1, 0, 0, 0, -(L5+L7), 0],
                     [1, 0, 0, 0, -L7, 0],
                     [0, 0, 1, 0, L8, 0],
                     [1, 0, 0, 0, 0, 0]]

        self.S_tw = (se3.adjoint(self.zero) @ np.array(self.B_tw).T).T.tolist()