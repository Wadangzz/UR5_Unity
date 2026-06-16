import numpy as np
from dataclasses import dataclass
from MyRobotMath import SE3


@dataclass
class JointSpec:
    type: str  # 'R' or 'P'
    axis: str  # 'x', 'y', 'z'
    link : str # 'x', 'y', 'z'


class UR5:

    def __init__(self,L1=89.2,L2=135.85,L3=425,L4=119.7,L5=392.25,L6=93,L7=94.65,L8=82.3):

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

        self.S_tw = (SE3.Adjoint(self.zero) @ np.array(self.B_tw).T).T.tolist()
