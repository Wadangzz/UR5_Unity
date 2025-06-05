import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

class SE3:

    def skew(self, v):
        """
        Create a skew-symmetric matrix from a 3D vector.
        :param v: 3D vector (list)
        :return: 3x3 skew-symmetric matrix
        """
        return np.array([[0, -v[2], v[1]],
                        [v[2], 0, -v[0]],
                        [-v[1], v[0], 0]])
    
    def adjoint(self, T):
        """
        Compute the adjoint representation of a transformation matrix.
        :param T: 4x4 transformation matrix
        :return: Adjoint representation (6x6 matrix)
        """
        R = T[:3, :3]
        p = T[:3, 3]
        
        adj = np.zeros((6, 6))
        adj[:3, :3] = R
        adj[3:6, 3:6] = R
        adj[3:6, :3] = self.skew(p) @ R
        
        return adj
     
    def matexp(self, degree, screw, joint='R',unit='degree'): # Matrix exponential
        """
        Compute the matrix exponential of a se(3) element.
        :param degree : Angle of rotation
        :param screw axis 6D vector representing the se(3) element
        :return: 4x4 transformation matrix
        """
        if unit == 'degree':
            if joint == 'R': # Revolute Joint 일 때
                theta = np.deg2rad(degree)
            elif joint == 'P': # Prismatic Joint 일 때
                theta = degree
        elif unit == 'radian':
            if joint == 'R': # Revolute Joint 일 때
                theta = degree
            elif joint == 'P': # Prismatic Joint 일 때
                theta = np.rad2deg(degree)

        omega = screw[:3]
        v = screw[3:6]
        
        omega_hat = self.skew(omega)
        omega_hat_sq = np.dot(omega_hat, omega_hat)
        
        R = np.eye(3) + np.sin(theta)*omega_hat + (1 - np.cos(theta))*omega_hat_sq
        G_theta = np.eye(3)*theta + (1 - np.cos(theta))*omega_hat + (theta - np.sin(theta))*omega_hat_sq
        p = G_theta @ v
        
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = p
        
        return T
    
    def matFK(self,m,matexps):
        """
        Compute the forword kinematics transformation matrix (body axis).
        :param m : Initial transformation matrix (4x4)
        :param matexps : List of body transformation matrices (4x4)
        :return: 4x4 transformation matrix
        """
        T_sb = m
        for i in range(len(matexps)):
            T_sb = np.dot(T_sb, matexps[i]) # Forward kinematics to get the end-effector transformation matrix

        return T_sb
    
    def space_jacobian(self, matexps, v):
        """
        Compute the space Jacobian for a series of transformations.
        :param matexps: List of space transformation matrices (4x4)
        :param v: List of space twists (6D vectors)
        :return: Space Jacobian (6xN matrix)
        """
        j_s = np.zeros((6, len(matexps)))
        j_s[:, 0] = np.array(v[0]).reshape(6,)
        mul = np.eye(4)
        for i in range(1,len(matexps)):
            mul = np.dot(mul, matexps[i-1])
            j_s[:, i] = np.dot(self.adjoint(mul), np.array(v[i]).reshape(6,))
        
        return j_s

    def body_jacobian(self, m, matexps_b, matexps_s, v):
        """
        Compute the body Jacobian for a series of transformations.
        :param m: Initial transformation matrix (4x4)
        :param matexps_b: List of body transformation matrices (4x4)
        :param matexps_s: List of space transformation matrices (4x4)
        :param v: List of space twists (6D vectors)
        :return: Body Jacobian (6xN matrix)
        """
        T_sb = self.matFK(m,matexps_b)
        T_bs = np.linalg.inv(T_sb) # Inverse of the transformation matrix
        adj_bs = self.adjoint(T_bs)

        j_b = adj_bs @ self.space_jacobian(matexps_s, v)

        return j_b

    
    def pose_to_SE3(self, desired, degree = True):
        """
        Compute the desired transformation matrix.
        :param derised: List of disired position (x, y, z, roll, pitch, yaw)
        :param degree: Unit of axis (True = Degree, False = Radian)
        :return: Desired transformation matrix (4x4 matrix)
        """
        x, y, z = desired[0], desired[1], desired[2]
        j1 ,j2 ,j3 = desired[3], desired[4], desired[5]

        if degree:
            roll, pitch, yaw = np.deg2rad([j1, j2, j3])
        else:
            roll, pitch, yaw = j1, j2, j3
        
        rotmat = R.from_euler('xyz',[roll,pitch,yaw]).as_matrix()
       
        T = np.eye(4)
        T[:3,:3] = rotmat
        T[:3, 3] = [float(x), float(y), float(z)]

        return T
    
    def j_inv(self,j_b):
        """
        Compute pseudoinverse of the jacobian matrix.
        :param j_b : Body Jacobian (6xN matrix)
        :return : Pseudoinverse of the jacobian matrix.
        """
        # return np.linalg.inv(j_b.T @ j_b) @ j_b.T # Moore-Penrose 의사역행렬 (tall)
        return np.linalg.pinv(j_b) # 특이값 분해 기반 의사역행렬, square이면 그냥 역행렬
    
    # def matlogm(self, T):
    #     """
    #     Compute the relative twist 
    #     from the desired transformation matrix using matrix logarithm.
    #     :param T: Desired transformation matrix (4x4)
    #     :return: 6D vector, radian
    #     """
    #     S = np.zeros(6)
    #     R = T[:3,:3]
    #     p = T[:3,3]
    #     trace = np.trace(R)
    #     cos_theta = np.clip((trace-1)/2,-1.0,1.0) # 부동소수점으로 범위 초과하는거 방지
    #     theta_bd = np.acos(cos_theta)
    #     if abs(theta_bd) < 1e-6:
    #         theta_bd += np.random.normal(0,0.01) # theta_bd가 0이 되면 Nan 에러 발생, 가우시안 노이즈 추가
    #     omega_bd_hat = (1 / (2 * np.sin(theta_bd)) * (R-R.T))
    #     omega_bd_hat_sq = omega_bd_hat**2
    #     omega_bd = [omega_bd_hat[2][1],omega_bd_hat[0][2],omega_bd_hat[1][0]]
    #     v_bd = (np.eye(3)/theta_bd - 0.5*omega_bd_hat+(1/theta_bd-0.5/np.tan(0.5*theta_bd))*omega_bd_hat_sq) @ p
    #     S[:3] = omega_bd
    #     S[3:] = v_bd
    #     T_bd = theta_bd * S
    #     return T_bd, S, theta_bd
    
    def matlogm(self, T_bd):
        """
        Compute the relative twist from the desired transformation matrix using matrix logarithm.
        :param T_bd: 4x4 Transformation matrix
        :return: (Twist * θ), Twist vector (6D), θ (radian)
        """
        S = np.zeros(6)
        R_bd = T_bd[:3, :3]
        p = T_bd[:3, 3]
        trace = np.trace(R_bd)
        cos_theta = np.clip((trace - 1) / 2, -1.0, 1.0)
        theta = np.acos(cos_theta)

        epsilon = 1e-6
        if np.abs(theta) < epsilon:
        #     omega_hat = np.zeros((3, 3))
        #     v = p / np.linalg.norm(p) if np.linalg.norm(p) > epsilon else np.zeros(3)
        #     screw = np.concatenate([np.zeros(3), v])
        #     return theta * screw, screw, theta
            theta += np.random.normal(0,0.01) # theta_bd가 0이 되면 Nan 에러 발생, 가우시안 노이즈 추가

        omega_hat = (1 / (2 * np.sin(theta))) * (R_bd - R_bd.T)
        omega_hat_sq = omega_hat @ omega_hat
        omega = np.array([
            omega_hat[2,1],
            omega_hat[0,2],
            omega_hat[1,0]
        ])
        
        A_inv = (
            np.eye(3) / theta
            - 0.5 * omega_hat
            + (1 / theta - 0.5 / np.tan(theta / 2)) * omega_hat_sq
        )
        v = A_inv @ p
        screw = np.concatenate([omega, v])
        return theta * screw, screw, theta
    
    def CurrenntAngles(self, T_sb):
        """
        Compute the current Euler angle
        :param T_sb : Current forword kinematics transformation matrix (body axis)
        :return : List of Euler angle roll,pitch,yaw (Degree)
        """
        R_mat = T_sb[:3, :3]
        euler = R.from_matrix(R_mat).as_euler('zyx', degrees=True)  # yaw, pitch, roll
        return euler[::-1].tolist()  # roll, pitch, yaw 순으로 리턴
    
        # R_31 = T_sb[2,0]

        # if R_31**2 != 1:
        #     roll = np.atan2(T_sb[2,1],T_sb[2,2])
        #     pitch = np.atan2(-1*T_sb[2,0],np.sqrt(T_sb[0,0]**2+T_sb[1,0]**2))
        #     yaw = np.atan2(T_sb[1,0],T_sb[0][0])
        
        # else:
        #     yaw = 0
        #     pitch = -R_31 * np.pi/2 
        #     roll = -R_31 * np.atan2(T_sb[0,1],T_sb[1,1])
        
        # return np.rad2deg([roll, pitch, yaw]).tolist() // 이거는 너무 정확도가 떨어짐......

def deg2rad(value,joint): 
    """
    If revolute joint, convert degree to radian. 
    :param value: List of joint angle (degree)
    :param joint: List of joint
    :return: Numpy.array of joint angle (radian)
    """
    theta = np.array(value)
    for i in range(len(joint)):
        if joint[i].type == 'R':
            theta[i] = np.deg2rad(theta[i])
    
    return theta

def rad2deg(value,joint):
    """
    If revolute joint, convert redian to degree. 
    :param value: List of joint angle (radian)
    :param joint: List of joint
    :return: Numpy.array of joint angle (degree)
    """
    theta = np.array(value)
    for i in range(len(joint)):
        if joint[i].type == 'R':
           theta[i] = np.rad2deg(value[i])

    return theta

def theta_normalize(value,joint):
    """
    If revolute joint, normalize degree -180 to 180. 
    :param value: Numpy.Array of joint angle (degree)
    :param joint: List of joint
    :return: List of normalized joint angle 
    """
    theta = value.flatten().tolist()
    for i in range(len(value)):
        if joint[i].type == 'R':
            if value[i] % 360 > 180:
                theta[i] = theta[i] % 360 - 360
            else:
                theta[i] = theta[i] % 360
    
    return theta


def quintic_time_scaling(t, T):
    """
    Calculates time-based scaling functions using a 5th order polynomial.
    
    Parameters:
        t (float): Current time
        T (float): Total motion time
    
    Returns:
        tuple: (s, s_dot, s_ddot)
            - s: Position scaling function (ranges from 0 to 1)
            - s_dot: Velocity scaling function
            - s_ddot: Acceleration scaling function
    
    Note:
        This function uses a 5th order polynomial to generate smooth motion.
        At start point (t=0): position=0, velocity=0, acceleration=0
        At end point (t=T): position=1, velocity=0, acceleration=0
    """
    quintic = np.array([[T**3  ,   T**4 ,   T**5],
                        [3*T**2,  4*T**3, 5*T**4],
                        [6*T   , 12*T**2,20*T**3]])
    
    s_T = np.array([1, 0, 0]) # position 1, velocity 0, acceleration 0 at t = T

    cofficient = np.linalg.inv(quintic) @ s_T.reshape(3,1)
    a3, a4 ,a5 = cofficient.flatten()

    s = a3*(t)**3 + a4*(t)**4 + a5*(t)**5 # a0, a1, a2 are 0, so not included
    s_dot = (3*a3*(t)**2 + 4*a4*(t)**3 + 5*a5*(t)**4)
    s_ddot = (6*a3*(t) + 12*a4*(t)**2 + 20*a5*(t)**3)
    return s, s_dot, s_ddot

def IK(robot, init, desired):
    """
    Improved IK that avoids gimbal lock by comparing full SE(3) matrices.
    :param robot: Robot class (joint list, screw axes, zero config)
    :param init: initial joint angles (degree)
    :param desired: desired pose (x, y, z, roll, pitch, yaw)
    :return: solution joint angles (degree)
    """
    se3 = SE3()
    M = robot.zero
    B = robot.B_tw
    S = robot.S_tw
    L = len(robot.joints)

    T_d = se3.pose_to_SE3(desired)  # Desired SE(3)
    threshold = 1e-4
    count = 0

    while True:
        count += 1

        # Forward Kinematics
        matexps_b = [se3.matexp(init[i], B[i], joint=robot.joints[i].type) for i in range(L)]
        matexps_s = [se3.matexp(init[i], S[i], joint=robot.joints[i].type) for i in range(L)]
        T_sb = se3.matFK(M, matexps_b)

        # Error Transformation
        T_bd = np.linalg.inv(T_sb) @ T_d
        V_bd, _, _ = se3.matlogm(T_bd)

        # Jacobian
        J_b = se3.body_jacobian(M, matexps_b, matexps_s, S)
        J_pseudo = se3.j_inv(J_b)

        # Update joint angle (in rad)
        theta = deg2rad(init, robot.joints)
        thetak = theta.reshape(L, 1) + J_pseudo @ V_bd.reshape(6, 1)
        thetak = rad2deg(thetak, robot.joints)
        init = theta_normalize(thetak, robot.joints)

        # Check error norm (position only or full twist)
        pos_err = np.linalg.norm(T_d[:3, 3] - T_sb[:3, 3])
        rot_err = np.linalg.norm(V_bd[:3])
        if pos_err < threshold and rot_err < np.deg2rad(0.1):
            print(f"연산 횟수 : {count}, Joint Value : {init}")
            break

        if count >= 100:
            print(f"연산 종료 (Max iter). Joint Value : {init}")
            break

    return init

def interpolate_SE3_quat(T0, Td, T, N):
    """
    Interpolates between two SE(3) transformations using quaternion-based rotation interpolation (SLERP).
    
    Parameters:
        T0 (numpy.ndarray): Initial 4x4 transformation matrix
        Td (numpy.ndarray): Desired 4x4 transformation matrix
        T (float): Total motion time
        N (int): Number of interpolation points
    
    Returns:
        list: List of interpolated 4x4 transformation matrices
        
    Note:
        - Uses SLERP (Spherical Linear Interpolation) for rotation interpolation
        - Uses quintic time scaling for smooth position interpolation
        - Combines both position and orientation interpolation
    """
    pos0 = T0[:3, 3]
    posd = Td[:3, 3]

    key_times = [0, 1]
    key_rots = R.from_matrix([T0[:3, :3], Td[:3, :3]])
    slerp = Slerp(key_times, key_rots)

    trajectory = []
    for i in range(N):
        t = i / (N - 1) * T
        s, _, _ = quintic_time_scaling(t, T)
        interp_rot = slerp([s])[0]  # s in [0,1]
        interp_pos = (1 - s) * pos0 + s * posd

        T_interp = np.eye(4)
        T_interp[:3, :3] = interp_rot.as_matrix()
        T_interp[:3, 3] = interp_pos
        trajectory.append(T_interp)

    return trajectory


def task_trajectory(start, end, times=1.0, samples=100):
    """
    Generates a smooth trajectory in task space (SE(3)) between two poses.
    
    Parameters:
        start (list): Initial pose [x, y, z, roll, pitch, yaw] in degrees
        end (list): Desired pose [x, y, z, roll, pitch, yaw] in degrees
        times (float, optional): Total motion time in seconds. Defaults to 1.0
        samples (int, optional): Number of trajectory points. Defaults to 100
    
    Returns:
        list: List of 4x4 transformation matrices representing the trajectory
        
    Note:
        - Uses SE(3) interpolation for smooth motion in both position and orientation
        - Combines position and orientation interpolation using SLERP
        - Returns a sequence of transformation matrices for the entire trajectory
    """
    se3=SE3()
    theta_start = np.array(start)
    theta_end = np.array(end)

    Td = se3.pose_to_SE3(theta_end)
    print(Td)
    T0 = se3.pose_to_SE3(theta_start)
    print(T0)

    return interpolate_SE3_quat(T0, Td, times, samples)
    # _,screw,theta= se3.matlogm(np.linalg.inv(T_0) @ T_d)

    # T = times
    # N = samples

    # trajectory = []

    # for i in range(N):
    #     t = i / N * T
    #     s, _, _= quintic_time_scaling(t, T)
    #     T_s = T_0 @ se3.matexp(theta*s,screw*s,unit='radian')
    #     trajectory.append(T_s)
    
    # return trajectory#, T_d


def joint_trajectory(start, end, times=1.0, samples=100):
    """
    Generates a smooth trajectory in joint space between two joint configurations.
    
    Parameters:
        start (list): Initial joint angles in degrees
        end (list): Desired joint angles in degrees
        times (float, optional): Total motion time in seconds. Defaults to 1.0
        samples (int, optional): Number of trajectory points. Defaults to 100
    
    Returns:
        tuple: (trajectory, velocity, acceleration)
            - trajectory: List of joint angles for each time step
            - velocity: List of joint velocities for each time step
            - acceleration: List of joint accelerations for each time step
            
    Note:
        - Uses quintic time scaling for smooth motion
        - Handles joint angle wrapping (angles > 180 degrees)
        - Returns complete trajectory information including position, velocity, and acceleration
    """
    theta_start = np.array(start)
    theta_end = np.array(end)
    
    # 180도 초과하는 각도에 대해 반대 방향으로 회전하도록 수정
    for i in range(len(theta_end)):
        if np.abs(theta_end[i]) > 180:
            if theta_end[i] > 0:
                theta_end[i] = theta_end[i] - 360
            else:
                theta_end[i] = theta_end[i] + 360
    
    d_theta = theta_end - theta_start

    T = times
    N = samples

    trajectory = []
    velocity = []
    acceleration = []

    for i in range(N):
        t = i / N * T
        s, s_dot, s_ddot = quintic_time_scaling(t, T)
        theta_desired = theta_start + s*d_theta
        theta_dot = s_dot*(d_theta)
        theta_ddot = s_ddot*(d_theta)
        trajectory.append(theta_desired)
        velocity.append(theta_dot)
        acceleration.append(theta_ddot)
    
    return trajectory

# def IK(robot,init,desired):

#     se3 = SE3()
#     M = robot.zero
#     B = robot.B_tw
#     S = robot.S_tw
#     L = len(robot.joints)

#     T_d = se3.pose_to_SE3(desired)
#     threshold = 1e-6 # 오차 범위
#     count = 0
        
#     while True:

#         matexps_b = []
#         matexps_s = []

#         count += 1 # 연산 횟수 증가

#         # Forward Kinematics
#         matexps_b = [se3.matexp(init[i], B[i], joint=robot.joints[i].type) for i in range(L)]
#         matexps_s = [se3.matexp(init[i], S[i], joint=robot.joints[i].type) for i in range(L)]
#         T_sb = se3.matFK(M, matexps_b)

#         T_sb = se3.matFK(M,matexps_b) # Forward Kinematics 적용 변환행렬
#         estimated = []
#         for i in range(3):
#             estimated.append(T_sb[i,3].item()) # 현재 x, y, z
        
#         eulerAngles = se3.CurrenntAngles(T_sb)
#         for eulerAngle in eulerAngles:
#             estimated.append(eulerAngle) # Euler 각도 추정값

#         pos_err = np.array(desired[:3]) - np.array(estimated[:3]) # x, y, z 오차

#         T_bd = np.dot(np.linalg.inv(T_sb),T_d) # Relative Trasformation Matrix
#         J_b = se3.body_jacobian(M,matexps_b,matexps_s,S) # Body Jacobian
#         J_pseudo = se3.j_inv(J_b) # Jacobian의 역행렬 (또는 의사역행렬)
#         V_bd,_,_ = se3.matlogm(T_bd) # Ralative Twist, 각도 오차

#         theta = deg2rad(init,robot.joints)

#         thetak = theta.reshape(L,1) + J_pseudo @ V_bd.reshape(6,1) # Newton Raphson Method
#         thetak = rad2deg(thetak,robot.joints)

#         # 각도 정규화 후 갱신 (-180~180)
#         init = theta_normalize(thetak,robot.joints)
#         # print(init)

#         if np.all(np.abs(pos_err) < threshold): # 오차가 임계값 이내면 break
#             print(estimated)
#             print(f"연산 횟수 : {count}, Joint Value : {init}")
#             break
#         if count >= 100:
#             print(estimated)
#             print(f"연산 횟수 : {count}, Joint Value : {init}")
#             break

#     return init