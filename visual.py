import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from MyRobotMath import quintic_time_scaling

def plot_trajectory(theta_start, d_theta, length, times=1.0, samples=400):

    T = times
    N = samples
    time = np.linspace(0, T, N)

    trajectory = []
    velocity = []
    acceleration = []

    for i in range(N):
        t = i / N * T
        s, s_dot, s_ddot = quintic_time_scaling(t, T)
        theta_desired = theta_start + s*(d_theta)
        theta_dot = s_dot*(d_theta)
        theta_ddot = s_ddot*(d_theta)
        trajectory.append(theta_desired)
        velocity.append(theta_dot)
        acceleration.append(theta_ddot)
    
    trajectory = np.array(trajectory).T  # shape = (4, N)
    velocity = np.array(velocity).T
    acceleration = np.array(acceleration).T

    fig, axs = plt.subplots(1, 3, figsize=(15, 4))
    fig.suptitle("Joint Trajectory", fontsize=16)

    joint_names = []
    for i in range(length):
        joint_names.append(f"J{i+1}")

    # Position (Trajectory)
    for i in range(length):
        axs[0].plot(time, trajectory[i], label=joint_names[i])

    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel("J1,J2,J4 (deg) / J3 (cm)")
    axs[0].set_title("Joint Position Profile")
    axs[0].legend()
    axs[0].grid(True)

    # Velocity
    for i in range(length):
        axs[1].plot(time, velocity[i], label=joint_names[i])

    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel("J1,J2,J4 (deg/s) / J3 (cm/s)") 
    axs[1].set_title("Joint Velocity Profile")
    axs[1].legend()
    axs[1].grid(True)

    # Acceleration
    for i in range(length):
        axs[2].plot(time, acceleration[i], label=joint_names[i])
    axs[2].set_xlabel("Time (s)")

    axs[2].set_ylabel("J1,J2,J4 (deg/s²) / J3 (cm/s²)")
    axs[2].set_title("Joint Acceleration Profile")
    axs[2].legend()
    axs[2].grid(True)

    plt.tight_layout()
    plt.show()

def animate_robot(robot,trajectory,lim = 180,times=1.0,samples=500):

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    ax.view_init(elev=20, azim=-65)
    
    trajectory_points = []
    ee_path_line = ax.plot([], [], [], 'r--', linewidth=1.0)[0]

    lines = [ax.plot([0, 0], [0, 0], [0, 0], link.color + '-', linewidth=2)[0] for link in robot.links]
    joints = [ax.scatter(0, 0, 0, color='red', s=10) for _ in robot.links]
    ee = ax.scatter(0, 0, 0, color='green', s=10)
    ee_text = ax.text(0, 0, 0, "", fontsize=8, color='black')

    def update(frame):
        # 현재 프레임의 각도로 로봇 업데이트
        robot.update_angles(trajectory[frame])
        for i, link in enumerate(robot.links):
            lines[i].set_data([link.start_point[0], link.end_point[0]],
                            [link.start_point[1], link.end_point[1]])
            lines[i].set_3d_properties([link.start_point[2], link.end_point[2]])
            joints[i]._offsets3d = ([link.start_point[0]], [link.start_point[1]], [link.start_point[2]])

        end_effector = robot.links[4].end_point
        ee._offsets3d = ([end_effector[0]], [end_effector[1]], [end_effector[2]])
        ee_text.set_position((end_effector[0], end_effector[1]))
        ee_text.set_3d_properties(end_effector[2])
        ee_text.set_text(f"{end_effector.round(1)}")

        trajectory_points.append(end_effector.copy())

        if len(trajectory_points) > 1:
            path = np.array(trajectory_points)
            ee_path_line.set_data(path[:, 0], path[:, 1])
            ee_path_line.set_3d_properties(path[:, 2])

        ax.set_xlim([-lim, lim])
        ax.set_ylim([-lim, lim])
        ax.set_zlim([0, 2*lim])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f"Time: {frame/samples*times:.2f} s")

    # 애니메이션 생성 및 실행
    ani = FuncAnimation(fig, update, frames=samples, interval = times*1000/samples, repeat=False)
    ani.save('animation.gif', writer = 'pillow', fps = 30)
    plt.show()