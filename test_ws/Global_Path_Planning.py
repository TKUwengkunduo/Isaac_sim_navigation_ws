import numpy as np
import matplotlib.pyplot as plt

def simulate_trajectory(x, y, theta, v, omega, dt, predict_time):
    x_traj, y_traj = [x], [y]
    for _ in np.arange(0, predict_time, dt):
        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt
        theta += omega * dt
        x_traj.append(x)
        y_traj.append(y)
    return x_traj, y_traj, theta

def evaluate_trajectory(x_traj, y_traj, goal, obstacles):
    dist_to_goal = np.sqrt((x_traj[-1] - goal[0])**2 + (y_traj[-1] - goal[1])**2)
    min_dist_to_obstacle = min([np.sqrt((x - ox)**2 + (y - oy)**2) for x, y in zip(x_traj, y_traj) for ox, oy in obstacles], default=float('inf'))
    if min_dist_to_obstacle < 1.5:  # 假设障碍物安全距离为1米
        return float('inf')
    return dist_to_goal

def dynamic_window_approach(x, y, theta, v, omega, goal, obstacles, robot_params):
    # 設定模擬的時間間隔，這決定了每一步模擬的時間長度
    dt = 0.1
    # 設定預測時間，這是對未來軌跡進行模擬的時間範圍
    predict_time = 0.8
    # 從機器人的速度範圍內產生一系列可能的速度值供後續計算使用
    v_samples = np.linspace(robot_params['min_speed'], robot_params['max_speed'], 20)
    # 從機器人的旋轉速度範圍內產生一系列可能的旋轉速度值供後續計算使用
    omega_samples = np.linspace(-robot_params['max_rotation_speed'], robot_params['max_rotation_speed'], 40)

    # 初始化最佳得分為無窮大，用於後續尋找最小得分（最佳軌跡）
    best_score = float('inf')
    # 初始化最佳速度和旋轉速度為當前速度和旋轉速度
    best_v, best_omega = v, omega
    # 遍歷所有可能的速度和旋轉速度組合
    for v_sample in v_samples:
        for omega_sample in omega_samples:
            # 使用模擬函數計算給定速度和旋轉速度下的預測軌跡
            x_traj, y_traj, _ = simulate_trajectory(x, y, theta, v_sample, omega_sample, dt, predict_time)
            # 評估該軌跡，得到一個得分，評估標準包括到達目標的距離和避開障礙物
            score = evaluate_trajectory(x_traj, y_traj, goal, obstacles)
            # 如果該軌跡的得分更低（更優），則更新最佳速度和旋轉速度
            if score < best_score:
                best_score = score
                best_v, best_omega = v_sample, omega_sample

    # 返回計算出的最佳速度和旋轉速度
    return best_v, best_omega


# 地图和全局路径规划
map_size = (50, 50)
obstacles = [(2, 0), (10, 9), (20, 16), (25, 27), (30, 29)]  # 障碍物位置
goal = (40, 40)  # 目标位置
global_path = [(i, i) for i in range(41)]  # 简化的全局路径

# 机器人参数
robot_params = {'min_speed': 0, 'max_speed': 2, 'max_rotation_speed': np.pi/4, 'wheel_base': 0.5}
x, y, theta = 0, 0, 0  # 初始位置和姿态
v, omega = 0, 0  # 初始速度和转向速度

# 模拟机器人移动
local_path_x, local_path_y = [x], [y]
for _ in range(600):  # 模拟600个时间步
    v, omega = dynamic_window_approach(x, y, theta, v, omega, goal, obstacles, robot_params)
    x, y, theta = simulate_trajectory(x, y, theta, v, omega, 0.1, 0.1)[0][-1], simulate_trajectory(x, y, theta, v, omega, 0.1, 0.1)[1][-1], theta + omega * 0.1
    local_path_x.append(x)
    local_path_y.append(y)
    if np.sqrt((x - goal[0])**2 + (y - goal[1])**2) < 1:  # 到达目标
        break

# 绘图
plt.figure(figsize=(10, 10))
plt.plot(*zip(*global_path), label="Global Path")
plt.scatter(*zip(*obstacles), color='red', label="Obstacles")
plt.plot(local_path_x, local_path_y, label="Local Path")
plt.scatter(*goal, color='green', label="Goal")
plt.xlim(0, map_size[0])
plt.ylim(0, map_size[1])
plt.legend()
plt.grid(True)
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('DWA Local Path Planning')
plt.show()
