import rospy
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 全局变量用于存储加速度数据
accel_x_data = []
accel_y_data = []
accel_z_data = []

# IMU消息的回调函数
def imu_callback(data):
    accel_x_data.append(data.linear_acceleration.x)
    accel_y_data.append(data.linear_acceleration.y)
    accel_z_data.append(data.linear_acceleration.z)
    # 限制列表大小
    accel_x_data = accel_x_data[-50:]
    accel_y_data = accel_y_data[-50:]
    accel_z_data = accel_z_data[-50:]

# 初始化ROS节点
rospy.init_node('imu_visualizer', anonymous=True)

# 订阅IMU数据主题
rospy.Subscriber("/imu/data_raw", Imu, imu_callback)

# 设置matplotlib图表
fig, ax = plt.subplots()
ax.set_xlim(0, 50)
ax.set_ylim(-20, 20) # 根据你的IMU数据范围调整
line_x, = ax.plot([], [], label='X Acceleration')
line_y, = ax.plot([], [], label='Y Acceleration')
line_z, = ax.plot([], [], label='Z Acceleration')

# 更新图表的函数
def update(frame):
    line_x.set_data(list(range(len(accel_x_data))), accel_x_data)
    line_y.set_data(list(range(len(accel_y_data))), accel_y_data)
    line_z.set_data(list(range(len(accel_z_data))), accel_z_data)
    return line_x, line_y, line_z

# 创建动画
ani = FuncAnimation(fig, update, frames=range(100), blit=True)

plt.legend()
plt.show()

# ROS节点循环
rospy.spin()
