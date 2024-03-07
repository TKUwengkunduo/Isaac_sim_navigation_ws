# import rclpy
# from rclpy.node import Node
# from tf2_msgs.msg import TFMessage

# class TFListener(Node):
#     def __init__(self):
#         super().__init__('tf_listener')
#         self.subscription = self.create_subscription(
#             TFMessage,
#             '/tf',
#             self.tf_callback,
#             10)
#         self.subscription  # prevent unused variable warning

#     def tf_callback(self, msg):
#         self.get_logger().info('Received /tf message')
#         for transform in msg.transforms:
#             self.get_logger().info(
#                 f"Frame: {transform.header.frame_id} to {transform.child_frame_id}\n"
#                 f"Translation: {transform.transform.translation}\n"
#                 f"Rotation: {transform.transform.rotation}"
#             )

# def main(args=None):
#     rclpy.init(args=args)
#     tf_listener = TFListener()

#     try:
#         rclpy.spin(tf_listener)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         tf_listener.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist

# class ContinuousMoveNode(Node):
#     def __init__(self):
#         super().__init__('continuous_move')
#         self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
#         timer_period = 0.1  # seconds
#         self.timer = self.create_timer(timer_period, self.move_forward)
#         self.get_logger().info('ContinuousMoveNode has been started.')

#     def move_forward(self):
#         msg = Twist()
#         msg.linear.x = 0.5  # Change this value to adjust the speed
#         msg.angular.z = 0.0
#         self.publisher_.publish(msg)
#         self.get_logger().info('Publishing: Move Forward')

# def main(args=None):
#     rclpy.init(args=args)
#     node = ContinuousMoveNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
import math
from scipy.spatial.transform import Rotation as R

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.subscription = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_position = {"x": -1.8, "y": 7}  # 目标位置

    def tf_callback(self, msg):
        self.get_logger().info('Received /tf message')
        # 假设机器人当前位置和姿态在第一个transform消息中
        current_position = msg.transforms[0].transform.translation
        current_orientation = msg.transforms[0].transform.rotation
        self.control_robot(current_position, current_orientation)

    def control_robot(self, current_position, current_orientation):
        # 四元数转换为欧拉角（偏航角）
        r = R.from_quat([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])
        euler = r.as_euler('xyz', degrees=True)
        current_yaw = math.radians(euler[2])  # 获取偏航角（以弧度为单位）

        # 目标方向与当前位置的差值
        dx = self.target_position['x'] - current_position.x
        dy = self.target_position['y'] - current_position.y

        # 目标点的角度
        angle_to_target = math.atan2(dy, dx)

        # 计算需要转向的角度差
        angle_difference = angle_to_target - current_yaw
        # 确保角度差在-pi到pi之间
        angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))

        # 目标距离
        distance_to_target = math.sqrt(dx**2 + dy**2)

        # 比例控制参数
        linear_speed_factor = 1.8
        angular_speed_factor = 10

        # 计算线速度和角速度
        linear_speed = linear_speed_factor * distance_to_target
        angular_speed = angular_speed_factor * angle_difference

        # 限速
        max_linear_speed = 2.0
        max_angular_speed = 2.0
        linear_speed = max(min(linear_speed, max_linear_speed), -max_linear_speed)
        angular_speed = max(min(angular_speed, max_angular_speed), -max_angular_speed)

        # 发布速度命令
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: Move towards target {self.target_position} with orientation correction")

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()

    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
