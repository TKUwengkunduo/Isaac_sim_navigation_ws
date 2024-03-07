from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

class IMUToEulerNode(Node):
    def __init__(self):
        super().__init__('imu_to_euler_node')
        self.subscription = self.create_subscription(
            Imu,
            'imu/data_raw',
            self.imu_callback,
            10)
        self.euler_pub = self.create_publisher(Vector3, '/imu/euler_angles', 10)

    def imu_callback(self, msg):
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        
        # Convert quaternion to Euler angles in degrees
        r = R.from_quat(quaternion)
        roll, pitch, yaw = r.as_euler('xyz', degrees=True)
        
        # Publish Euler angles in degrees
        euler_msg = Vector3()
        euler_msg.x = roll  # Roll in degrees
        euler_msg.y = pitch  # Pitch in degrees
        euler_msg.z = yaw  # Yaw in degrees
        self.euler_pub.publish(euler_msg)

        # Log Euler angles, formatted to 3 decimal places
        self.get_logger().info(
            f'Published Euler angles in degrees: Roll={roll:4.3f}, Pitch={pitch:4.3f}, Yaw={yaw:4.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    imu_to_euler_node = IMUToEulerNode()
    rclpy.spin(imu_to_euler_node)
    imu_to_euler_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
