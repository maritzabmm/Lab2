import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler

class TFRLStatic(Node):
    """
    Post a transform from 'robot' to 'lidar'
    lidar is positioned with an offset in X (forward) and Z (up) from the robot center.
    """
    def __init__(self):
        super().__init__('tf_rl_static')
        self.tf_rl_static = StaticTransformBroadcaster(self)
        self.make_transforms()
        self.get_logger().info('Transform published!')

    def make_transforms(self):
        transform_stamped = TransformStamped()

        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'robot' # Parent frame - the global reference
        transform_stamped.child_frame_id = 'lidar' # Child frame - moves in circular path

        # Set translation
        transform_stamped.transform.translation.x = 0.2 # 20cm forward (0.2 meters)
        transform_stamped.transform.translation.y = 0.0
        transform_stamped.transform.translation.z = 0.3 # 30cm up (0.3 meters)
        # x and z values generated with autocompletion

        # Set rotation using roll, pitch, and yaw
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        q = quaternion_from_euler(roll, pitch, yaw)

        self.get_logger().info('Quaternion values: x={}, y={}, z={}, w={}'.format(q[0], q[1], q[2], q[3]))

        transform_stamped.transform.rotation.x = q[0]
        transform_stamped.transform.rotation.y = q[1]
        transform_stamped.transform.rotation.z = q[2]
        transform_stamped.transform.rotation.w = q[3] 

        self.tf_rl_static.sendTransform(transform_stamped)

def main(args=None):
    rclpy.init(args=args)
    tf_rl_static = TFRLStatic()
    rclpy.spin(tf_rl_static)
    tf_rl_static.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()