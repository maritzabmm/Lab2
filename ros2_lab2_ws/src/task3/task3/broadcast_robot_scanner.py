import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

class TFRSBroadcaster(Node):
    """
    Broadcasts a transform from the 'robot' frame to the 'scanner' frame.
    The 'scanner' oscillates left and right at a given frequency
    positioned above the robot with Z and X offsets.
    """
    def __init__(self):
        super().__init__('tf_rs_broadcaster')
        self.tf_rs_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info('Node with scanner started!')

    def timer_callback(self):
        """
        Periodically broadcasts the transform.
        """
        t = self.get_clock().now().seconds_nanoseconds()[0] + \
            self.get_clock().now().seconds_nanoseconds()[1] / 1e9

        # Transform message
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'robot' # Parent frame - the global reference
        transform_stamped.child_frame_id = 'scanner' # Child frame - moves in circular path

        # Oscillation in Y
        amplitude = 0.3 # in meters
        frequency = 1.0 # in Hz
        y_oscillation = amplitude * math.sin(2 * math.pi * frequency * t) # reviewed with AI
        
        # Set translation for scanner
        x_offset = 0.1 # 10cm forward from robot center
        z_offset = 0.4 # 40cm above robot center
        transform_stamped.transform.translation.x = x_offset
        transform_stamped.transform.translation.y = y_oscillation
        transform_stamped.transform.translation.z = z_offset

        # Set rotation using roll, pitch, and yaw
        roll = 0.0
        pitch = 0.0
        yaw =-math.atan2(y_oscillation, x_offset) * 0.5  # reviewed with AI
        q = quaternion_from_euler(roll, pitch, yaw)

        transform_stamped.transform.rotation.x = q[0]
        transform_stamped.transform.rotation.y = q[1]
        transform_stamped.transform.rotation.z = q[2]
        transform_stamped.transform.rotation.w = q[3]

        self.tf_rs_broadcaster.sendTransform(transform_stamped)

def main(args=None):
    rclpy.init(args=args)
    tf_rs_broadcaster = TFRSBroadcaster()
    rclpy.spin(tf_rs_broadcaster)
    tf_rs_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()