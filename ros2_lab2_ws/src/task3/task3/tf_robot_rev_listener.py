import rclpy
import math
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion

class TFRRListener(Node):
    """
    Listens for a transform from 'map' to 'lidar' and logs the yaw angle.
    """
    def __init__(self):
        super().__init__('tf_rr_listener')
        self.tf_buffer = Buffer()
        self.tf_rr_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.timer_callback) #Timer every 0.1 seconds

        # Rev variables
        self.previous_angle = None
        self.total_angle_change = 0.0
        self.revolutions = 0.0
        self.angle_threshold = math.pi  # Threshold for detecting wraparound

        self.get_logger().info('Listener for revs node started!')

    def timer_callback(self):
        """
        Periodically looks up the transform and prints the yaw.
        """
        try:

            transform = self.tf_buffer.lookup_transform(
                'map', # parent frame id
                'robot', # child frame id
                rclpy.time.Time()
            )

            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # Calculate angle from origin (atan2 gives angle from -π to π)
            current_angle = math.atan2(y, x)

            if self.previous_angle is not None:
                # Calculate angle difference
                angle_diff = current_angle - self.previous_angle
                
                # Handle wraparound (when angle jumps from π to -π or vice versa)
                # If statements reviewed with AI
                if angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                elif angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
                
                # Accumulate total angle change
                self.total_angle_change += angle_diff
                
                # Calculate number of complete revolutions
                self.revolutions = self.total_angle_change / (2 * math.pi)
                
                # Log every 1 second (10 callbacks at 0.1s interval)
                if int(self.get_clock().now().nanoseconds / 1e9) % 1 == 0:
                    self.get_logger().info(
                        f'Total revolutions: {self.revolutions:.3f} | '
                        f'Complete revolutions: {int(abs(self.revolutions))}'
                    )
            
            # Update previous angle
            self.previous_angle = current_angle

        except TransformException as ex:
            self.get_logger().info(f'Could not transform: {ex}')

def main(args=None):
    rclpy.init(args=args)
    tf_rr_listener = TFRRListener()
    rclpy.spin(tf_rr_listener)
    tf_rr_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()