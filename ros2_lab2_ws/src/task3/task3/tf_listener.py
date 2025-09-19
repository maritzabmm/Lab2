import rclpy
import math
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion

class TFListener(Node):
    """
    Listens for a transform from 'map' to 'lidar' and logs the yaw angle.
    """
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(2.0, self.timer_callback) #Timer every 2 seconds

        self.declare_parameter('target_frame', 'lidar')
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.get_logger().info(f'Looking up transform from map to {self.target_frame}...')
        # Validate frame
        if self.target_frame not in ['lidar', 'scanner']:
            self.get_logger().error("Invalid target_frame parameter. Use 'lidar' or 'scanner'.")
            return

        # Variables for speed calculation
        self.previous_position = None
        self.previous_time = None

        self.get_logger().info('Listener node started!')

    def timer_callback(self):
        """
        Periodically looks up the transform and prints the yaw.
        """
        try:

            transform = self.tf_buffer.lookup_transform(
                'map', # parent frame id
                self.target_frame, # child frame id
                rclpy.time.Time()
            )

            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            # Calculate distance from map origin
            distance = math.sqrt(x*x + y*y + z*z) # Formula reviewed with AI

            # Get current time
            current_time = self.get_clock().now()
            current_position = (x, y, z)
            
            # Calculate speed if we have previous data
            speed = 0.0
            if self.previous_position is not None and self.previous_time is not None:
                # Calculate time difference in seconds
                time_diff = (current_time - self.previous_time).nanoseconds / 1e9
                
                # Calculate position difference
                dx = x - self.previous_position[0]
                dy = y - self.previous_position[1]
                dz = z - self.previous_position[2]
                distance_traveled = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                # Calculate speed (distance/time)
                if time_diff > 0:
                    speed = distance_traveled / time_diff
            
            # Log the information
            self.get_logger().info(
                #f'Position: ({x:.3f}, {y:.3f}, {z:.3f}) | '
                f'Distance from map: {distance:.3f}m | ' # Review units
                f'Speed: {speed:.3f}m/s'
            )
            
            # Update previous values for next iteration
            self.previous_position = current_position
            self.previous_time = current_time

        except TransformException as ex:
            self.get_logger().info(f'Could not transform: {ex}')

def main(args=None):
    rclpy.init(args=args)
    tf_listener = TFListener()
    rclpy.spin(tf_listener)
    tf_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()