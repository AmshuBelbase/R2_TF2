# import math
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 

class FrameListener(Node):

    def __init__(self):
        super().__init__('tf2_to_cmd_vel')

        self.target_frame = self.declare_parameter(
            'target_frame', 'ball').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        from_frame_rel = self.target_frame
        to_frame_rel = 'r2' 

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        msg = Twist()
        
        scale_forward_speed = 0.5
        msg.linear.x = scale_forward_speed * t.transform.translation.x
        msg.linear.y = scale_forward_speed * t.transform.translation.y
        msg.angular.z = t.transform.rotation.z
        self.publisher.publish(msg) 


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
