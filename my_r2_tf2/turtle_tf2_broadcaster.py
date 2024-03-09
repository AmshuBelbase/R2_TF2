from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import random
import matplotlib.pyplot as plt

class FramePublisher(Node):

    def __init__(self):
        super().__init__('turtle_tf2_broadcaster')

        # Declare and acquire `turtlename` parameter
        self.target_frame = self.declare_parameter(
            'target_frame', 'ball').get_parameter_value().string_value  

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.on_timer_publish()
        self.timer = self.create_timer(10.0, self.on_timer_publish) 

    def on_timer_publish(self):
        t = TransformStamped() 
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'r2'
        t.child_frame_id = self.target_frame

        x_axis = random.randint(-8, 8)
        y_axis = random.randint(-8, 8)
        # plt.arrow(0, 0, x_axis, y_axis, width = 0.05)
        # plt.show(block=False)
        # plt.pause(3)
        # x_axis = -4
        # y_axis = 4
        x_axis_f = float(x_axis)
        y_axis_f = float(y_axis)

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = x_axis_f
        t.transform.translation.y = y_axis_f
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message 
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.get_logger().info(f'Transform Published - {t.header.frame_id} to {self.target_frame}')

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
