import numpy as np
import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist 



class GetCmdVel(Node):

    def __init__(self):
        super().__init__('get_cmd_vel') 
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z
        self.get_logger().info(f"X: {linear_x}, Y: {linear_y}, Z: {angular_z}") 
        # Define a Inverse Kinetics ~ 4x3 matrix
        matrix_4x3 = np.array([[11.1347660235131, -11.1363136451255, -5.21299212039026],
                            [11.1368294711969, 11.1342501018466, 5.21299207563835],
                            [-11.1337341562841, 11.1373452733668, 5.21299207563835],
                            [-11.1357977951537, -11.1352819212825, -5.21299212039026]])

        # Define a Bot Velocity ~ 3x1 matrix
        matrix_3x1 = np.array([[linear_x],
                            [linear_y],
                            [angular_z]])
        # Perform matrix multiplication
        result_matrix = np.dot(matrix_4x3, matrix_3x1)
        self.get_logger().info(f"Front Right: {result_matrix[0,0]}, Front Left: {result_matrix[1,0]}, Back Left: {result_matrix[2,0]}, Back Right: {result_matrix[3,0]}")        

def main(args=None):
    rclpy.init(args=args)
    node = GetCmdVel()
    rclpy.spin(node)
    node.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
