import numpy as np
import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist 
import serial 
import time

serial_port = '/dev/ttyACM0'
baud_rate = 115200 
ser = serial.Serial(serial_port, baud_rate, timeout=1)

class GetCmdVel(Node):

    def __init__(self):
        super().__init__('get_cmd_vel')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)

    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x
        linear_y = msg.linear.y 
        angular_z = msg.angular.z
        self.get_logger().info(f"X: {linear_x}, Y: {linear_y}, Z: {angular_z}")  
        matrix_4x3 = np.array([[15.75, 0, -5.66909078166105],
                            [0, 15.75, 5.66909078166105],
                            [-15.75, 0, 5.66909078166105],
                            [0, -15.75,-5.66909078166105]]) 
 
        matrix_3x1 = np.array([[linear_x],
                            [linear_y],
                            [angular_z]])
        
        result_matrix = np.dot(matrix_4x3, matrix_3x1)        
          
        # Define floats to send
        fr = result_matrix[0,0]
        fl = result_matrix[1,0]
        bl = result_matrix[2,0]
        br = result_matrix[3,0]

        # Convert to bytes
        data = (str(fr) + '|' + 
                str(fl) + '|' +
                str(bl) + '|' +
                str(br)) + "#"
        
        # Send data
        ser.write(data.encode())  
        self.get_logger().info(f"Sent: {data}")
        self.get_logger().info(f"Front Right: {result_matrix[0,0]}, Front Left: {result_matrix[1,0]}, Back Left: {result_matrix[2,0]}, Back Right: {result_matrix[3,0]}")

def main(args=None):
    rclpy.init(args=args)
    node = GetCmdVel()
    rclpy.spin(node)
    node.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
