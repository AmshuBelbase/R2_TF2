from geometry_msgs.msg import TransformStamped
import rclpy
import os 
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import random
import matplotlib.pyplot as plt
import cv2 
import sys
import numpy as np
import serial 
import time 
from launch_ros.substitutions import FindPackageShare
INPUT_WIDTH = 640
INPUT_HEIGHT = 640
SCORE_THRESHOLD = 0.2 
NMS_THRESHOLD = 0.4
CONFIDENCE_THRESHOLD = 0.4
cam_source = 2
FRAME_SKIP = 2  # Number of frames to skip
is_cuda = len(sys.argv) > 1 and sys.argv[1] == "cuda"

class FramePublisher(Node):

    def __init__(self):
        super().__init__('detected_tf2_broadcaster')
        pkg_share = FindPackageShare(package='my_r2_tf2').find('my_r2_tf2')
        class_file_name = 'classes.txt'
        onnx_file_name = 'best.onnx'
        self.class_path = os.path.join(pkg_share, class_file_name)
        self.onnx_path = os.path.join(pkg_share, onnx_file_name)
        
        self.target_frame = self.declare_parameter(
            'target_frame', 'ball').get_parameter_value().string_value  
        
        class_list = self.load_classes()

        colors = [(255, 255, 0), (0, 255, 0), (0, 255, 255), (255, 0, 0)] 

        net = self.build_model(is_cuda)

        capture = self.load_capture()

        start = time.time_ns()
        frame_count = 0
        total_frames = 0
        fps = -1

        prev_frame = None  # Previous frame for interpolation

        while True: 
            _, frame_all = capture.read()
            if frame_all is None:
                print("End of stream")
                break
            frame = frame_all
            # Split the stereo frame into left and right images
            if cam_source == 2:
                height, width, _ = frame_all.shape
                width //= 2
                frame = frame_all[:, :width, :]
                right_frame = frame_all[:, width:, :]
                prev_frame = frame.copy()  # Save current frame for interpolation 
                
            frame_count += 1
            total_frames += 1

            # Skip frames if needed
            if frame_count % FRAME_SKIP != 0:
                continue

            inputImage = FramePublisher.format_yolov5(frame)
            outs = FramePublisher.detect(inputImage, net)

            class_ids, confidences, boxes = FramePublisher.wrap_detection(inputImage, outs[0])
            # Interpolate with previous frame
            if prev_frame is not None:
                alpha = 0.5  # Interpolation factor
                frame = cv2.addWeighted(prev_frame, alpha, frame, 1-alpha, 0)
                
            if not boxes:
                self.linear_x = 0
                self.linear_y = 0
                self.tf_broadcaster = TransformBroadcaster(self)
                self.on_timer_publish()
            else:  
                for (classid, confidence, box) in zip(class_ids, confidences, boxes):
                    color = colors[int(classid) % len(colors)]
                    cv2.rectangle(frame, box, color, 2)
                    cv2.rectangle(frame, (box[0], box[1] - 20),
                                (box[0] + box[2], box[1]), color, -1)
                    cv2.putText(frame, class_list[classid], (box[0],
                                box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0))
                    height, width, channels = frame.shape  
                    # print(box)
                    dist = 0
                    if(box[2]>box[3]):
                        dist = box[2]
                    else:
                        dist = box[3]

                    width_height_max = [16, 18, 20, 21, 25, 28, 33, 40, 48, 54, 58, 64, 69, 74, 85]
                    ball_distance = [310, 280, 250, 230, 200, 180, 150, 125, 100, 90, 80, 70, 60, 50, 40] 
                    dist = np.interp(dist, width_height_max, ball_distance)
                    dist = int(dist)
 
                    # Define the input and output range
                    i_min = 40
                    i_max = 310
                    o_min = 100
                    o_max = 60
                    scale_factor = 100
                    if(dist > 310 or dist < 40):
                        scale_factor = 100
                    else:
                        scale_factor = (dist-i_min) * (o_max-o_min) / (i_max - i_min) + o_min 
 

                    self.linear_x = (int(width/2) - box[0])/scale_factor #nominal 40
                    self.linear_y = (height - box[1])/scale_factor

                    self.get_logger().info(f'distance - {dist} scale factor - {scale_factor} linear_x - {self.linear_x} linear_y - {self.linear_y}')

                    # plt.arrow(0,0,self.linear_x,self.linear_y, width=0.5)
                    # plt.show(block=False)
                    # plt.pause(0.01) 
                    # Initialize the transform broadcaster
                    self.tf_broadcaster = TransformBroadcaster(self)
                    self.on_timer_publish()
                    # self.timer = self.create_timer(10.0, self.on_timer_publish)  

            if frame_count >= 0:  # 30
                end = time.time_ns()
                fps = 1000000000 * frame_count / (end - start)
                frame_count = 0
                start = time.time_ns()

            if fps > 0:
                fps_label = "FPS: %.2f" % fps
                cv2.putText(frame, fps_label, (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            cv2.imshow("output", frame)

            if cv2.waitKey(1) > -1:
                print("finished by user")
                break 

    @staticmethod
    def format_yolov5(frame): 
        row, col, _ = frame.shape
        _max = max(col, row)
        result = np.zeros((_max, _max, 3), np.uint8)
        result[0:row, 0:col] = frame
        return result


    def on_timer_publish(self):
        t = TransformStamped() 
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'r2'
        t.child_frame_id = self.target_frame

        x_axis = self.linear_x
        y_axis = self.linear_y

        x_axis_f = float(x_axis)
        y_axis_f = float(y_axis) 
        # We get x and y translation coordinates from the message
        # And set the z coordinate to 0
        t.transform.translation.x = x_axis_f
        t.transform.translation.y = y_axis_f
        t.transform.translation.z = 0.0 
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.get_logger().info(f'Transform Published - {t.header.frame_id} to {self.target_frame}')

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def wrap_detection(input_image, output_data):
        class_ids = []
        confidences = []
        boxes = []

        rows = output_data.shape[0]

        image_width, image_height, _ = input_image.shape

        x_factor = image_width / INPUT_WIDTH
        y_factor = image_height / INPUT_HEIGHT
        
        for r in range(rows):
            row = output_data[r]
            confidence = row[4]
            if confidence >= 0.4:

                classes_scores = row[5:]
                _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
                class_id = max_indx[1]
                if (classes_scores[class_id] > .25):

                    confidences.append(confidence)

                    class_ids.append(class_id)

                    x, y, w, h = row[0].item(), row[1].item(
                    ), row[2].item(), row[3].item()
                    left = int((x - 0.5 * w) * x_factor)
                    top = int((y - 0.5 * h) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    box = np.array([left, top, width, height])
                    boxes.append(box)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.25, 0.45)

        result_class_ids = []
        result_confidences = []
        result_boxes = []

        for i in indexes:
            result_confidences.append(confidences[i])
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])

        return result_class_ids, result_confidences, result_boxes
    
    def build_model(self, is_cuda):
        net = cv2.dnn.readNet(self.onnx_path)
        if is_cuda:
            print("Attempting to use CUDA")
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
        else:
            print("Running on CPU")
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        return net
    
    @staticmethod
    def detect(image, net):
        blob = cv2.dnn.blobFromImage(
            image, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
        net.setInput(blob)
        preds = net.forward()
        return preds


    def load_capture(self):
        # capture = cv2.VideoCapture("./video17.mp4")
        capture = cv2.VideoCapture(cam_source)  # Open camera capture object
        return capture
 
    def load_classes(self):
        class_list = []
        current_directory = os.getcwd()  
        self.get_logger().info(f'Current working directory: - {current_directory}')
        with open(self.class_path, "r") as f:
            class_list = [cname.strip() for cname in f.readlines()]
        return class_list


def main(): 
    rclpy.init()
    node = FramePublisher()  
    rclpy.shutdown() 
