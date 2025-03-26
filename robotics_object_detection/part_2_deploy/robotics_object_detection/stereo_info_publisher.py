import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
import yaml
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import numpy as np

class StereoPublisher(Node):
    def __init__(self):
        super().__init__('stereo_publisher')

        self.img_left_sub = self.create_subscription(
            Image, 'stereo/left/image', self.left_image_callback, 1)
        self.img_right_sub = self.create_subscription(
            Image, 'stereo/right/image', self.right_image_callback, 1)
        
        self.info_left_pub = self.create_publisher(CameraInfo, 'stereo/left/camera_info', 10)
        self.info_right_pub = self.create_publisher(CameraInfo, 'stereo/right/camera_info', 10)

    def left_image_callback(self, msg):
        left_info_msg = self._left_info_msg
        left_info_msg.header.stamp = msg.header.stamp
        left_info_msg.header.frame_id = msg.header.frame_id
        
        self.info_left_pub.publish(left_info_msg)

    def right_image_callback(self, msg):
        right_info_msg = self._right_info_msg
        right_info_msg.header.stamp = msg.header.stamp
        right_info_msg.header.frame_id = msg.header.frame_id
        
        self.info_right_pub.publish(right_info_msg)
        
    def fill_camera_info_msg(self, camera_info_msg, yaml_data):
        camera_info_msg.height = yaml_data['image_height']
        camera_info_msg.width = yaml_data['image_width']
        camera_info_msg.distortion_model = yaml_data['distortion_model']
        camera_info_msg.d = yaml_data['distortion_coefficients']['data']
        camera_info_msg.k = yaml_data['camera_matrix']['data']
        camera_info_msg.r = yaml_data['rectification_matrix']['data']
        camera_info_msg.p = yaml_data['projection_matrix']['data']

        return camera_info_msg

    def create_msgs(self, left_yaml, right_yaml):

        left_info_msg = CameraInfo()
        right_info_msg = CameraInfo()
        
        left_info_msg = self.fill_camera_info_msg(left_info_msg, left_yaml)
        right_info_msg = self.fill_camera_info_msg(right_info_msg, right_yaml)
        
        left_info_msg.header.frame_id = 'camera_left'
        right_info_msg.header.frame_id = 'camera_right'

        self._left_info_msg = left_info_msg
        self._right_info_msg = right_info_msg
        


def main(args=None):
    rclpy.init(args=args)

    package_name = 'robotics_object_detection' #Replace with your package name
    package_share_directory = get_package_share_directory(package_name)

    # read pyyaml files from 'resources' folder
    
    with open(f'{package_share_directory}/calibration_results/left.yaml', 'r') as file:
        left_yaml = yaml.safe_load(file)
        
    with open(f'{package_share_directory}/calibration_results/right.yaml', 'r') as file:
        right_yaml = yaml.safe_load(file)

    print('Left camera info:')
    print(left_yaml)
    print()
    print()
    print('Right camera info:')
    print(right_yaml)
    print()

    node = StereoPublisher()
    node.create_msgs(left_yaml, right_yaml)

    rclpy.spin(node)
            
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
