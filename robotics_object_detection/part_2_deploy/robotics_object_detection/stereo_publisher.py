import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import depthai as dai

class StereoPublisher(Node):
    def __init__(self):
        super().__init__('stereo_publisher')

        self.img_left_pub = self.create_publisher(
            Image, 'stereo/left/image', 10)
        self.img_right_pub = self.create_publisher(
            Image, 'stereo/right/image', 10)

        self.bridge = CvBridge()

    def publish_images(self, left_image_bgr, right_image_bgr):

        left_img_msg = self.bridge.cv2_to_imgmsg(
            left_image_bgr, encoding='mono8')
        right_img_msg = self.bridge.cv2_to_imgmsg(
            right_image_bgr, encoding='mono8')

        stamp = self.get_clock().now().to_msg()

        left_img_msg.header.stamp = stamp
        right_img_msg.header.stamp = stamp

        left_img_msg.header.frame_id = 'camera_left'
        right_img_msg.header.frame_id = 'camera_right'

        self.img_left_pub.publish(left_img_msg)
        self.img_right_pub.publish(right_img_msg)


def main(args=None):
    rclpy.init(args=args)

    node = StereoPublisher()

    device = dai.Device()
    pipeline = dai.Pipeline()
    
    camLeft = pipeline.create(dai.node.MonoCamera)
    camRight = pipeline.create(dai.node.MonoCamera)
    
    camLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    camLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
    camLeft.setFps(5)
    
    camRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    camRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    camRight.setFps(5)
    
    xoutLeft = pipeline.create(dai.node.XLinkOut)
    xoutRight = pipeline.create(dai.node.XLinkOut)
    
    xoutLeft.setStreamName("left")
    xoutRight.setStreamName("right")
    
    camLeft.out.link(xoutLeft.input)
    camRight.out.link(xoutRight.input)
    
    streams = ["left", "right"]
    
    print("Pipeline created. Pipeline is starting now.")

    with device:
        device.startPipeline(pipeline)
        
        qList = [device.getOutputQueue(stream, 8, blocking=False) for stream in streams]
        
        while rclpy.ok():
            
            left_image = None
            right_image = None
            
            for q in qList:
                name = q.getName()
                frame = q.get().getCvFrame()
                
                if name == "left":
                    left_image = frame
                if name == "right":
                    right_image = frame
                    
            if left_image is not None and right_image is not None:
                print("Publishing images")
                node.publish_images(left_image, right_image)
                left_image = None
                right_image = None
            
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
