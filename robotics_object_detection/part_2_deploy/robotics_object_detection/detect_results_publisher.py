from dataclasses import dataclass

from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import torch
from ultralytics import YOLO



@dataclass
class DetectResults:
    score: float
    class_id: int
    box: list[int, int, int, int]  # [x1, y1, x2, y2]

    def write_as_csv(self, frame_id: int, file):
        """Write the detection result as a CSV line.
        
        Parameters
        ----------
        frame_id : int
            The frame ID of the image.
        file : file object
            The file object to write to.
        """
        ### TODO ### 
        # Write the detection result as a CSV line, update the csv_row string
        # You can use {variable} to format the string
        # Use commas to separate the values
        # CSV format: frame_id (int), class_id (int), score, x1 (int), y1 (int), x2 (int), y2 (int) - without spaces
        # Remember to keep the newline character at the end of the string!
        # Example CSV row: "0,1,0.99,10,20,30,40\n"
        csv_row = f'{frame_id},\n'
        
        ### END OF TODO ###
        
        file.write(csv_row)

class DetectResultsPublisher(Node):
    def __init__(self, model_path: str):
        super().__init__('detect_results_publisher')
        
        self.model = YOLO(
            model=model_path,
            task='detect'
            )
        
        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Using device: {self.device}')
        
        self.frame_id = 0
        
        ### TODO ###
        # Create a subscription to the compressed image topic
        # The message type is sensor_msgs.msg.CompressedImage
        # Check the topic name in the ROSBAG file
        # The queue size is 10
        # The callback function is image_callback
        
        self.image_sub = None # Replace with the subscription code
        
        #### END OF TODO ###
        
        self.image_pub = self.create_publisher(
            CompressedImage,
            '/camera/detected_image/compressed',
            10
        )
        
        self.output_file = open('detection_results.txt', 'w')
        
    def image_callback(self, msg):
        """Callback function for the image subscriber.
        Parameters
        ----------
        msg : sensor_msgs.msg.CompressedImage
            The compressed image message.
        """
        
        # Convert the compressed image message to a numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # Perform inference
        model_predictions: DetectResults = self.infer_image(image)
        self.get_logger().info(f'Processing frame: {self.frame_id}, Number of detections: {len(model_predictions)}')
        
        # Publish the results
        self.publish_results(image, model_predictions)
        self.write_results(self.frame_id, model_predictions)
        
        self.frame_id += 1
        

    def infer_image(self, image: np.ndarray) -> list[DetectResults]:
        """Run inference on the image and return the results.
        
        Parameters
        ----------
        image : np.ndarray
            The input image in BGR format.
        Returns
        -------
        list[DetectResults]
            A list of detection results.
        """
        ### TODO ###
        # Run inference on the image using the YOLO model "self.model.predict"
        # Set the following parameters:
        # - imgsz=640
        # - conf=0.25
        # - classes=[0, 1, 2] # person, bicycle, car
        # - verbose=False
        # - show=False
        # - device=self.device
        # The results should be stored in the variable "results" 
        
        results = None # Replace with the inference code
        
        ### END OF TODO ###
        
        processed_results: list[DetectResults] = []
        for result in results:
            for box in result.boxes:
                ### TODO ###
                # Create a DetectResults object for each detection
                # Use the box attributes: box.conf[0], box.cls[0], box.xyxy[0]
                # Remember to convert variables to the correct types used in the DetectResults class
                
                new_result = None # Replace with the DetectResults object creation
                ### END OF TODO ###
                
                processed_results.append(new_result)
        
        return processed_results
        
    def publish_results(self, image: np.ndarray, results: list[DetectResults]):
        """Draw the detection results on the image and publish it.
        Parameters
        ----------
        image : np.ndarray
            The input image in BGR format.
        results : list[DetectResults]
            The detection results.
        """
        
        for result in results:
            if result is not None:
                ### TODO ###
                # Draw the bounding box on the image, use the cv2.rectangle function
                    # Here is example: https://roboflow.com/use-opencv/draw-a-rectangle-with-cv2-rectangle
                # Draw the class ID and score using cv2.putText
                    # Here is example: https://roboflow.com/use-opencv/draw-text-with-cv2-puttext
                
                pass # Replace with the drawing code
            
                ### END OF TODO
                
            
        # Convert the image to CompressedImage message
        _, buffer = cv2.imencode('.jpg', image)
        compressed_image = CompressedImage()
        compressed_image.header.stamp = self.get_clock().now().to_msg()
        compressed_image.format = 'jpeg'
        compressed_image.data = buffer.tobytes()
        compressed_image.header.frame_id = 'camera_frame'
        
        # Publish the image
        self.image_pub.publish(compressed_image)

        
    def write_results(self, frame_id:int, results: list[DetectResults]):
        """Write the detection results to a file.
        Parameters
        ----------
        frame_id : int
            The frame ID of the image.
        results : list[DetectResults]
            The detection results.
        """
        for result in results:
            if result is not None:
                result.write_as_csv(frame_id, self.output_file)        


def main(args=None):
    rclpy.init(args=args)

    package_name = 'robotics_object_detection' #Replace with your package name
    package_share_directory = get_package_share_directory(package_name)

    model_path = f'{package_share_directory}/yolo11n.pt'

    node = DetectResultsPublisher(model_path=model_path)

    rclpy.spin(node)
    
    node.output_file.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
