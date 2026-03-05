import rclpy
from rclpy.node import Node
from dl_ros2_msgs.srv import PhotoShoot
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import random 
from datetime import datetime

class TakeAPicture(Node):
    def __init__(self):
        super().__init__('take_a_picture_node')
        self.br = CvBridge()
        self.cv_image = None
        
        self.train_split_ratio = 0.8 

        self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',
            self.image_callback,
            10
        )
        self.create_service(PhotoShoot, 'photoshoot', self.save_image)
        
        self.focus_dir = "/home/wego/wego_ws/src/dataset/dl_images"
        self.yolo_train_dir = "/home/wego/wego_ws/src/dataset/yolo_images/images/train"
        self.yolo_val_dir = "/home/wego/wego_ws/src/dataset/yolo_images/images/val"
                
        for path in [self.focus_dir, self.yolo_train_dir, self.yolo_val_dir]:
            os.makedirs(path, exist_ok=True)
            self.get_logger().info(f'Directory check: {path}')

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def save_image(self, request, response):
        if self.cv_image is None:
            self.get_logger().warn('No image received yet!')
            response.photoshoot = False
            return response
        
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")
        filename = f'img_{timestamp}.jpg'

        try:
            if request.chalkak:
                image = cv2.resize(self.cv_image, (300, 300))
                save_path = os.path.join(self.focus_dir, filename)
                cv2.imwrite(save_path, image)
                self.get_logger().info(f'Saved to DL focus dir: {filename}')
            else:
                if random.random() < self.train_split_ratio:
                    target_dir = self.yolo_train_dir
                    split_type = "TRAIN"
                else:
                    target_dir = self.yolo_val_dir
                    split_type = "VAL"

                save_path = os.path.join(target_dir, filename)
                cv2.imwrite(save_path, self.cv_image)
                
                self.get_logger().info(f'[{split_type}] Saved to YOLO dir: {filename}')

            response.photoshoot = True
        except Exception as e:
            self.get_logger().error(f'Failed to save image: {e}')
            response.photoshoot = False

        return response

def main(args=None):
    rclpy.init(args=args)
    node = TakeAPicture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()