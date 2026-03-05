import rclpy
from rclpy.node import Node
from dl_ros2_msgs.msg import Control, BoundingBoxes
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
import cv2
import time
import numpy as np

def sigmoid(x, gain):
    return gain / (1 + np.exp(-x))

class DlControlNode(Node):
    def __init__(self):
        super().__init__('dl_control_node')
        self.br = CvBridge()
        
        self.stop_flag = False
        self.start_flag = False

        self.control_1 = 0.0
        self.control_2 = 0.0

        self.prev_control_1 = 0.0
        self.prev_control_2 = 0.0

        self.flags = {
            'slow_down': {'active': False, 'past_time': time.time(), 'duration_sec': 0.0, 'size_threshold': 0.0},
            'speed_up': {'active': False, 'past_time': time.time(), 'duration_sec': 0.0, 'size_threshold': 0.0},
            'turn_right': {'active': False, 'past_time': time.time(), 'duration_sec': 0.0, 'size_threshold': 0.0},
            'red_light': {'active': False, 'past_time': time.time(), 'duration_sec': 0.0, 'size_threshold': 0.0},
            'yellow_light': {'active': False, 'past_time': time.time(), 'duration_sec': 0.0, 'size_threshold': 0.0},
            'green_light': {'active': False, 'past_time': time.time(), 'duration_sec': 0.0, 'size_threshold': 0.0},
            'pedestrian': {'active': False, 'past_time': time.time(), 'duration_sec': 0.0, 'pass_by_duration_sec': 0.0, 'size_threshold': 0.0}
        }
        
        self.current_image = None
        self.current_boxes = []

        self._declare_parameters()
        self._get_parameters()
        
        self.stop_sub_ = self.create_subscription(Bool, '/emergency_stop', self.stop_callback, 10)
        self.control_sub_ = self.create_subscription(Control, '/control', self.control_callback, 10)
        self.box_sub_ = self.create_subscription(BoundingBoxes, '/yolo_bounding_boxes', self.yolo_callback, 10)
        self.image_sub_ = self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed', self.image_callback, 10)
        
        self.cmd_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dl_image_pub = self.create_publisher(Image, '/dl_image', 10)
        self.dl_image_compressed_pub = self.create_publisher(CompressedImage, '/dl_image/compressed', 10)

        self.create_timer(0.1, self.timer_callback)

    def _declare_parameters(self):
        self.declare_parameter('max_steering', 0.0)
        self.declare_parameter('linear_x_base_speed', 0.0)
        self.declare_parameter('max_speed', 0.0)
        self.declare_parameter('min_speed', 0.0)

        self.declare_parameter('control_1_gain', 0.0)
        self.declare_parameter('control_2_gain', 0.0)

        self.declare_parameter('prev_gain', 0.0)
        self.declare_parameter('current_gain', 0.0)
        self.declare_parameter('threshold_max', 0.0)
        self.declare_parameter('threshold_min', 0.0)
        self.declare_parameter('deceleration_ratio', 0.0)

        self.declare_parameter('speed_up_factor', 0.0)
        self.declare_parameter('slow_down_factor', 0.0)
        self.declare_parameter('turn_right_factor', 0.0)
        self.declare_parameter('yellow_light_factor', 0.0)
        self.declare_parameter('green_light_factor', 0.0)

        self.declare_parameter('slow_down_duration_sec', 0.0)
        self.declare_parameter('speed_up_duration_sec', 0.0)
        self.declare_parameter('turn_right_duration_sec', 0.0)
        self.declare_parameter('red_light_duration_sec', 0.0)
        self.declare_parameter('yellow_light_duration_sec', 0.0)
        self.declare_parameter('green_light_duration_sec', 0.0)
        self.declare_parameter('pedestrian_duration_sec', 0.0)
        self.declare_parameter('pedestrian_pass_by_duration_sec', 0.0)

        self.declare_parameter('slow_down_size_threshold', 0.0)
        self.declare_parameter('speed_up_size_threshold', 0.0)
        self.declare_parameter('turn_right_size_threshold', 0.0)
        self.declare_parameter('red_light_size_threshold', 0.0)
        self.declare_parameter('yellow_light_size_threshold', 0.0)
        self.declare_parameter('green_light_size_threshold', 0.0)
        self.declare_parameter('pedestrian_size_threshold', 0.0)
    
    def _get_parameters(self):
        self.max_steering = self.get_parameter('max_steering').get_parameter_value().double_value
        self.linear_x_base_speed = self.get_parameter('linear_x_base_speed').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.min_speed = self.get_parameter('min_speed').get_parameter_value().double_value

        self.threshold_max = self.get_parameter('threshold_max').get_parameter_value().double_value
        self.threshold_min = self.get_parameter('threshold_min').get_parameter_value().double_value
        
        self.speed_up_factor = self.get_parameter('speed_up_factor').get_parameter_value().double_value
        self.slow_down_factor = self.get_parameter('slow_down_factor').get_parameter_value().double_value
        self.turn_right_factor = self.get_parameter('turn_right_factor').get_parameter_value().double_value
        self.yellow_light_factor = self.get_parameter('yellow_light_factor').get_parameter_value().double_value
        self.green_light_factor = self.get_parameter('green_light_factor').get_parameter_value().double_value
        
        for flag_key in self.flags:
            self.flags[flag_key]['duration_sec'] = self.get_parameter(f'{flag_key}_duration_sec').value
            if flag_key == 'pedestrian':
                self.flags[flag_key]['pass_by_duration_sec'] = self.get_parameter('pedestrian_pass_by_duration_sec').value
            self.flags[flag_key]['size_threshold'] = self.get_parameter(f'{flag_key}_size_threshold').value

    def stop_callback(self, msg: Bool):
        self.stop_flag = msg.data

    def image_callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.current_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if self.current_image is None:
                self.get_logger().warn("Failed to decode compressed image")
        except Exception as e:
            self.get_logger().warn(f"Failed to process image: {e}")

    def apply_derivative_control(self, current_val, prev_val):
            diff = abs(current_val - prev_val)

            if diff < self.threshold_min:
                return prev_val

            if abs(current_val) < abs(prev_val) and diff < self.threshold_max:
                return prev_val * self.get_parameter('prev_gain').value + current_val * self.get_parameter('current_gain').value

            return current_val

    def control_callback(self, msg: Control):
        raw_control_1 = msg.normal_mode * self.get_parameter('control_1_gain').value
        raw_control_2 = msg.turn_mode * self.get_parameter('control_2_gain').value

        self.control_1 = self.apply_derivative_control(raw_control_1, self.prev_control_1)
        self.control_2 = self.apply_derivative_control(raw_control_2, self.prev_control_2)

        self.start_flag = True

    def yolo_callback(self, msg: BoundingBoxes):
        self.current_boxes = msg.bounding_boxes
        current_time = time.time()
        
        temp_flags = {key: False for key in self.flags}
        for box in self.current_boxes:
            if box.class_name in self.flags:
                area = (box.xmax - box.xmin) * (box.ymax - box.ymin)
                if area > self.flags[box.class_name]['size_threshold']:
                    temp_flags[box.class_name] = True

        for flag_key, flag_data in self.flags.items():
            
            if flag_key == 'pedestrian':
                time_elapsed = current_time - flag_data['past_time']

                if temp_flags['pedestrian']: 
                    if time_elapsed > flag_data['pass_by_duration_sec']:
                        flag_data['past_time'] = current_time
                        flag_data['active'] = True 
                    else:
                        flag_data['active'] = time_elapsed <= flag_data['duration_sec']
                else: 
                    flag_data['active'] = time_elapsed <= flag_data['duration_sec']

            else:
                if temp_flags[flag_key]:
                    flag_data['past_time'] = current_time
                    flag_data['active'] = True
                else:
                    flag_data['active'] = current_time - flag_data['past_time'] <= flag_data['duration_sec']

    def timer_callback(self):
        self.publish_cmd_vel()
        self.publish_debug_image()

    def publish_cmd_vel(self):
        cmd = Twist()
        
        if self.stop_flag or not self.start_flag:
            self.cmd_pub_.publish(cmd)
            return

        if self.flags['red_light']['active'] or self.flags['pedestrian']['active']:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            sigmoid_1 = self.control_1 * sigmoid(self.control_1, self.max_steering)
            sigmoid_2 = self.control_2 * sigmoid(self.control_2, self.max_steering) 
            
            if self.flags['turn_right']['active']:
                cmd.linear.x = self.linear_x_base_speed - (abs(sigmoid_2) * self.get_parameter('deceleration_ratio').value)
                cmd.angular.z = sigmoid_2 * self.turn_right_factor
            else:
                cmd.linear.x = self.linear_x_base_speed - (abs(sigmoid_1) * self.get_parameter('deceleration_ratio').value)
                cmd.angular.z = sigmoid_1

            if self.flags['speed_up']['active']:
                cmd.linear.x *= self.speed_up_factor
                cmd.angular.z *= self.speed_up_factor
            elif self.flags['slow_down']['active']:
                cmd.linear.x *= self.slow_down_factor
                cmd.angular.z *= self.slow_down_factor / 2

            if self.flags['yellow_light']['active']:
                cmd.linear.x *= self.yellow_light_factor
                cmd.angular.z *= self.yellow_light_factor / 2
            elif self.flags['green_light']['active']:
                cmd.linear.x *= self.green_light_factor
                cmd.angular.z *= self.green_light_factor

            cmd.linear.x = np.clip(cmd.linear.x, self.min_speed, self.max_speed)
            cmd.angular.z = np.clip(cmd.angular.z, -self.max_steering, self.max_steering)

        self.cmd_pub_.publish(cmd)

    def publish_debug_image(self):
        if self.current_image is None:
            return

        debug_image = self.current_image.copy()
        h, w, _ = debug_image.shape

        for box in self.current_boxes:
            xmin, ymin, xmax, ymax = box.xmin, box.ymin, box.xmax, box.ymax
            label = f"{box.class_name}: {box.probability:.2f}"
            
            area = (xmax - xmin) * (ymax - ymin)
            box_color = (0, 0, 255)

            if box.class_name in self.flags:
                threshold = self.flags[box.class_name]['size_threshold']
                if area > threshold:
                    box_color = (0, 255, 0)

            cv2.rectangle(debug_image, (xmin, ymin), (xmax, ymax), box_color, 2)
            cv2.putText(debug_image, label, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
    
        ref_x = w // 2 
        scale_factor = w / 300.0
        line_y = h - 100

        if self.flags['turn_right']['active']:
            turn_mode_scaled = self.control_2 / self.get_parameter('control_2_gain').value
            cx_2_300 = 150 - turn_mode_scaled
            cx_2_scaled = int(cx_2_300 * scale_factor)

            cv2.putText(debug_image, 'Turn Mode', (cx_2_scaled, line_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.line(debug_image, (ref_x, line_y + 35), (cx_2_scaled, line_y + 35), (0, 0, 255), 3)
            cv2.circle(debug_image, (ref_x, line_y + 35), 7, (0, 255, 0), -1)
            cv2.circle(debug_image, (cx_2_scaled, line_y + 35), 7, (0, 0, 255), -1)
        else:
            normal_mode_scaled = self.control_1 / self.get_parameter('control_1_gain').value
            cx_1_300 = 150 - normal_mode_scaled
            cx_1_scaled = int(cx_1_300 * scale_factor)

            cv2.putText(debug_image, 'Normal Mode', (cx_1_scaled, line_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            cv2.line(debug_image, (ref_x, line_y + 35), (cx_1_scaled, line_y + 35), (255, 0, 0), 3)
            cv2.circle(debug_image, (ref_x, line_y + 35), 7, (0, 255, 0), -1) 
            cv2.circle(debug_image, (cx_1_scaled, line_y + 35), 7, (255, 0, 0), -1) 
        
        out_msg = self.br.cv2_to_imgmsg(debug_image, encoding='bgr8')
        self.dl_image_pub.publish(out_msg)

        debug_compressed_msg = self.br.cv2_to_compressed_imgmsg(debug_image)
        self.dl_image_compressed_pub.publish(debug_compressed_msg)

def main(args=None):
    rclpy.init(args=args)
    dl_control_node = None
    try:
        dl_control_node = DlControlNode()
        rclpy.spin(dl_control_node)
    except Exception as e:
        print(f"Error running node: {e}")
    finally:
        if dl_control_node is not None:
            dl_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
