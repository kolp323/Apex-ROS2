import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ObstacleAnalyzer(Node):
    def __init__(self):
        super().__init__('obstacle_analyzer')

        # --- ROS Parameters ---
        # You can tune these live using: ros2 param set /lane_correction_node <param_name> <value>
        self.declare_parameter('kp', 0.0008)
        self.declare_parameter('kd', 0.0004)
        self.declare_parameter('max_correction', 0.3) # Limits max angular velocity correction (rad/s)
        self.declare_parameter('roi_ratio', 0.6)      # Height ratio for ROI
        self.declare_parameter('enable_debug', False) # Set true to see debug images via topic

        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.max_corr = self.get_parameter('max_correction').value
        self.roi_ratio = self.get_parameter('roi_ratio').value
        self.enable_debug = self.get_parameter('enable_debug').value

        # --- Internal Variables ---
        self.last_error = 0.0
        self.bridge = CvBridge()

        # --- Topics ---
        # Subscribe to camera image
        self.sub_image = self.create_subscription(
            Image, 
            '/camera/image_raw', # CHANGE THIS to your actual camera topic
            self.image_callback, 
            10
        )
        
        # Publish correction value (float)
        self.pub_correction = self.create_publisher(Float32, '/lane/angular_correction', 10)
        
        # Optional: Publish debug image
        self.pub_debug_img = self.create_publisher(Image, '/lane/debug_image', 10)

        self.get_logger().info("Lane Correction Node Started.")

    def refresh_parameters(self):
        """Update parameters dynamically"""
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.max_corr = self.get_parameter('max_correction').value
        self.roi_ratio = self.get_parameter('roi_ratio').value
        self.enable_debug = self.get_parameter('enable_debug').value


    def find_lane_edge_scanline(self, binary_img):
        """Find lane center using scanlines (Your logic)"""
        h, w = binary_img.shape[:2]
        center_x = w // 2
        
        scan_rows = [h//2, h//2 + 20, h//2 - 20]
        left_edges = []
        right_edges = []

        for row_y in scan_rows:
            if row_y < 0 or row_y >= h: continue
            row_data = binary_img[row_y, :]
            
            # Find left edge
            l_found = -1
            for x in range(center_x, 0, -1):
                if row_data[x] == 255: 
                    l_found = x
                    break
            if l_found != -1: left_edges.append(l_found)

            # Find right edge
            r_found = -1
            for x in range(center_x, w - 1):
                if row_data[x] == 255: 
                    r_found = x
                    break
            if r_found != -1: right_edges.append(r_found)

        target_x = -1
        # Logic to determine target X based on found edges
        if len(left_edges) > 0 and len(right_edges) > 0:
            avg_l = int(np.mean(left_edges))
            avg_r = int(np.mean(right_edges))
            if (avg_r - avg_l) > 100: 
                target_x = (avg_l + avg_r) // 2
            else:
                # If too narrow, pick the closer one
                if abs(avg_l - center_x) < abs(avg_r - center_x):
                    target_x = avg_l + 250 
                else:
                    target_x = avg_r - 250
        elif len(left_edges) > 0:
            avg_l = int(np.mean(left_edges))
            target_x = avg_l + 220 
        elif len(right_edges) > 0:
            avg_r = int(np.mean(right_edges))
            target_x = avg_r - 220

        return target_x

    def image_callback(self, msg):
        self.refresh_parameters()

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        h, w = cv_image.shape[:2]
        roi_h_start = int(h * (1 - self.roi_ratio))
        roi = cv_image[roi_h_start:h, :]
        
        # Debug image
        debug_img = None
        if self.enable_debug:
            debug_img = cv_image.copy()
            cv2.rectangle(debug_img, (0, roi_h_start), (w, h), (0, 255, 0), 2)


        # 2. Preprocessing (Your color filtering logic)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Filter saturation (remove colorful items like blue stickers)
        s_channel = hsv[:, :, 1]
        _, low_sat_mask = cv2.threshold(s_channel, 80, 255, cv2.THRESH_BINARY_INV) 

        # Filter brightness (keep dark items like black lane)
        v_channel = hsv[:, :, 2]
        _, dark_mask = cv2.threshold(v_channel, 70, 255, cv2.THRESH_BINARY_INV)

        # Combine
        binary = cv2.bitwise_and(low_sat_mask, dark_mask)

        # Clean up
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        # 3. Find Lane Center
        target_cx = self.find_lane_edge_scanline(binary)
        center_x = w // 2
        correction = 0.0

        if target_cx != -1:
            # PID Control
            error = center_x - target_cx
            p_term = self.kp * error
            d_term = self.kd * (error - self.last_error)
            self.last_error = error

            # Calculate and clip correction
            correction = p_term + d_term
            correction = np.clip(correction, -self.max_corr, self.max_corr)

            if self.enable_debug:
                draw_y = roi_h_start + (h - roi_h_start) // 2
                cv2.circle(debug_img, (target_cx, draw_y), 8, (0, 0, 255), -1) 
                cv2.line(debug_img, (center_x, roi_h_start), (center_x, h), (255, 0, 0), 2)
                cv2.line(debug_img, (center_x, draw_y), (target_cx, draw_y), (0, 255, 255), 3)
                cv2.putText(debug_img, f"Err:{error} Cmd:{correction:.3f}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        else:
            # No lane found
            self.last_error = 0.0
            correction = 0.0
            if self.enable_debug:
                cv2.putText(debug_img, "NO LANE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # 4. Publish Correction
        self.publish_correction(correction)

        if self.enable_debug:
            self.publish_debug_image(debug_img)

    def publish_correction(self, value):
        msg = Float32()
        msg.data = float(value)
        self.pub_correction.publish(msg)

    def publish_debug_image(self, cv_img):
        try:
            msg = self.bridge.cv2_to_imgmsg(cv_img, "bgr8")
            self.pub_debug_img.publish(msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Publish debug image error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()