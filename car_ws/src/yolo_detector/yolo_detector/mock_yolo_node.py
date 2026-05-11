import rclpy
from rclpy.node import Node
from rclpy.time import Time
import math
import random

# 消息依赖
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import TransformStamped

# TF 依赖
from tf2_ros import Buffer, TransformListener, TransformException

class MockYoloPublisher(Node):

    def __init__(self):
        super().__init__('mock_yolo_publisher')

        # --- 参数设置 ---
        self.declare_parameter('trigger_distance', 0.4) # 每 0.4 米触发一次
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        
        self.TRIGGER_DIST = self.get_parameter('trigger_distance').value
        self.IMG_W = self.get_parameter('image_width').value
        self.IMG_H = self.get_parameter('image_height').value

        # --- 发布者 ---
        self.yolo_pub = self.create_publisher(
            Detection2DArray, 
            '/yolo_detections', 
            10
        )

        # --- TF 监听 (用于计算里程) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- 内部状态 ---
        self.last_pose_x = None
        self.last_pose_y = None
        self.seq_id = 0 

        # 启动时就发布一次
        self.publish_fake_detections()
        self.get_logger().info("Mock YOLO Node 启动时已立即发布第一条数据。")

        # 定时检查位置 (10Hz)
        self.timer = self.create_timer(0.1, self.check_position_and_publish)
        
        self.get_logger().info(f"Mock YOLO Node 已启动。每隔 {self.TRIGGER_DIST}m 发布一次并排数据。")

    def check_position_and_publish(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 
                'body', 
                rclpy.time.Time()
            )
            
            curr_x = trans.transform.translation.x
            curr_y = trans.transform.translation.y

            if self.last_pose_x is None:
                self.last_pose_x = curr_x
                self.last_pose_y = curr_y
                return

            dist = math.sqrt((curr_x - self.last_pose_x)**2 + (curr_y - self.last_pose_y)**2)

            if dist >= self.TRIGGER_DIST:
                self.get_logger().info(f"已行驶 {dist:.2f}m，发布虚假 YOLO 数据...")
                self.publish_fake_detections()
                self.last_pose_x = curr_x
                self.last_pose_y = curr_y

        except TransformException as e:
            pass

    def publish_fake_detections(self):
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_color_optical_frame"

        # 模拟两个数字 ID (1-9 循环)
        # 让两个数字不同，以便测试决策逻辑
        val_left = (self.seq_id % 9) + 1
        val_right = ((self.seq_id + 4) % 9) + 1 
        self.seq_id += 1

        # 设定统一的距离 (Y轴坐标) 和大小
        # 假设图像高 480，Y=300 大约在中近距离
        common_y = 300.0 
        common_size = 60.0

        # --- 检测 1: 左侧数字 ---
        det1 = Detection2D()
        # X 轴：1/4 处 (左侧)
        det1.bbox.center.position.x = float(self.IMG_W / 4) 
        det1.bbox.center.position.y = common_y
        det1.bbox.size_x = common_size
        det1.bbox.size_y = common_size
        
        hyp1 = ObjectHypothesisWithPose()
        hyp1.hypothesis.class_id = str(val_left) 
        hyp1.hypothesis.score = 0.95
        det1.results.append(hyp1)
        msg.detections.append(det1)

        # --- 检测 2: 右侧数字 ---
        det2 = Detection2D()
        # X 轴：3/4 处 (右侧)
        det2.bbox.center.position.x = float(self.IMG_W * 3 / 4)
        det2.bbox.center.position.y = common_y
        det2.bbox.size_x = common_size
        det2.bbox.size_y = common_size
        
        hyp2 = ObjectHypothesisWithPose()
        hyp2.hypothesis.class_id = str(val_right)
        hyp2.hypothesis.score = 0.92
        det2.results.append(hyp2)
        msg.detections.append(det2)

        self.yolo_pub.publish(msg)
        self.get_logger().info(f"发送并排目标: [左: {val_left}, 右: {val_right}]")

def main(args=None):
    rclpy.init(args=args)
    node = MockYoloPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()