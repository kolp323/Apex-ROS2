import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

# 不再需要导入 tf2_ros 和 TransformException

import numpy as np

class PointCloudMerger(Node):
    
    def __init__(self):
        super().__init__('pointcloud_merger_node')
        
        self.get_logger().info('PointCloudMerger Node has started.')
        
        # --- 1. 声明和获取参数 ---
        self.declare_parameters(
            namespace='',
            parameters=[
                # 注意：Lidar 话题已更新为你提供的 /cloud_registered
                ('input_topic_lidar', '/cloud_registered'), 
                ('input_topic_bev', '/bev/obstacles'),
                ('output_topic_fused', '/fused_pointcloud'),
                # 目标坐标系已改为 body
                ('target_frame', 'body'), 
                # TF 参数虽然不再使用，但为了防止外部调用，保留声明
                ('tf_timeout_seconds', 0.05),
                ('fusion_time_tolerance', 0.01)
            ]
        )

        self.input_topic_lidar = self.get_parameter('input_topic_lidar').value
        self.input_topic_bev = self.get_parameter('input_topic_bev').value
        self.output_topic_fused = self.get_parameter('output_topic_fused').value
        self.target_frame = self.get_parameter('target_frame').value
        self.tf_timeout_seconds = self.get_parameter('tf_timeout_seconds').value
        self.fusion_time_tolerance = self.get_parameter('fusion_time_tolerance').value

        # --- 移除 TF 监听器初始化 ---
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- 2. 初始化点云存储 ---
        self.bev_pc_msg = None
        self.lidar_pc_msg = None
        
        # --- 3. 初始化 ROS 接口 ---
        
        # 订阅 BEV 障碍物点云 (已在 body 坐标系)
        self.create_subscription(
            PointCloud2, 
            self.input_topic_bev,
            self.bev_callback, 
            10
        )
        
        # 订阅 Lidar 原始点云 (已在 body 坐标系)
        self.create_subscription(
            PointCloud2, 
            self.input_topic_lidar,
            self.lidar_callback, 
            10
        )
        
        # 发布融合后的点云
        self.fused_publisher = self.create_publisher(
            PointCloud2, 
            self.output_topic_fused,
            10
        )
        self.get_logger().info(f"Fusion Node publishing to: {self.output_topic_fused} in frame: {self.target_frame}")

    # --- 移除 transform_pointcloud 函数 ---
    # 由于不需要 TF 变换，此函数不再需要。
    
    def bev_callback(self, msg: PointCloud2):
        """处理 BEV 点云的回调函数"""
        # 确保话题的 frame_id 与目标 frame 一致，否则发出警告
        if msg.header.frame_id != self.target_frame:
             self.get_logger().warn(f"BEV点云的Frame ID ({msg.header.frame_id}) 与目标 Frame ID ({self.target_frame}) 不匹配！")
        self.bev_pc_msg = msg
        self.try_fuse_pointclouds()

    def lidar_callback(self, msg: PointCloud2):
        """处理 Lidar 点云的回调函数"""
        if msg.header.frame_id != self.target_frame:
             self.get_logger().warn(f"Lidar点云的Frame ID ({msg.header.frame_id}) 与目标 Frame ID ({self.target_frame}) 不匹配！")
        self.lidar_pc_msg = msg
        self.try_fuse_pointclouds()

    def try_fuse_pointclouds(self):
        """尝试融合两个点云 (无需 TF 变换)"""
        # 确保两个传感器都有数据
        if self.bev_pc_msg is None or self.lidar_pc_msg is None:
            return

        # 优先使用 LiDAR 的时间戳作为融合点云的基准时间
        fusion_stamp = self.lidar_pc_msg.header.stamp
        
        # 1. 提取两个点云的 (x, y, z) 坐标
        # **直接使用原始消息，因为它们已在同一坐标系 (body)**
        bev_points = self.extract_points(self.bev_pc_msg)
        lidar_points = self.extract_points(self.lidar_pc_msg)
        
        if len(lidar_points) == 0:
            self.get_logger().warn("Lidar点云为空，跳过融合。")
            return

        # 2. 合并点云 (NumPy 数组)
        if len(bev_points) > 0:
            fused_points_np = np.concatenate((lidar_points, bev_points), axis=0)
        else:
            fused_points_np = lidar_points
        
        # 3. 创建并发布融合后的 PointCloud2 消息
        fused_header = Header(
            stamp=fusion_stamp, 
            frame_id=self.target_frame  # <--- 使用配置的 'body' 作为目标 frame_id
        )
        
        # 定义点云的字段 (只包含 x, y, z)
        fields = [
            point_cloud2.PointField(name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1)
        ]
        
        fused_pc_msg = point_cloud2.create_cloud(
            fused_header, 
            fields, 
            fused_points_np.tolist()
        )
        
        self.fused_publisher.publish(fused_pc_msg)
        
        # 融合完成后，清空 BEV 点云，等待下一个新的 BEV 帧
        # Lidar 点云保留，直到下一个 Lidar 帧到达
        self.bev_pc_msg = None


    def extract_points(self, pc_msg: PointCloud2) -> np.ndarray:
        """从 PointCloud2 消息中提取 (x, y, z) 数组"""
        points_generator = point_cloud2.read_points(pc_msg, field_names=('x', 'y', 'z'), skip_nans=True)
        points_list = list(points_generator)
        
        if not points_list:
            return np.empty((0, 3), dtype=np.float32)
            
        return np.array(points_list, dtype=np.float32)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()