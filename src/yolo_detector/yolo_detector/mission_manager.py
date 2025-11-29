import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from rclpy.duration import Duration
from rclpy.time import Time as RclpyTime

from geometry_msgs.msg import PoseStamped, Point
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from vision_msgs.msg import Detection2DArray

import tf2_ros
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener, TransformException

# --- 导入 GoalConverter (确保该文件在同一包下或PYTHONPATH中) ---
# 如果是在同一个包内，通常需要: from .goal_converter import GoalConverter
from yolo_detector.goal_converter import GoalConverter 

class MissionManager(Node):
    
    STATE_IDLE = 0
    STATE_NAV_FINAL = 1
    STATE_NAV_BONUS = 2

    def __init__(self):
        super().__init__('mission_manager')

        # --- 1. 声明参数 (包括 IPM 参数) ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('final_goal_topic', '/rviz_final_goal'),
                ('yolo_detection_topic', '/yolo_detections'), # 直接订阅 YOLO
                ('nav_action_server', '/navigate_to_pose'),
                ('robot_base_frame', 'body'),
                ('global_frame', 'map'),
                ('final_goal_value', -1),
                ('min_forward_dist', 0.3),
                ('tf_timeout_seconds', 0.5),
                ('bonus_goal_timeout', 2.5),
                
                # IPM 参数
                ('bev_width', 640),
                ('bev_height', 480),
                ('world_width_m', 2.0),
                ('world_height_m', 3.0),
                ('origin_offset_x_m', 0.5),
                ('origin_offset_y_m', 1.0)
            ]
        )
        
        # --- 2. 获取参数 ---
        final_goal_topic = self.get_parameter('final_goal_topic').value
        yolo_detection_topic = self.get_parameter('yolo_detection_topic').value
        nav_action_server = self.get_parameter('nav_action_server').value
        
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        
        self.FINAL_GOAL_VALUE = self.get_parameter('final_goal_value').value
        self.min_forward_dist = self.get_parameter('min_forward_dist').value
        self.tf_timeout_seconds = self.get_parameter('tf_timeout_seconds').value
        self.BONUS_GOAL_TIMEOUT_SEC = self.get_parameter('bonus_goal_timeout').value

        # --- 3. 初始化 GoalConverter ---
        # 从参数服务器构建字典
        ipm_params = {
            'bev_width': self.get_parameter('bev_width').value,
            'bev_height': self.get_parameter('bev_height').value,
            'world_width_m': self.get_parameter('world_width_m').value,
            'world_height_m': self.get_parameter('world_height_m').value,
            'origin_offset_x_m': self.get_parameter('origin_offset_x_m').value,
            'origin_offset_y_m': self.get_parameter('origin_offset_y_m').value
        }
        self.goal_converter = GoalConverter(ipm_params)
        self.get_logger().info("GoalConverter 已初始化。")

        # --- 4. 内部状态与 TF ---
        self.state = self.STATE_IDLE
        self.final_goal_pose = None
        self.current_active_goal_handle = None
        self.current_active_goal_value = self.FINAL_GOAL_VALUE
        self.current_active_goal_type = None
        self.tracked_bonus_goals = {} # { goal_id: {pose: Pose, value: int, first_seen: Time} }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- 5. ROS 接口 ---
        
        # (Input 1) 订阅 RViz 终点
        self.create_subscription(
            PoseStamped,
            final_goal_topic,
            self.rviz_final_goal_callback,
            10
        )
        
        # (Input 2) 直接订阅 YOLO 原始检测
        self.create_subscription(
            Detection2DArray,
            yolo_detection_topic,
            self.yolo_detection_callback,
            10
        )
        
        # (Output) Nav2 Action Client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, nav_action_server)

        self.get_logger().info("Mission Manager (Integrated) 已启动。")

    # --------------------------------------------------------------------------
    # --- 辅助函数：TF 几何检查 (确保目标在前方) ---
    # --------------------------------------------------------------------------
    def check_goal_is_reachable_and_transform(self, pose_in_body: PoseStamped) -> PoseStamped or None:
        """
        1. 检查 body 系下的目标是否在前方 min_forward_dist 之外。
        2. 如果有效，将其转换到 map 系并返回。
        """
        # 1. 几何检查 (直接在 Body 系下做，无需 TF)
        if pose_in_body.pose.position.x < self.min_forward_dist:
            # self.get_logger().debug(f"目标太近或在身后 (x={pose_in_body.pose.position.x:.2f}), 忽略。")
            return None

        # 2. 转换到 Map 系 (用于追踪和导航)
        try:
            # 查找 Body -> Map 的变换
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                rclpy.time.Time(), # 获取最新变换
                timeout=Duration(seconds=self.tf_timeout_seconds)
            )
            pose_in_map = tf2_geometry_msgs.do_transform_pose(pose_in_body.pose, transform)
            
            result_pose = PoseStamped()
            result_pose.header.frame_id = self.global_frame
            result_pose.header.stamp = self.get_clock().now().to_msg()
            result_pose.pose = pose_in_map
            return result_pose
            
        except TransformException as e:
            self.get_logger().warn(f"TF 变换失败 (Body -> Map): {e}")
            return None

    # --------------------------------------------------------------------------
    # --- 回调函数 ---
    # --------------------------------------------------------------------------

    def rviz_final_goal_callback(self, msg: PoseStamped):
        if self.state != self.STATE_IDLE:
             self.get_logger().warn("比赛进行中，忽略新终点。")
             return
        self.get_logger().info("收到终点，开始任务！")
        self.final_goal_pose = msg
        self.send_new_goal(self.final_goal_pose, self.FINAL_GOAL_VALUE, 'FINAL')
        self.state = self.STATE_NAV_FINAL

    def yolo_detection_callback(self, msg: Detection2DArray):
        """处理 YOLO 原始检测数据"""
        if self.state == self.STATE_IDLE or not msg.detections:
            return

        # 1. 使用 GoalConverter 找到最佳检测
        best_detection, max_value = self.goal_converter.find_best_detection(msg.detections)
        
        if best_detection is None:
            return

        # 2. 使用 GoalConverter 计算 Body 系坐标
        u = best_detection.bbox.center.position.x
        v = best_detection.bbox.center.position.y
        robot_x, robot_y, _ = self.goal_converter.pixel_to_body_coordinates(u, v)

        # 3. 构建 Body 系下的 PoseStamped
        pose_in_body = PoseStamped()
        pose_in_body.header.frame_id = self.robot_base_frame
        pose_in_body.header.stamp = msg.header.stamp
        pose_in_body.pose.position = Point(x=robot_x, y=robot_y, z=0.0)
        pose_in_body.pose.orientation.w = 1.0

        # 4. 执行“过近检查”并转换到 Map 系
        pose_in_map = self.check_goal_is_reachable_and_transform(pose_in_body)
        
        if pose_in_map is None:
            return # 目标无效（太近或TF失败）

        # --- 更新追踪列表、超时检查、决策发送 ---
        
        now = self.get_clock().now()
        goal_id = max_value # 简化：ID即价值

        # 更新/添加追踪记录
        if goal_id not in self.tracked_bonus_goals:
            self.get_logger().info(f"发现新目标 (Value: {goal_id})")
            self.tracked_bonus_goals[goal_id] = {
                'pose': pose_in_map, 'value': goal_id, 'first_seen': now
            }
        else:
            # 更新位置（因为小车动了，或者检测更准了）
            self.tracked_bonus_goals[goal_id]['pose'] = pose_in_map

        # 超时检查
        self.cleanup_stale_goals(now)

        # 决策：是否切换目标？
        if max_value > self.current_active_goal_value:
            # 再次确认这个最高分目标是否还在追踪列表中（未超时）
            if max_value in self.tracked_bonus_goals:
                self.get_logger().info(f"决策：切换到更高价值目标 {max_value}")
                self.send_new_goal(pose_in_map, max_value, 'BONUS')
                self.state = self.STATE_NAV_BONUS

    def cleanup_stale_goals(self, now):
        ids_to_remove = []
        for gid, data in self.tracked_bonus_goals.items():
            elapsed = (now - data['first_seen']).nanoseconds * 1e-9
            if elapsed > self.BONUS_GOAL_TIMEOUT_SEC:
                # self.get_logger().info(f"目标 {gid} 超时，移除。")
                ids_to_remove.append(gid)
        for gid in ids_to_remove:
            del self.tracked_bonus_goals[gid]

    # --------------------------------------------------------------------------
    # --- Action Client 逻辑 (保持不变) ---
    # --------------------------------------------------------------------------
    def send_new_goal(self, pose_stamped: PoseStamped, value: int, goal_type: str):
        if self.current_active_goal_handle is not None and \
           self.current_active_goal_handle.status == GoalStatus.STATUS_EXECUTING:
            self.current_active_goal_handle.cancel_goal_async()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        self.current_active_goal_value = value
        self.current_active_goal_type = goal_type
        
        self.get_logger().info(f"发送 Nav2 目标 (Type: {goal_type}, Value: {value})")
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被 Nav2 拒绝')
            return
        self.current_active_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if self.current_active_goal_type == 'FINAL':
                self.get_logger().info("任务完成：到达终点！")
                self.state = self.STATE_IDLE
                self.final_goal_pose = None
            elif self.current_active_goal_type == 'BONUS':
                self.get_logger().info(f"任务完成：收集到加分点 {self.current_active_goal_value}")
                # 移除已收集的目标
                if self.current_active_goal_value in self.tracked_bonus_goals:
                    del self.tracked_bonus_goals[self.current_active_goal_value]
                # 恢复终点
                self.get_logger().info("恢复导航至终点...")
                self.send_new_goal(self.final_goal_pose, self.FINAL_GOAL_VALUE, 'FINAL')
                self.state = self.STATE_NAV_FINAL
        else:
            self.get_logger().warn(f"导航异常 (Status: {result.status})，尝试恢复终点...")
            if self.final_goal_pose:
                self.send_new_goal(self.final_goal_pose, self.FINAL_GOAL_VALUE, 'FINAL')
                self.state = self.STATE_NAV_FINAL

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()