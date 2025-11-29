# bev_obstacle_detector/mission_manager.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from rclpy.duration import Duration
from rclpy.time import Time as RclpyTime
import math # 【新增】用于计算距离

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import MarkerArray, Marker

import tf2_ros
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener, TransformException

# 导入 GoalConverter
from yolo_detector.goal_converter import GoalConverter 

class MissionManager(Node):
    
    STATE_IDLE = 0
    STATE_NAV_FINAL = 1
    STATE_NAV_BONUS = 2

    def __init__(self):
        super().__init__('mission_manager')

        # --- 1. 参数声明 ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('final_goal_topic', '/rviz_final_goal'),
                ('yolo_detection_topic', '/yolo_detections'), 
                ('nav_action_server', '/navigate_to_pose'),
                ('robot_base_frame', 'body'),
                ('global_frame', 'map'),
                ('final_goal_value', -1),
                ('min_forward_dist', 0.3),
                ('tf_timeout_seconds', 0.5),
                ('bonus_goal_timeout', 2.5),
                ('goal_republish_thresh', 0.15), # 【新增】防抖阈值 (米)
                ('miss_threshold_dist', 0.1),
                
                # IPM 参数
                ('bev_width', 640), ('bev_height', 480), ('world_width_m', 2.0),
                ('world_height_m', 3.0), ('origin_offset_x_m', 0.5), ('origin_offset_y_m', 1.0),

                # 调试参数
                ('marker_array_topic', '/debug/goal'),
                ('enable_debug', False),
            ]
        )
        
        # 参数获取
        final_goal_topic = self.get_parameter('final_goal_topic').value
        yolo_detection_topic = self.get_parameter('yolo_detection_topic').value
        nav_action_server = self.get_parameter('nav_action_server').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.FINAL_GOAL_VALUE = self.get_parameter('final_goal_value').value
        self.min_forward_dist = self.get_parameter('min_forward_dist').value
        self.tf_timeout_seconds = self.get_parameter('tf_timeout_seconds').value
        self.BONUS_GOAL_TIMEOUT_SEC = self.get_parameter('bonus_goal_timeout').value
        self.MISS_THRESHOLD_DIST = self.get_parameter('miss_threshold_dist').value
        self.GOAL_REPUBLISH_THRESH = self.get_parameter('goal_republish_thresh').value
        self.MARKER_ARRAY_TOPIC = self.get_parameter('marker_array_topic').value
        self.enable_debug = self.get_parameter('enable_debug').get_parameter_value().bool_value

        # 初始化 GoalConverter
        ipm_params = {
            'bev_width': self.get_parameter('bev_width').value,
            'bev_height': self.get_parameter('bev_height').value,
            'world_width_m': self.get_parameter('world_width_m').value,
            'world_height_m': self.get_parameter('world_height_m').value,
            'origin_offset_x_m': self.get_parameter('origin_offset_x_m').value,
            'origin_offset_y_m': self.get_parameter('origin_offset_y_m').value
        }
        self.goal_converter = GoalConverter(ipm_params)

        # --- 2. 状态管理 ---
        self.state = self.STATE_IDLE
        self.final_goal_pose = None
        
        # 当前正在追踪的目标 (Map Frame)
        self.current_active_pose_map = None 
        self.current_active_goal_value = self.FINAL_GOAL_VALUE
        self.current_active_goal_type = None
        self.current_active_goal_handle = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 【新增】当前 Bonus 任务开始时间
        self.active_bonus_start_time = None

        # --- 3. ROS 接口 ---
        self.create_subscription(PoseStamped, final_goal_topic, self.rviz_final_goal_callback, 10)
        self.create_subscription(Detection2DArray, yolo_detection_topic, self.yolo_detection_callback, 10)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, nav_action_server)

        if self.enable_debug:
            self.marker_pub = self.create_publisher(MarkerArray, self.MARKER_ARRAY_TOPIC, 10)
            self.get_logger().info(f"启用调试模式")

        # 监控定时器
        self.monitor_timer = self.create_timer(0.1, self.monitor_goal_status)

        self.get_logger().info(f"Mission Manager 已启动。防抖阈值: {self.GOAL_REPUBLISH_THRESH}m")

    # --------------------------------------------------------------------------
    # --- 辅助函数 ---
    # --------------------------------------------------------------------------
    def transform_pose(self, pose: PoseStamped, target_frame: str) -> PoseStamped or None:
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout_seconds)
            )
            return tf2_geometry_msgs.do_transform_pose(pose.pose, transform)
        except TransformException as e:
            self.get_logger().warn(f"TF 变换失败 ({self.robot_base_frame} -> {target_frame}): {e}")
            return None

    def calc_distance(self, pose1: PoseStamped, pose2: PoseStamped) -> float:
        """计算两个 Pose 之间的欧几里得距离"""
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return math.sqrt(dx*dx + dy*dy)
    # --------------------------------------------------------------------------
    # --- 辅助函数：Marker 可视化 ---
    # --------------------------------------------------------------------------
    def publish_current_goal_marker(self):
        """将当前活跃目标 (Final 或 Bonus) 发布到 RViz"""
        if not self.enable_debug:
            return

        marker_array = MarkerArray()
        
        # 1. 删除旧标记
        delete_marker = Marker(header=self.get_current_header(), ns="active_goal", action=Marker.DELETEALL)
        marker_array.markers.append(delete_marker)

        if self.current_active_pose_map is None:
            self.marker_pub.publish(marker_array)
            return

        # 2. 创建新的活动目标标记
        marker = Marker(header=self.get_current_header(), ns="active_goal", action=Marker.ADD)
        marker.id = 1
        
        # 标记类型和颜色
        if self.current_active_goal_type == 'FINAL':
            marker.type = Marker.ARROW
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 0.0, 1.0, 1.0 # 蓝色 (终点)
            text = "FINAL"
        else: # BONUS
            marker.type = Marker.SPHERE
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.5, 0.0, 1.0 # 橙色 (奖励)
            text = f"BONUS ({self.current_active_goal_value})"
            
        marker.pose = self.current_active_pose_map.pose
        marker.scale.x, marker.scale.y, marker.scale.z = 0.5, 0.5, 0.5
        marker.lifetime = Duration(seconds=0.5).to_msg() # 频繁刷新
        marker_array.markers.append(marker)

        # 3. 创建文本标记
        text_marker = Marker(header=self.get_current_header(), ns="active_goal_label", action=Marker.ADD)
        text_marker.id = 2
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.pose = self.current_active_pose_map.pose
        text_marker.pose.position.z += 0.6
        text_marker.text = text
        text_marker.scale.z = 0.3
        text_marker.color.r, text_marker.color.g, text_marker.color.b, text_marker.color.a = 1.0, 1.0, 1.0, 1.0
        text_marker.lifetime = Duration(seconds=0.5).to_msg()
        marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)

    def get_current_header(self):
        """获取带有当前时间戳和全局 Frame ID 的 Header"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.global_frame
        return header
    # --------------------------------------------------------------------------
    # --- 核心回调 ---
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
        if self.state == self.STATE_IDLE or not msg.detections:
            return

        # 1. 在 *本帧* 中找最佳
        best_detection, max_value = self.goal_converter.find_best_detection(msg.detections)
        
        if best_detection is None:
            return

        # 2. 转换坐标 (Pixel -> Body)
        u = best_detection.bbox.center.position.x
        v = best_detection.bbox.center.position.y
        robot_x, robot_y, _ = self.goal_converter.pixel_to_body_coordinates(u, v)

        # 3. 构造 Body Pose
        pose_in_body = PoseStamped()
        pose_in_body.header.frame_id = self.robot_base_frame
        pose_in_body.header.stamp = msg.header.stamp
        pose_in_body.pose.position = Point(x=robot_x, y=robot_y, z=0.0)
        pose_in_body.pose.orientation.w = 1.0

        # 4. 几何过滤：是否太近？
        if robot_x < self.min_forward_dist:
            self.get_logger().info(f"[检测回调]：目标距离过近 ({robot_x:.2f}m)，不发送。")
            return

        # 5. 转换到 Map Frame
        pose_in_map_raw = self.transform_pose(pose_in_body, self.global_frame)
        if pose_in_map_raw is None:
            return
            
        pose_in_map = PoseStamped()
        pose_in_map.header.frame_id = self.global_frame
        pose_in_map.header.stamp = self.get_clock().now().to_msg()
        pose_in_map.pose = pose_in_map_raw

        # --- 6. 【核心修改】决策与防抖逻辑 ---
        
        should_send_goal = False

        # 情况 A: 发现更高价值目标 -> 必须发送
        if max_value > self.current_active_goal_value:
            self.get_logger().info(f"决策：发现更高价值目标 {max_value}，执行覆盖。")
            should_send_goal = True
            
        # 情况 B: 价值相同 -> 检查位置变化 (防抖)
        elif max_value == self.current_active_goal_value:
            # 只有当当前正在执行 Bonus 任务时才需要防抖
            if self.current_active_goal_type == 'BONUS' and self.current_active_pose_map is not None:
                # 计算新旧目标的距离
                dist = self.calc_distance(pose_in_map, self.current_active_pose_map)
                
                if dist > self.GOAL_REPUBLISH_THRESH:
                    self.get_logger().info(f"决策：目标位置修正 (偏离 {dist:.2f}m)，更新目标。")
                    should_send_goal = True
                else:
                    # 距离变化很小，忽略本次检测，避免 Nav2 重规划
                    self.get_logger().debug(f"防抖：目标位置稳定 (偏离 {dist:.2f}m)，跳过。")
                    should_send_goal = False
            else:
                # 这种边缘情况（价值相同但类型不同？）通常直接发送
                should_send_goal = True

        # 执行发送
        if should_send_goal:
            self.get_logger().info(f"[检测回调]：尝试发送新目标 {pose_in_map.pose} 价值{max_value}。")
            self.send_new_goal(pose_in_map, max_value, 'BONUS')
            self.state = self.STATE_NAV_BONUS
            # 【新增】目标更新时，立即刷新可视化
            if self.enable_debug:
                self.publish_current_goal_marker()

    # --------------------------------------------------------------------------
    # --- 实时监控逻辑 ---
    # --------------------------------------------------------------------------
    def monitor_goal_status(self):
        if self.state != self.STATE_NAV_BONUS or self.current_active_pose_map is None:
            return

        # --- 超时检查 ---
        if self.active_bonus_start_time is not None:
            elapsed = (self.get_clock().now() - self.active_bonus_start_time).nanoseconds * 1e-9
            if elapsed > self.BONUS_GOAL_TIMEOUT_SEC:
                self.get_logger().warn(f"加分点任务超时 ({elapsed:.1f}s > {self.BONUS_GOAL_TIMEOUT_SEC}s)，放弃并恢复终点。")
                self.recover_to_final()
                return

        # --- 位置检查 ---
        pose_in_body_now = self.transform_pose(self.current_active_pose_map, self.robot_base_frame)
        if pose_in_body_now is None:
            return

        dist_x = pose_in_body_now.position.x
        
        if dist_x < self.MISS_THRESHOLD_DIST:
            self.get_logger().warn(f"加分点已到达或错过 (X={dist_x:.2f}m)，放弃并恢复终点。")
            self.recover_to_final()
        
        # 【新增】如果开启了调试，持续刷新目标标记
        if self.enable_debug and self.state != self.STATE_IDLE:
             self.publish_current_goal_marker()

    def recover_to_final(self):
        if self.final_goal_pose:
            self.get_logger().info(">>> 恢复导航至终点...")
            self.send_new_goal(self.final_goal_pose, self.FINAL_GOAL_VALUE, 'FINAL')
            self.state = self.STATE_NAV_FINAL
            # 【新增】目标被清除时，刷新可视化
            if self.enable_debug:
                self.publish_current_goal_marker()

        else:
            self.get_logger().error("尝试恢复但无终点信息！")
            self.state = self.STATE_IDLE

    # --------------------------------------------------------------------------
    # --- Action Client 逻辑 ---
    # --------------------------------------------------------------------------
    def send_new_goal(self, pose_stamped: PoseStamped, value: int, goal_type: str):
        if self.current_active_goal_handle is not None and \
           self.current_active_goal_handle.status == GoalStatus.STATUS_EXECUTING:
            self.current_active_goal_handle.cancel_goal_async()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        self.current_active_goal_value = value
        self.current_active_goal_type = goal_type
        self.current_active_pose_map = pose_stamped 

        # 如果是 Bonus 目标，重置开始时间
        if goal_type == 'BONUS':
            self.active_bonus_start_time = self.get_clock().now()
        else:
            self.active_bonus_start_time = None 

        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 拒绝了目标')
            return
        self.current_active_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.current_active_goal_type == 'FINAL':
                self.get_logger().info("TASK FINISHED: 到达终点。")
                self.state = self.STATE_IDLE
                self.final_goal_pose = None
                # 【新增】任务完成时，清除标记
                if self.enable_debug:
                    self.publish_current_goal_marker()
            elif self.current_active_goal_type == 'BONUS':
                self.get_logger().info("Bonus Reached: 到达加分点。")
                self.recover_to_final()
        
        elif status == GoalStatus.STATUS_CANCELED:
            pass 
            
        else: # ABORTED / LOST
            pass
            # self.get_logger().warn(f"Nav2 失败 ({status})，尝试恢复终点...")
            # if self.final_goal_pose:
                # self.recover_to_final()

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()