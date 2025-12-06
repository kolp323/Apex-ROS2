# bev_obstacle_detector/mission_manager.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
import math 

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateThroughPoses
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import MarkerArray, Marker

import tf2_ros
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener, TransformException

from yolo_detector.goal_converter import GoalConverter 

class MissionManager(Node):
    
    STATE_IDLE = 0
    STATE_NAV_FINAL = 1
    STATE_NAV_BONUS = 2

    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameters(
            namespace='',
            parameters=[
                # --- 【修改】固定坐标参数改为 [x, y, z, qx, qy, qz, qw] ---
                ('goal_a', [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]), 
                ('goal_b', [2.0, 0.0, 0.0, 0.0, 0.0, 0.707, 0.707]), # 示例：绕Z轴旋转90度
                
                # 到达判定阈值
                ('goal_reached_dist', 0.2),

                ('yolo_detection_topic', '/yolo_detections'), 
                ('robot_base_frame', 'body'),
                ('global_frame', 'map'),
                ('final_goal_value', -1),
                ('min_forward_dist', 0.3),
                ('tf_timeout_seconds', 0.5),
                ('bonus_goal_timeout', 2.5),
                ('goal_republish_thresh', 0.15), 
                ('miss_threshold_dist', 0.1),
                
                ('bev_width', 640), ('bev_height', 480), ('world_width_m', 2.0),
                ('world_height_m', 3.0), ('origin_offset_x_m', 0.5), ('origin_offset_y_m', 1.0),

                ('marker_array_topic', '/debug/goal'),
                ('enable_debug', True), 
            ]
        )
        
        # 参数获取
        self.goal_a_coords = self.get_parameter('goal_a').value
        self.goal_b_coords = self.get_parameter('goal_b').value
        
        # 简单校验参数长度
        if len(self.goal_a_coords) != 7 or len(self.goal_b_coords) != 7:
            self.get_logger().error("参数 goal_a 或 goal_b 格式错误！必须为 [x, y, z, qx, qy, qz, qw] 7位数组")

        self.GOAL_REACHED_DIST = self.get_parameter('goal_reached_dist').value 

        yolo_detection_topic = self.get_parameter('yolo_detection_topic').value
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

        ipm_params = {
            'bev_width': self.get_parameter('bev_width').value,
            'bev_height': self.get_parameter('bev_height').value,
            'world_width_m': self.get_parameter('world_width_m').value,
            'world_height_m': self.get_parameter('world_height_m').value,
            'origin_offset_x_m': self.get_parameter('origin_offset_x_m').value,
            'origin_offset_y_m': self.get_parameter('origin_offset_y_m').value
        }
        self.goal_converter = GoalConverter(ipm_params)

        self.state = self.STATE_IDLE
        self.final_goal_queue = [] 
        self.current_goal_index = 0 
        self.final_goal_pose = None 
        self.load_fixed_goals()

        self.current_active_pose_map = None 
        self.current_active_goal_value = self.FINAL_GOAL_VALUE
        self.current_active_goal_type = None
        self.current_active_goal_handle = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.active_bonus_start_time = None

        self.create_subscription(Detection2DArray, yolo_detection_topic, self.yolo_detection_callback, 10)
        self.nav_to_pose_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        if self.enable_debug:
            self.marker_pub = self.create_publisher(MarkerArray, self.MARKER_ARRAY_TOPIC, 10)

        self.monitor_timer = self.create_timer(0.1, self.monitor_goal_status)
        self.start_timer = self.create_timer(2.0, self.start_mission_callback)

        self.get_logger().info(f"Mission Manager 启动。主动切换阈值: {self.GOAL_REACHED_DIST}m")

    # --------------------------------------------------------------------------
    # --- 辅助函数 ---
    # --------------------------------------------------------------------------
    def create_pose_stamped(self, pose_array):
        """
        解析 7 位数组 [x, y, z, qx, qy, qz, qw]
        """
        p = PoseStamped()
        p.header.frame_id = self.global_frame
        p.header.stamp = self.get_clock().now().to_msg()
        
        # 位置
        p.pose.position.x = float(pose_array[0])
        p.pose.position.y = float(pose_array[1])
        p.pose.position.z = float(pose_array[2])
        
        # 四元数
        p.pose.orientation.x = float(pose_array[3])
        p.pose.orientation.y = float(pose_array[4])
        p.pose.orientation.z = float(pose_array[5])
        p.pose.orientation.w = float(pose_array[6])
        
        return p

    def load_fixed_goals(self):
        # 【修改】直接传入数组，不再解包
        goal_a = self.create_pose_stamped(self.goal_a_coords)
        goal_b = self.create_pose_stamped(self.goal_b_coords)
        self.final_goal_queue = [goal_a, goal_b]

    def start_mission_callback(self):
        self.start_timer.cancel()
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 Action Server 连接超时！")
            return
        self.get_logger().info(">>> 自动启动任务：前往目标 A ...")
        self.current_goal_index = 0
        self.final_goal_pose = self.final_goal_queue[0]
        self.send_new_goal(self.final_goal_pose, self.FINAL_GOAL_VALUE, 'FINAL')
        self.state = self.STATE_NAV_FINAL
        if self.enable_debug:
            self.publish_current_goal_marker()

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
            return None

    def calc_distance(self, pose1: PoseStamped, pose2: PoseStamped) -> float:
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return math.sqrt(dx*dx + dy*dy)

    # --- 获取机器人当前在 Map 下的位姿 ---
    def get_robot_pose(self) -> PoseStamped or None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout_seconds)
            )
            p = PoseStamped()
            p.header.frame_id = self.global_frame
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x = transform.transform.translation.x
            p.pose.position.y = transform.transform.translation.y
            p.pose.position.z = transform.transform.translation.z
            p.pose.orientation = transform.transform.rotation
            return p
        except TransformException:
            return None

    # --- 封装切换到 B 的逻辑 ---
    def switch_to_phase_b(self):
        # 只有在阶段 A (index=0) 时才允许切换，防止重复调用
        if self.current_goal_index == 0:
            self.get_logger().info(">>> 切换至第二阶段：目标 B")
            self.current_goal_index = 1
            self.final_goal_pose = self.final_goal_queue[1]
            self.send_new_goal(self.final_goal_pose, self.FINAL_GOAL_VALUE, 'FINAL')
            self.state = self.STATE_NAV_FINAL
            if self.enable_debug:
                self.publish_current_goal_marker()

    # --------------------------------------------------------------------------
    # --- 监控逻辑 ---
    # --------------------------------------------------------------------------
    def monitor_goal_status(self):
        # 0. 基础检查
        if self.state == self.STATE_IDLE:
            return

        # 获取机器人当前位置
        robot_pose = self.get_robot_pose()
        if robot_pose is None: return

        # --- 1. Final 阶段的监控逻辑 ---
        if self.state == self.STATE_NAV_FINAL:
            # 仅在去往目标 A (index=0) 时检查距离
            if self.current_goal_index == 0 and self.final_goal_pose is not None:
                dist = self.calc_distance(robot_pose, self.final_goal_pose)
                # 如果距离小于阈值，且 Nav2 还没反馈成功，则强制切换
                if dist < self.GOAL_REACHED_DIST:
                    self.get_logger().info(f"[主动检测] 距离目标 A 仅 {dist:.2f}m < {self.GOAL_REACHED_DIST}m，强制切换。")
                    self.switch_to_phase_b()
            
            # 去往目标 B (index=1) 时，不进行此检查（由 Nav2 决定何时停止）

        # --- 2. Bonus 阶段的监控逻辑 ---
        elif self.state == self.STATE_NAV_BONUS and self.current_active_pose_map is not None:
            # 超时检查
            if self.active_bonus_start_time is not None:
                elapsed = (self.get_clock().now() - self.active_bonus_start_time).nanoseconds * 1e-9
                if elapsed > self.BONUS_GOAL_TIMEOUT_SEC:
                    self.get_logger().warn(f"Bonus 超时，恢复终点。")
                    self.recover_to_final()
                    return

            # 位置检查 (检查是否到达加分点)
            dist_bonus = self.calc_distance(robot_pose, self.current_active_pose_map)
            if dist_bonus < self.MISS_THRESHOLD_DIST:
                self.get_logger().info(f"Bonus 到达 (Dist={dist_bonus:.2f}m)，恢复终点。")
                self.recover_to_final()
        
        # 调试可视化
        if self.enable_debug:
             self.publish_current_goal_marker()

    # --------------------------------------------------------------------------
    # --- 其他回调 ---
    # --------------------------------------------------------------------------
    def yolo_detection_callback(self, msg: Detection2DArray):
        if self.state == self.STATE_IDLE or not msg.detections: return
        best_detection, max_value = self.goal_converter.find_best_detection(msg.detections)
        if best_detection is None: return
        u = best_detection.bbox.center.position.x
        v = best_detection.bbox.center.position.y
        robot_x, robot_y, _ = self.goal_converter.pixel_to_body_coordinates(u, v)
        if robot_x < self.min_forward_dist: return

        # 构造 Body Pose 并转换
        pose_in_body = PoseStamped()
        pose_in_body.header.frame_id = self.robot_base_frame
        pose_in_body.header.stamp = msg.header.stamp
        pose_in_body.pose.position = Point(x=robot_x, y=robot_y, z=0.0)
        pose_in_body.pose.orientation.w = 1.0
        
        pose_in_map_raw = self.transform_pose(pose_in_body, self.global_frame)
        if pose_in_map_raw is None: return
        pose_in_map = PoseStamped()
        pose_in_map.header.frame_id = self.global_frame
        pose_in_map.header.stamp = self.get_clock().now().to_msg()
        pose_in_map.pose = pose_in_map_raw

        should_send_goal = False
        if max_value > self.current_active_goal_value:
            should_send_goal = True
        elif max_value == self.current_active_goal_value:
            if self.current_active_goal_type == 'BONUS' and self.current_active_pose_map is not None:
                dist = self.calc_distance(pose_in_map, self.current_active_pose_map)
                if dist > self.GOAL_REPUBLISH_THRESH:
                    should_send_goal = True

        if should_send_goal:
            self.get_logger().info(f"[检测] 发送新目标 Bonus 价值 {max_value}")
            self.send_new_goal(pose_in_map, max_value, 'BONUS')
            self.state = self.STATE_NAV_BONUS
            if self.enable_debug:
                self.publish_current_goal_marker()

    def recover_to_final(self):
        if self.final_goal_pose:
            phase_name = "A" if self.current_goal_index == 0 else "B"
            self.get_logger().info(f">>> 恢复至终点 {phase_name}")
            self.send_new_goal(self.final_goal_pose, self.FINAL_GOAL_VALUE, 'FINAL')
            self.state = self.STATE_NAV_FINAL
            if self.enable_debug:
                self.publish_current_goal_marker()
        else:
            self.state = self.STATE_IDLE

    def send_new_goal(self, pose_stamped: PoseStamped, value: int, goal_type: str):
        if self.current_active_goal_handle is not None and \
           self.current_active_goal_handle.status == GoalStatus.STATUS_EXECUTING:
            self.current_active_goal_handle.cancel_goal_async()

        self.current_active_goal_value = value
        self.current_active_goal_type = goal_type
        self.current_active_pose_map = pose_stamped 

        if goal_type == 'BONUS':
            self.active_bonus_start_time = self.get_clock().now()
        else:
            self.active_bonus_start_time = None 

        goal_msg = NavigateThroughPoses.Goal()
        if goal_type == 'BONUS' and self.final_goal_pose is not None:
            goal_msg.poses = [pose_stamped, self.final_goal_pose]
        else:
            goal_msg.poses = [pose_stamped]

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
                # 如果 Nav2 反馈成功，也尝试调用切换逻辑
                if self.current_goal_index == 0:
                    self.get_logger().info("Nav2 反馈到达 A，执行切换...")
                    self.switch_to_phase_b()
                else:
                    self.get_logger().info("MISSION COMPLETE: 到达 B，任务结束。")
                    self.state = self.STATE_IDLE
                    self.final_goal_pose = None

            elif self.current_active_goal_type == 'BONUS':
                self.get_logger().info("Bonus Reached")
                self.recover_to_final()
        
    def publish_current_goal_marker(self):
        if not self.enable_debug: return
        marker_array = MarkerArray()
        delete_marker = Marker(header=self.get_current_header(), ns="active_goal", action=Marker.DELETEALL)
        marker_array.markers.append(delete_marker)
        if self.current_active_pose_map is None:
            self.marker_pub.publish(marker_array)
            return
        marker = Marker(header=self.get_current_header(), ns="active_goal", action=Marker.ADD)
        marker.id = 1
        if self.current_active_goal_type == 'FINAL':
            marker.type = Marker.ARROW
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 0.0, 1.0, 1.0 
            phase_name = "A" if self.current_goal_index == 0 else "B"
            text = f"FINAL-{phase_name}"
        else: 
            marker.type = Marker.SPHERE
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.5, 0.0, 1.0 
            text = f"BONUS ({self.current_active_goal_value})"
        marker.pose = self.current_active_pose_map.pose
        marker.scale.x, marker.scale.y, marker.scale.z = 0.5, 0.5, 0.5
        marker_array.markers.append(marker)
        text_marker = Marker(header=self.get_current_header(), ns="active_goal_label", action=Marker.ADD)
        text_marker.id = 2
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.pose = self.current_active_pose_map.pose
        text_marker.pose.position.z += 0.6
        text_marker.text = text
        text_marker.scale.z = 0.3
        text_marker.color.r, text_marker.color.g, text_marker.color.b, text_marker.color.a = 1.0, 1.0, 1.0, 1.0
        marker_array.markers.append(text_marker)
        self.marker_pub.publish(marker_array)

    def get_current_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.global_frame
        return header

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()