import numpy as np

class GoalConverter:
    """
    负责将 YOLO 像素坐标转换为机器人本体坐标系 (body frame) 下的物理坐标。
    这是一个纯计算类，不依赖于 ROS。
    """
    def __init__(self, params: dict):
        self.bev_width = params['bev_width']
        self.bev_height = params['bev_height']
        
        # 计算物理比例
        self.meters_per_pixel_x = params['world_width_m'] / self.bev_width
        self.meters_per_pixel_y = params['world_height_m'] / self.bev_height
        
        self.origin_offset_x_m = params['origin_offset_x_m']
        self.origin_offset_y_m = params['origin_offset_y_m']

    def pixel_to_body_coordinates(self, u: float, v: float) -> tuple:
        """
        将 BEV 图像中的像素 (u, v) 转换为 body 坐标系下的 (x, y, z=0)。
        X: 前进方向, Y: 侧向方向
        """
        # X (前进):
        robot_x = self.origin_offset_x_m + (self.bev_height - v) * self.meters_per_pixel_y
        
        # Y (侧向):
        robot_y = self.origin_offset_y_m - u * self.meters_per_pixel_x
        
        return robot_x, robot_y, 0.0

    @staticmethod
    def find_best_detection(detections):
        """
        在当前帧的检测结果中找到价值最高的点。
        返回: (best_detection, max_value)
        """
        best_detection = None
        max_value = -1

        for det in detections:
            try:
                # 假设 class_id 命名如 'num_8' 或 '8'
                class_id_str = det.results[0].hypothesis.class_id
                # 尝试提取末尾数字，或整个字符串
                value = int(class_id_str[-1]) 
                
                if value > max_value:
                    max_value = value
                    best_detection = det
            except (ValueError, IndexError, AttributeError):
                continue
        return best_detection, max_value