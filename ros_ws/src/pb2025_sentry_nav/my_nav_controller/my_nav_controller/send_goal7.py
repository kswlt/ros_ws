#yaw控制测试
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int8, String  # 状态消息类型
from action_msgs.msg import GoalStatus  # 用于取消导航
import math

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')

        # 创建 Action 客户端
        self.client = ActionClient(self, NavigateToPose, '/red_standard_robot1/navigate_to_pose')

        # 发布导航状态（1=导航中，0=未导航）
        self.status_publisher = self.create_publisher(Int8, 'nav2_status', 10)
        
        # 发布策略状态
        self.strategy_publisher = self.create_publisher(String, 'strategy_status', 10)

        # 每秒发布一次状态
        self.timer = self.create_timer(1.0, self.publish_status)

        # 目标点(只保留目标点1)
        self.target_point = (-1.39, 5.17, 0.0)  # 目标点1 - x, y, 初始朝向

        # 状态变量
        self.current_goal_handle = None     # 当前导航任务
        self.is_navigating = False          # 导航状态
        self.buff = 0.1                     # 取消导航用的标志
        
        # 旋转相关状态变量
        self.is_rotating = False            # 是否正在执行旋转序列
        self.rotation_step = 0              # 当前旋转步骤
        self.rotation_angles = [            # 旋转角度序列(弧度)
            0.0,                            # 0度
            math.pi/2,                      # 90度
            math.pi,                        # 180度
            3*math.pi/2                     # 270度
        ]
        self.rotation_delay = 3.0           # 旋转间隔时间(秒)
        self.rotation_timer = None          # 旋转定时器
        
        self.robot_state = "初始化"          # 机器人状态
        
        self.get_logger().info("导航系统初始化完成，前往目标点1")
        
        # 直接前往目标点1
        self.send_goal()

    def send_goal(self, custom_yaw=None):
        """发送导航目标点，可选自定义朝向"""
        x, y, default_yaw = self.target_point
        
        # 使用自定义朝向或默认朝向
        yaw = custom_yaw if custom_yaw is not None else default_yaw
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = self.yaw_to_quaternion(yaw)
        
        self.get_logger().info(f"发送导航目标: x={x}, y={y}, yaw={yaw}")

        self.client.wait_for_server(timeout_sec=5.0)
        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        
    def send_rotation_goal(self):
        """发送旋转点"""
        if not self.is_rotating or self.rotation_step >= len(self.rotation_angles):
            self.is_rotating = False
            self.rotation_step = 0
            return
            
        # 获取目标点的坐标
        x, y, _ = self.target_point
        yaw = self.rotation_angles[self.rotation_step]
        
        # 更新状态
        angle_deg = int(yaw * 180 / math.pi)
        self.get_logger().info(f"发送旋转点: 朝向={angle_deg}度")
        self.robot_state = f"原地旋转中: {angle_deg}度"
        
        # 发送导航目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = self.yaw_to_quaternion(yaw)
        
        self.client.wait_for_server(timeout_sec=5.0)
        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        
    def schedule_next_rotation(self):
        """安排下一个旋转点"""
        # 更新旋转步骤
        self.rotation_step += 1
        
        if self.rotation_step >= len(self.rotation_angles):
            # 旋转序列完成
            self.is_rotating = False
            self.rotation_step = 0
            self.get_logger().info("完成一周旋转")
            self.robot_state = "守卫中"
            return
            
        # 创建定时器延迟发送下一个旋转点
        if self.rotation_timer:
            self.destroy_timer(self.rotation_timer)
        
        self.get_logger().info(f"将在{self.rotation_delay}秒后发送下一个旋转点")
        self.robot_state = f"等待下一次旋转...{self.rotation_delay}秒"
        self.rotation_timer = self.create_timer(self.rotation_delay, self._execute_next_rotation)
        
    def _execute_next_rotation(self):
        """执行下一个旋转点发送"""
        # 销毁定时器
        if self.rotation_timer:
            self.destroy_timer(self.rotation_timer)
            self.rotation_timer = None
            
        # 发送下一个旋转点
        if self.is_rotating:
            self.send_rotation_goal()

    def goal_response_callback(self, future):
        """处理导航目标的接受情况"""
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn("导航目标未被接受")
            self.is_navigating = False
            return
        
        self.current_goal_handle = goal_handle
        self.get_logger().info("导航目标已被接受，开始导航")
        self.is_navigating = True

        # 监听导航任务完成情况
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """处理导航完成或失败"""
        if self.buff == 1:
            self.buff = 0
            return
            
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("导航成功到达目标点")
            
            # 检查是否正在执行旋转序列
            if self.is_rotating:
                # 完成当前旋转步骤，安排下一个
                self.schedule_next_rotation()
                return
            
            # 第一次到达目标点，开始旋转序列
            if not self.is_rotating:
                self.get_logger().info("到达目标点，开始执行旋转序列")
                self.is_rotating = True
                self.rotation_step = 0
                self.send_rotation_goal()
                return
                
        else:
            self.get_logger().warn(f"导航失败，状态码: {status}")
            
            # 如果旋转过程中导航失败，尝试下一个旋转点
            if self.is_rotating:
                self.get_logger().warn("旋转点导航失败，尝试下一个旋转点")
                self.schedule_next_rotation()
                return
                
            # 初始导航失败，3秒后重试
            self.get_logger().warn("导航失败，3秒后重试")
            retry_timer = self.create_timer(3.0, lambda: self.retry_navigation())
            self.create_timer(0.1, lambda: self.destroy_timer(retry_timer))

        self.is_navigating = False
        self.current_goal_handle = None
        
    def retry_navigation(self):
        """重试导航到目标点"""
        self.get_logger().info("重试导航到目标点")
        self.send_goal()

    def cancel_goal(self):
        """取消当前导航"""
        if self.current_goal_handle is None:
            return
            
        self.buff = 1
        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        """取消任务完成后的回调"""
        self.get_logger().info("导航目标已取消")
        self.is_navigating = False
        self.current_goal_handle = None

    def publish_status(self):
        """每秒发布导航状态 (1=导航中, 0=未导航)"""
        msg = Int8()
        msg.data = 1 if self.is_navigating else 0
        self.status_publisher.publish(msg)
        
        # 发布策略状态
        status_msg = String()
        status_msg.data = f"目标: 目标点1 | 状态: {self.robot_state}"
        self.strategy_publisher.publish(status_msg)

    def yaw_to_quaternion(self, yaw):
        """将 yaw 角转换为四元数"""
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

def main(args=None):
    rclpy.init(args=args)
    nav_client = NavigationClient()
    rclpy.spin(nav_client)
    nav_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



