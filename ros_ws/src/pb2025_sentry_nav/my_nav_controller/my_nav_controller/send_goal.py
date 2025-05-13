# 导入必要的库
import time  # 用于时间相关操作
import rclpy  # ROS2 Python客户端库
from rclpy.node import Node  # ROS2节点基类
from rclpy.action import ActionClient  # ROS2动作客户端
from nav2_msgs.action import NavigateToPose  # 导航到指定位置的动作消息
from rclpy.qos import QoSProfile  # 服务质量配置
from referee_msg.msg import Referee  # 自定义裁判系统消息类型，用于接收STM32数据
from action_msgs.msg import GoalStatus  # 用于处理导航目标状态
from geometry_msgs.msg import Quaternion  # 四元数消息，用于表示旋转
from std_msgs.msg import Int8  # 整型消息，用于发布导航状态
from std_srvs.srv import Trigger  # 用于创建服务
import math  # 数学库，用于角度转换

class NavigationClient(Node):
    """
    导航客户端节点类，负责处理导航请求和状态
    """
    def __init__(self):
        """
        初始化导航客户端节点
        """
        super().__init__('navigation_client')  # 初始化节点，命名为'navigation_client'

        # 创建 Action 客户端，连接到导航服务
        self.client = ActionClient(self, NavigateToPose, '/red_standard_robot1/navigate_to_pose')

        # 订阅 STM32 发送的裁判系统数据 (保留这个用于其他功能)
        self.subscription = self.create_subscription(
            Referee,
            'stm32_ros2_data',
            self.condition_callback,
            10
        )

        # 发布导航状态（1=导航中，0=未导航）
        self.status_publisher = self.create_publisher(Int8, 'nav2_status', 10)

        # 创建定时器，每秒发布一次状态
        self.timer = self.create_timer(1.0, self.publish_status)

        # 导航状态变量
        self.current_goal_handle = None  # 记录当前导航状态
        self.is_navigating = False  # 当前导航状态
        self.buff = 0  # 修的bug
        
        # 上坡相关变量
        self.is_climbing = False  # 是否正在上坡
        self.climb_index = 0  # 当前上坡导航点索引
        
        # 上坡导航点序列 - 5个点从坡底到坡顶，根据实际坡道位置调整
        self.climbing_points = [
            (-1.22, 2.11, 0.0),   # 坡道底部起点
            (0.64, 1.90, 0.0),   # 上坡1/4处
            (0.96, 2.07, 0.0),   # 上坡中点
            (0.89, 3.22, 0.0),   # 上坡3/4处
            (-0.78, 3.33, 0.0)    # 坡道顶部
        ]
        
        self.get_logger().info("导航节点初始化完成，准备开始上坡")
        
        # 使用定时器延迟一下启动上坡，确保系统完全初始化
        self.create_timer(2.0, self.delayed_start_climbing)

    def send_goal(self, x, y, yaw):
        """发送导航目标点"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = self.yaw_to_quaternion(yaw)  # 转换成四元数
        
        self.get_logger().info(f"发送目标点: x={x}, y={y}, yaw={yaw}")
        
        self.client.wait_for_server(timeout_sec=5.0)
        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """处理导航目标的接受情况"""
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn("导航目标未被接受")
            self.is_navigating = False  # 未导航
            return
            
        self.current_goal_handle = goal_handle
        self.get_logger().info("导航目标已被接受，开始导航")
        self.is_navigating = True  # 进入导航状态
        
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
            
            # 如果正在上坡，继续到下一个上坡点
            if self.is_climbing:
                self.navigate_to_next_climbing_point()
                return
                
        else:
            self.get_logger().warn(f"导航失败，状态码: {status}")
            
            # 如果上坡导航失败，也尝试继续下一个点
            if self.is_climbing and self.climb_index < len(self.climbing_points):
                self.get_logger().warn("上坡导航点失败，尝试继续下一个点")
                self.navigate_to_next_climbing_point()
                return

        # 非上坡模式或上坡完成时
        self.is_navigating = False
        self.current_goal_handle = None

    def climb_service_callback(self, request, response):
        """服务回调：开始上坡"""
        self.get_logger().info("收到上坡请求")
        self.start_climbing()
        response.success = True
        response.message = "上坡模式已启动"
        return response
    
    def start_climbing(self):
        """开始上坡模式，按顺序导航通过5个引导点"""
        # 如果已经在上坡模式，直接返回
        if self.is_climbing:
            self.get_logger().info("已经处于上坡模式")
            return
        
        # 取消当前导航任务（如果有）
        self.cancel_goal()
        
        # 设置上坡状态
        self.is_climbing = True
        self.climb_index = 0
        
        self.get_logger().info("开始上坡模式")
        
        # 导航到第一个上坡点
        self.navigate_to_next_climbing_point()

    def navigate_to_next_climbing_point(self):
        """导航到下一个上坡点"""
        # 检查是否处于上坡模式
        if not self.is_climbing:
            return
        
        # 检查是否已完成所有上坡点
        if self.climb_index >= len(self.climbing_points):
            self.get_logger().info("上坡导航完成")
            # 完成上坡导航，重置状态
            self.is_climbing = False
            self.is_navigating = False
            self.current_goal_handle = None
            return
        
        # 获取当前要导航的上坡点
        x, y, yaw = self.climbing_points[self.climb_index]
        self.get_logger().info(f"上坡导航: 前往第 {self.climb_index+1}/5 个点 ({x}, {y})")
        
        # 发送导航目标
        self.send_goal(x, y, yaw)
        
        # 增加索引，准备下一个点
        self.climb_index += 1

    def cancel_goal(self):
        """取消当前导航"""
        if self.current_goal_handle is None:
            return  # 没有目标，不需要取消
            
        self.buff = 1
        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        """取消任务完成后的回调"""
        self.get_logger().info("导航目标已取消")
        self.is_navigating = False  # 取消后进入未导航状态
        self.current_goal_handle = None  # 清除当前任务

    def condition_callback(self, msg):
        """接收 STM32 传来的数据"""
        # 删除上坡触发逻辑，这里可以保留空函数，或者添加其他需要的功能
        pass

    def publish_status(self):
        """每秒发布导航状态 (1=导航中, 0=未导航)"""
        msg = Int8()
        msg.data = 1 if self.is_navigating else 0
        self.status_publisher.publish(msg)

    def yaw_to_quaternion(self, yaw):
        """将 yaw 角转换为四元数"""
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

def main(args=None):
    rclpy.init(args=args)
    nav_client = NavigationClient()
    rclpy.spin(nav_client)  # 让节点一直运行，等待消息
    nav_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()