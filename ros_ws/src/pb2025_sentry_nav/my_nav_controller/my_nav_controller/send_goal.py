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

        # 订阅 STM32 发送的裁判系统数据
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

        # 预定义目标点位置字典
        self.target_points = {
            1: (-3.7, -5.75, 0.0),  # 目标点1
            2: (-3.7, -5.75, 0.0),  # 目标点2
            3: (-3.7, -5.75, 0.0),  # 目标点3
            4: (-3.7, -5.75, 0.0),  # 目标点4
            5: (-0.79, -7.21, 0.0),  # 目标点5
            6: (-3.7, -5.75, 0.0),  # 目标点6
            7: (-3.7, -5.75, 0.0),  # 目标点7
            8: (-0.58, -0.26, 0.0),  # 目标点8 - 用于回到补给区
            9: (-6.83, 1.95, 0.0),   # 目标点9
            10: (-6.8, 1.70, 0.0)    # 目标点10
        }

        # 巡逻点定义 - 八个巡逻点的坐标
        self.patrol_points = [
            (-4.79, 4.96, 0.0),  # 巡逻点1
            (-2.33, 4.81, 0.0),  # 巡逻点2
            (-2.31, 5.90, 0.0), # 巡逻点3
            (-4.01, 5.97, 0.0),  # 巡逻点4
            (-2.31, 5.90, 0.0), # 巡逻点5
            (-2.33, 4.81, 0.0),  # 巡逻点6
            (-4.79, 4.96, 0.0),  # 巡逻点7
            (-2.34, 2.33, 0.0)   # 巡逻点8
        ]
        
        # 安全点 - 用于血量低时前往
        self.safe_point_index = 7  # 巡逻点5作为安全点
        
        self.current_condition = 0  # 记录当前目标编号
        self.current_goal_handle = None  # 记录当前导航状态
        self.is_navigating = False  # 当前导航状态
        self.buff = 0.1  # 修的bug
        
        # 巡逻模式相关变量
        self.patrol_mode = False  # 巡逻模式开关
        self.current_patrol_index = 0  # 当前巡逻点索引
        self.patrol_timer = None  # 巡逻计时器
        self.normal_patrol = True  # 是否处于正常巡逻
        
        # 血量监控相关变量
        self.last_hp = None  # 上次的血量值
        self.hp_history = []  # 存储最近的血量记录 [(timestamp, hp)]
        self.hp_window = 1.0  # 计算掉血速度的时间窗口（秒）
        self.low_hp_threshold = 90  # 低血量阈值，改为90
        self.is_returning_home = False  # 是否正在前往安全点
        
        # 初始化完成后直接启动巡逻模式
        self.get_logger().info("系统初始化完成，准备开始8点巡逻模式")
        self.patrol_mode = True
        # 使用定时器延迟一下启动巡逻，确保系统完全初始化
        self.create_timer(2.0, self.delayed_start_patrol)

    def delayed_start_patrol(self):
        """延迟启动巡逻，确保系统完全初始化"""
        x, y, yaw = self.patrol_points[self.current_patrol_index]
        self.send_goal(x, y, yaw)
        self.get_logger().info(f"开始巡逻: 前往第 {self.current_patrol_index+1}/8 个巡逻点")
        # 这是一次性定时器，执行后销毁
        for timer in self.timers:
            if timer.callback == self.delayed_start_patrol:
                self.destroy_timer(timer)
                break

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
            
            # 如果是前往安全点，则继续在那里等待
            if not self.normal_patrol and self.current_patrol_index == self.safe_point_index:
                self.get_logger().info("已到达安全点，等待血量恢复...")
                return
                
            # 如果在巡逻模式下成功到达目标，立即前往下一个巡逻点（没有停顿）
            if self.patrol_mode:
                self.navigate_to_next_patrol_point()
        else:
            self.get_logger().warn(f"导航失败，状态码: {status}")
            
            # 即使导航失败，也尝试继续下一个点
            if self.patrol_mode:
                self.navigate_to_next_patrol_point()

        if not self.patrol_mode:
            self.is_navigating = False  # 任务结束
            self.current_goal_handle = None  # 清除当前导航任务

    def navigate_to_next_patrol_point(self):
        """导航到下一个巡逻点"""
        if self.patrol_timer:
            self.destroy_timer(self.patrol_timer)
            self.patrol_timer = None
            
        if self.normal_patrol:
            # 正常巡逻在8个点之间循环
            self.current_patrol_index = (self.current_patrol_index + 1) % 8  # 8点循环
        else:
            # 如果不是正常巡逻，则前往安全点
            self.current_patrol_index = self.safe_point_index
            
        x, y, yaw = self.patrol_points[self.current_patrol_index]
        self.get_logger().info(f"巡逻中: 前往第 {self.current_patrol_index+1}/8 个巡逻点")
        self.send_goal(x, y, yaw)

    def start_patrol(self):
        """开始巡逻模式"""
        if self.patrol_mode:
            self.get_logger().info("已经处于巡逻模式")
            return
            
        self.patrol_mode = True
        self.current_patrol_index = 0
        self.normal_patrol = True  # 恢复正常巡逻模式
        
        # 取消当前导航任务（如果有）
        self.cancel_goal()
        
        # 前往第一个巡逻点
        x, y, yaw = self.patrol_points[self.current_patrol_index]
        self.send_goal(x, y, yaw)

    def stop_patrol(self):
        """停止巡逻模式"""
        if not self.patrol_mode:
            return
            
        self.patrol_mode = False
        if self.patrol_timer:
            self.destroy_timer(self.patrol_timer)
            self.patrol_timer = None
            
        self.get_logger().info("停止巡逻模式")
        
        # 取消当前巡逻导航
        self.cancel_goal()

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
        """接收 STM32 传来的数据，判断是否需要导航和进行血量检测"""
        # 血量检测逻辑
        current_time = time.time()
        current_hp = msg.remain_hp
        
        # 定义最大血量值
        max_hp = 100  # 假设最大血量为100，根据实际情况调整
        
        # 如果血量低于阈值并且不是在前往安全点，则去安全点
        if current_hp < self.low_hp_threshold and self.normal_patrol and self.patrol_mode:
            self.get_logger().warn(f"血量低于安全阈值({self.low_hp_threshold})! 当前血量: {current_hp}")
            self.normal_patrol = False  # 切换到非正常巡逻模式
            self.is_returning_home = True
            
            # 立即取消当前导航并前往安全点
            if self.current_goal_handle:
                self.cancel_goal()
                
            self.current_patrol_index = self.safe_point_index
            x, y, yaw = self.patrol_points[self.current_patrol_index]
            self.get_logger().info(f"前往安全点: 巡逻点{self.safe_point_index+1} ({x}, {y})")
            self.send_goal(x, y, yaw)
            return
            
        # 如果血量回满，恢复正常巡逻
        if current_hp >= max_hp and not self.normal_patrol and self.patrol_mode:
            self.get_logger().info(f"血量已回满: {current_hp}，重新开始正常巡逻")
            self.normal_patrol = True
            self.is_returning_home = False
            
            # 取消当前导航并重新开始巡逻
            if self.current_goal_handle:
                self.cancel_goal()
                
            self.current_patrol_index = 0
            x, y, yaw = self.patrol_points[self.current_patrol_index]
            self.send_goal(x, y, yaw)
            return
        
        # 实现快速掉血检测
        if self.last_hp is not None:
            # 记录当前血量和时间
            self.hp_history.append((current_time, current_hp))
            
            # 移除超出时间窗口的记录
            while self.hp_history and (current_time - self.hp_history[0][0] > self.hp_window):
                self.hp_history.pop(0)
            
            # 检测快速掉血
            if len(self.hp_history) >= 2:
                oldest_time, oldest_hp = self.hp_history[0]
                hp_loss = oldest_hp - current_hp
                time_diff = current_time - oldest_time
                
                if time_diff > 0:
                    hp_loss_rate = hp_loss / time_diff  # 每秒掉血量
                    # 如果掉血速度超过5滴每秒，终止巡逻，前往目标点5
                    if hp_loss_rate > 5 and self.patrol_mode:
                        self.get_logger().warn(f"检测到快速掉血! 速率: {hp_loss_rate:.2f}/秒")
                        self.stop_patrol()
                        # 前往目标点5
                        x, y, yaw = self.target_points[5]
                        self.send_goal(x, y, yaw)
                        self.current_condition = 5
                        return
                        
        # 更新最后血量记录
        self.last_hp = current_hp
        
        # 保留停止巡逻模式的功能
        if msg.game_progress == 4 and msg.remain_hp == 888:  # 停止巡逻模式的条件
            self.stop_patrol()
            return
                
        # 如果正在巡逻模式，忽略普通导航命令
        if self.patrol_mode:
            return

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