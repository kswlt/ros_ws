import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile
from referee_msg.msg import Referee  # 自定义消息类型
from action_msgs.msg import GoalStatus  # 用于取消导航
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int8, String  # 状态消息类型
import math

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')

        # 创建 Action 客户端
        self.client = ActionClient(self, NavigateToPose, '/red_standard_robot1/navigate_to_pose')

        # 订阅 STM32 数据
        self.subscription = self.create_subscription(
            Referee,
            'stm32_ros2_data',
            self.condition_callback,
            10
        )

        # 发布导航状态（1=导航中，0=未导航）
        self.status_publisher = self.create_publisher(Int8, 'nav2_status', 10)
        
        # 发布策略状态
        self.strategy_publisher = self.create_publisher(String, 'strategy_status', 10)

        # 每秒发布一次状态
        self.timer = self.create_timer(1.0, self.publish_status)

        # 目标点集合 - 简化为3个主要点和1个过渡点
        self.target_points = {
            1: (-5.39, -5.17, 0.0),    # 目标点1 - 基地血量高时的停留点
            2: (-5.99, -5.68, 0.0),   # 目标点2 - 基地血量低时的停留点
            3: (-1.16, -0.33, 0.0),    # 回血点 - 用于回血和补给
            5: (-1.39, 1.01, 0.0)     # 目标点5 - 从回血点返回后的中转点
        }

        # 状态变量
        self.current_target = 0             # 当前目标点
        self.current_goal_handle = None     # 当前导航任务
        self.is_navigating = False          # 导航状态
        self.buff = 0.1                     # 取消导航用的标志
        self.game_has_started = False       # 比赛是否开始
        self.game_start_time = None         # 比赛开始时间
        
        # 血量和弹药相关
        self.current_hp = 600               # 当前血量
        self.current_ammo = 100             # 当前弹药量
        self.red_base_hp = 5000             # 红方基地血量
        self.is_healing = False             # 是否在回血点
        self.is_going_to_heal = False       # 是否正在前往回血点
        self.healing_start_time = None      # 开始回血的时间
        self.navigating_to_heal_point_id = 3 # 定义回血点ID
        
        # 回血和目标选择阈值
        self.low_hp_threshold = 390         # 低血量阈值
        self.high_hp_threshold = 580        # 血量回满阈值，接近满血600
        self.low_ammo_threshold = 20        # 低弹药阈值
        self.high_ammo_threshold = 90       # 弹药充足阈值
        self.base_hp_threshold = 3000       # 基地血量决策阈值
        self.healing_time = 5.0             # 回血时间(秒)，作为备用
        
        # 添加目标点发送延迟相关变量
        self.goal_delay = 1.0               # 两个目标点之间的延迟时间(秒)
        self.next_goal_timer = None         # 延迟发送的定时器
        self.delayed_target = None          # 延迟发送的目标点ID
        
        self.robot_state = "初始化"          # 机器人状态
        
        self.get_logger().info("导航系统初始化完成，等待比赛开始...")

    def send_goal(self, target_id):
        """发送导航目标点"""
        if target_id not in self.target_points:
            self.get_logger().error(f"目标点 {target_id} 不在定义的范围内")
            return
            
        x, y, yaw = self.target_points[target_id]
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = self.yaw_to_quaternion(yaw)
        
        self.current_target = target_id
        self.get_logger().info(f"发送目标点{target_id}: x={x}, y={y}, yaw={yaw}")

        self.client.wait_for_server(timeout_sec=1.0)
        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        
    def send_goal_with_delay(self, target_id, delay=None):
        """延迟一段时间后发送导航目标点"""
        if delay is None:
            delay = self.goal_delay
            
        # 取消之前的延迟发送定时器(如果存在)
        if self.next_goal_timer:
            self.destroy_timer(self.next_goal_timer)
            self.next_goal_timer = None
        
        # 保存要发送的目标点ID
        self.delayed_target = target_id
        
        # 更新状态
        self.robot_state = f"等待{delay}秒后前往目标点{target_id}"
        self.get_logger().info(f"将在{delay}秒后发送目标点{target_id}")
        
        # 创建定时器（将在回调中销毁）
        self.next_goal_timer = self.create_timer(delay, self._execute_delayed_goal)

    def _execute_delayed_goal(self):
        """执行延迟的目标点发送"""
        if self.delayed_target is not None:
            target = self.delayed_target
            self.delayed_target = None
            self.send_goal(target)
        
        # 销毁定时器以实现一次性功能
        if self.next_goal_timer:
            self.destroy_timer(self.next_goal_timer)
            self.next_goal_timer = None

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
            self.get_logger().info(f"导航成功到达目标点{self.current_target}")
            
            # 到达目标点后的行为
            if self.current_target == self.navigating_to_heal_point_id:  # 回血点
                # 到达回血点，开始回血
                self.is_healing = True
                self.is_going_to_heal = False  # 重置"正在前往回血点"标志
                self.healing_start_time = time.time()
                self.get_logger().info(f"已到达回血点，开始回血，等待血量达到{self.high_hp_threshold}")
                self.robot_state = "在回血点回血中"
                return
                
            elif self.current_target == 5:  # 从回血点返回后的中转点
                # 到达中转点后，根据基地血量决定下一个目标点
                if self.red_base_hp >= self.base_hp_threshold:
                    self.get_logger().info("到达目标点5，基地血量充足，准备前往目标点1")
                    self.send_goal_with_delay(1)
                else:
                    self.get_logger().info("到达目标点5，基地血量不足，准备前往目标点2")
                    self.send_goal_with_delay(2)
                return
                
        else:
            self.get_logger().warn(f"导航失败，状态码: {status}")
            
            # 如果前往回血点失败，重置标志
            if self.current_target == self.navigating_to_heal_point_id and self.is_going_to_heal:
                self.get_logger().warn("前往回血点失败")
                self.is_going_to_heal = False
                
            # 5秒后重试导航
            retry_timer = self.create_timer(1.0, lambda: self.retry_navigation(self.current_target))
            # 确保这个定时器是一次性的
            self.create_timer(0.1, lambda: self.destroy_timer(retry_timer))

        self.is_navigating = False
        self.current_goal_handle = None
        
    def retry_navigation(self, target_id):
        """重试导航到指定目标点"""
        self.get_logger().info(f"重试导航到目标点{target_id}")
        self.send_goal(target_id)

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

    def condition_callback(self, msg):
        """接收 STM32 传来的数据，进行决策"""
        # 更新关键数据
        self.current_hp = msg.remain_hp
        self.current_ammo = msg.bullet_remaining_num_17mm
        self.red_base_hp = msg.red_base_hp
        
        # ===== 最高优先级：检查比赛是否开始 =====
        if msg.game_progress == 4 and not self.game_has_started:
            self.game_has_started = True
            self.game_start_time = time.time()
            self.get_logger().info("比赛开始!")
            
            # 根据基地血量决定初始目标点，使用延迟发送
            if self.red_base_hp >= self.base_hp_threshold:
                self.get_logger().info("基地血量充足，准备前往目标点1")
                self.robot_state = "开始比赛，准备出发"
                self.send_goal_with_delay(1, 1.0)  # 开始时使用较短延迟
            else:
                self.get_logger().info("基地血量不足，准备前往目标点2")
                self.robot_state = "开始比赛，准备出发"
                self.send_goal_with_delay(2, 1.0)  # 开始时使用较短延迟
            return
            
        # 如果比赛未开始，不执行其他逻辑
        if not self.game_has_started:
            return
            
        # ===== 检查是否需要前往回血点 =====
        if (self.current_hp < self.low_hp_threshold or 
            self.current_ammo < self.low_ammo_threshold) and \
           not self.is_healing and \
           not self.is_going_to_heal and \
           (self.delayed_target != self.navigating_to_heal_point_id):  # 避免当延迟目标已经是回血点时还重入
            
            reason = "血量不足" if self.current_hp < self.low_hp_threshold else "弹药不足"
            self.get_logger().warn(f"{reason}，准备前往回血点")
            
            # 紧急情况使用较短延迟
            self.cancel_goal()
            self.robot_state = f"{reason}，准备前往回血点"
            self.is_going_to_heal = True  # 设置标志，表示已经决定要去回血点
            self.send_goal_with_delay(self.navigating_to_heal_point_id, 1.0)
            return
            
        # ===== 回血点检测 =====
        if self.is_healing:
            # 只检查血量是否已经回满，不再检查弹药
            if self.current_hp >= self.high_hp_threshold:
                # 血量已充足，可以离开
                self.is_healing = False
                self.healing_start_time = None
                
                # 根据基地血量直接决定下一个目标点，不需要经过中转点5
                if self.red_base_hp >= self.base_hp_threshold:
                    self.get_logger().info(f"回血完成，基地血量充足，直接前往目标点1")
                    self.robot_state = "回血完成，准备出发前往目标点1"
                    self.send_goal_with_delay(1)
                else:
                    self.get_logger().info(f"回血完成，基地血量不足，直接前往目标点2")
                    self.robot_state = "回血完成，准备出发前往目标点2"
                    self.send_goal_with_delay(2)
                return
            else:
                # 血量还不足，继续回血
                hp_needed = self.high_hp_threshold - self.current_hp
                self.robot_state = f"回血中...还需补充血量{hp_needed}"
                
        # 发布当前策略状态
        self.publish_strategy_status()

    def publish_status(self):
        """每秒发布导航状态 (1=导航中, 0=未导航)"""
        msg = Int8()
        msg.data = 1 if self.is_navigating else 0
        self.status_publisher.publish(msg)
        
    def publish_strategy_status(self):
        """发布当前策略和状态信息"""
        status_msg = String()
        
        game_duration = 0
        if self.game_has_started and self.game_start_time:
            game_duration = time.time() - self.game_start_time
            
        game_mins = int(game_duration / 60)
        game_secs = int(game_duration % 60)
        
        status_msg.data = (
            f"比赛时间: {game_mins}分{game_secs}秒 | "
            f"红方基地: {self.red_base_hp} | "
            f"血量: {self.current_hp} | "
            f"弹药: {self.current_ammo} | "
            f"目标点: {self.current_target} | "
            f"状态: {self.robot_state}"
        )
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



