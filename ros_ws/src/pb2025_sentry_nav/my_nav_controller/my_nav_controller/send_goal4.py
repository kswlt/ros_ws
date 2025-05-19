#区域塞半场基本功能实现第二版

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
from std_msgs.msg import Int8, String  # 整型消息，用于发布导航状态和字符串消息
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
        
        # 发布当前战略信息
        self.strategy_publisher = self.create_publisher(String, 'strategy_status', 10)

        # 创建定时器，每秒发布一次状态
        self.timer = self.create_timer(1.0, self.publish_status)

        # 预定义目标点位置字典
        self.target_points = {
            1: (-3.7, -5.75, 0.0),  # 目标点1 - 主战场
            2: (-3.7, -5.75, 0.0),  # 目标点2 - 前哨站
            3: (-3.7, -5.75, 0.0),  # 目标点3
            4: (-3.7, -5.75, 0.0),  # 目标点4
            5: (-0.79, -7.21, 0.0),  # 目标点5 - 回血点
            6: (-3.7, -5.75, 0.0),  # 目标点6
            7: (-3.7, -5.75, 0.0),  # 目标点7
            8: (-0.58, -0.26, 0.0),  # 目标点8 - 用于回到补给区
            9: (-6.83, 1.95, 0.0),   # 目标点9
            10: (-6.8, 1.70, 0.0)    # 目标点10
        }

        # 巡逻点定义 - 八个巡逻点的坐标（暂时注释，不使用）
        """
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
        """
        
        # 状态变量
        self.current_condition = 0  # 记录当前目标编号
        self.current_goal_handle = None  # 记录当前导航状态
        self.is_navigating = False  # 当前导航状态
        self.buff = 0.1  # 修的bug
        
        # 暂时注释掉巡逻模式相关变量
        """
        self.patrol_mode = False  # 巡逻模式开关
        self.current_patrol_index = 0  # 当前巡逻点索引
        self.patrol_timer = None  # 巡逻计时器
        self.normal_patrol = True  # 是否处于正常巡逻
        """
        
        # 比赛和血量相关变量
        self.game_has_started = False  # 比赛是否开始
        self.game_start_time = None  # 比赛开始时间
        self.current_hp = 600  # 当前血量（默认设为最大）
        self.current_ammo = 100  # 当前弹药量（默认设为最大）
        self.target_point2_arrival_time = None  # 到达目标点2的时间
        self.waiting_at_point2 = False  # 是否在目标点2等待
        self.hp_threshold_high = 400  # 血量恢复目标
        self.hp_threshold_medium = 350  # 血量中等阈值
        self.hp_threshold_medium_low = 300  # 血量较低阈值
        self.hp_threshold_critical = 150  # 血量危急阈值
        self.ammo_threshold_low = 40  # 弹药量低阈值
        self.point2_wait_time = 5.0  # 在目标点2等待的时间（秒）
        self.last_hp = None  # 上次血量记录
        self.is_healing = False  # 是否在回血点恢复
        
        # 添加当前目标点和当前状态变量
        self.current_target = 1  # 初始目标点为1
        self.robot_state = "初始化"  # 机器人当前状态
        
        # 添加死亡状态相关变量
        self.is_dead = False  # 是否已死亡
        self.death_time = None  # 死亡时间
        self.revival_count = 0  # 复活次数计数
        
        # 初始化完成
        self.get_logger().info("导航系统初始化完成，等待比赛开始...")

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
            self.get_logger().info(f"导航成功到达目标点{self.current_target}")
            
            # 如果到达了目标点2，记录时间
            if self.current_target == 2:
                self.target_point2_arrival_time = time.time()
                self.waiting_at_point2 = True
                self.get_logger().info(f"已到达目标点2，开始等待{self.point2_wait_time}秒观察是否掉血")
            
            # 如果到达了回血点，设置恢复状态
            elif self.current_target == 5:
                self.is_healing = True
                self.get_logger().info("已到达回血点，等待血量恢复...")
            
        else:
            self.get_logger().warn(f"导航失败，状态码: {status}")
            # 5秒后重试当前目标点
            self.create_timer(5.0, lambda: self.retry_current_goal())
            
        self.is_navigating = False  # 设置未导航状态
        
    def retry_current_goal(self):
        """重试当前目标点"""
        if not self.is_navigating:
            self.get_logger().info(f"重试前往目标点{self.current_target}")
            x, y, yaw = self.target_points[self.current_target]
            self.send_goal(x, y, yaw)

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
        """接收 STM32 传来的数据，根据比赛状态和血量调整导航"""
        # 更新当前血量和弹药量
        self.current_hp = msg.remain_hp
        self.current_ammo = msg.bullet_remaining_num_17mm
        
        # ===== 死亡和复活检测 =====
        # 检测死亡状态
        if self.current_hp == 0 and not self.is_dead:
            self.is_dead = True
            self.death_time = time.time()
            self.get_logger().warn("哨兵血量为0，进入死亡状态！停止所有导航任务")
            
            # 取消当前导航任务
            if self.is_navigating:
                self.cancel_goal()
                
            # 更新机器人状态
            self.robot_state = "死亡状态，等待复活"
            
            # 发布状态更新
            current_time = time.time()
            game_duration = current_time - self.game_start_time if self.game_has_started else 0
            game_mins = int(game_duration / 60)
            game_secs = int(game_duration % 60)
            self.publish_strategy_status(game_mins, game_secs)
            return
            
        # 检测复活状态
        if self.current_hp > 0 and self.is_dead:
            revival_time = time.time() - self.death_time if self.death_time else 0
            self.revival_count += 1
            self.is_dead = False
            self.get_logger().info(f"哨兵已复活！死亡持续时间: {revival_time:.2f}秒, 第{self.revival_count}次复活")
            
            # 更新机器人状态
            self.robot_state = "已复活，恢复导航"
            
            # 如果比赛已开始，恢复导航行为
            if self.game_has_started:
                # 根据当前血量决定导航目标
                if self.current_hp < self.hp_threshold_critical:
                    self.navigate_to_point(5)  # 血量危急，前往回血点
                    self.robot_state = "复活后血量危急，前往回血点"
                else:
                    self.navigate_to_point(1)  # 血量正常，前往目标点1
                    self.robot_state = "复活后血量正常，前往目标点1"
        
        # 如果处于死亡状态，跳过所有导航逻辑
        if self.is_dead:
            # 即使在死亡状态也更新比赛时间
            current_time = time.time()
            game_duration = current_time - self.game_start_time if self.game_has_started else 0
            game_mins = int(game_duration / 60)
            game_secs = int(game_duration % 60)
            self.publish_strategy_status(game_mins, game_secs)
            return
        
        # ===== 以下是正常导航逻辑 =====
        # 检查比赛是否开始
        if msg.game_progress == 4 and not self.game_has_started:
            self.game_has_started = True
            self.game_start_time = time.time()
            game_time_str = time.strftime('%H:%M:%S', time.localtime(self.game_start_time))
            self.get_logger().info(f"比赛开始! 时间: {game_time_str}")
            
            # 比赛开始，前往目标点1
            self.navigate_to_point(1)
            self.robot_state = "前往目标点1"
            return
            
        # 如果比赛未开始，不进行导航
        if not self.game_has_started:
            return
            
        # 记录比赛时间
        current_time = time.time()
        game_duration = current_time - self.game_start_time
        game_mins = int(game_duration / 60)
        game_secs = int(game_duration % 60)
        
        # 血量变化检测
        if self.last_hp is not None and self.last_hp != self.current_hp:
            hp_change = self.last_hp - self.current_hp
            if hp_change > 0:
                self.get_logger().info(f"血量减少: -{hp_change}! 当前血量: {self.current_hp}")
            else:
                self.get_logger().info(f"血量增加: +{abs(hp_change)}! 当前血量: {self.current_hp}")
        
        # 检查等待时间（如果在目标点2等待）
        if self.waiting_at_point2 and self.target_point2_arrival_time is not None:
            time_since_arrival = current_time - self.target_point2_arrival_time
            
            # 如果在目标点2等待了足够时间，且没有掉血，前往回血点
            if time_since_arrival >= self.point2_wait_time:
                self.waiting_at_point2 = False
                self.get_logger().info(f"在目标点2等待{self.point2_wait_time}秒，前往回血点")
                self.navigate_to_point(5)
                self.robot_state = "从目标点2前往回血点"
                return
        
        # 紧急情况：血量低于危急阈值或弹药量低于阈值，立即前往回血点
        if (self.current_hp < self.hp_threshold_critical or 
            self.current_ammo < self.ammo_threshold_low) and not self.is_healing:
            reason = "血量危急" if self.current_hp < self.hp_threshold_critical else "弹药不足"
            self.get_logger().warn(f"{reason}! 立即前往回血点")
            self.navigate_to_point(5)
            self.robot_state = f"{reason}，紧急前往回血点"
            return
            
        # 在回血点恢复时，检查血量是否恢复到目标值
        if self.is_healing and self.current_hp >= self.hp_threshold_high:
            self.is_healing = False
            self.get_logger().info(f"血量已恢复到{self.current_hp}，前往目标点1")
            self.navigate_to_point(1)
            self.robot_state = "血量恢复完成，返回目标点1"
            return
            
        # 根据血量调整目标点
        if not self.is_navigating and not self.is_healing:
            if self.hp_threshold_medium_low < self.current_hp < self.hp_threshold_medium:
                if self.current_target != 2:
                    self.get_logger().info(f"血量在{self.hp_threshold_medium_low}-{self.hp_threshold_medium}之间，前往目标点2")
                    self.navigate_to_point(2)
                    self.robot_state = "血量适中，前往目标点2"
            
        # 更新上次的血量记录
        self.last_hp = self.current_hp
        
        # 发布当前状态信息
        self.publish_strategy_status(game_mins, game_secs)

    def navigate_to_point(self, point_id):
        """导航到指定目标点"""
        # 取消当前导航（如果有）
        if self.is_navigating:
            self.cancel_goal()
            
        # 设置新的目标点
        self.current_target = point_id
        x, y, yaw = self.target_points[point_id]
        
        # 更新状态
        if point_id == 5:
            self.waiting_at_point2 = False
            
        # 发送导航请求
        self.send_goal(x, y, yaw)

    def publish_status(self):
        """每秒发布导航状态 (1=导航中, 0=未导航)"""
        msg = Int8()
        msg.data = 1 if self.is_navigating else 0
        self.status_publisher.publish(msg)
        
    def publish_strategy_status(self, game_mins, game_secs):
        """发布当前策略和状态信息"""
        status_msg = String()
        
        # 如果处于死亡状态，添加死亡时间信息
        death_info = ""
        if self.is_dead and self.death_time:
            death_duration = time.time() - self.death_time
            death_info = f"死亡时间: {int(death_duration)}秒 | "
        
        status_msg.data = (
            f"比赛时间: {game_mins}分{game_secs}秒 | "
            f"{death_info}"
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
    rclpy.spin(nav_client)  # 让节点一直运行，等待消息
    nav_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


