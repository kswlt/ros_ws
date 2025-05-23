import rclpy  #ros2的python接口
from rclpy.node import Node  # 导入Node类,用于创建节点
from referee_msg.msg import Referee # 导入自定义消息类型，这个是自己写的裁判系统消息类型
from geometry_msgs.msg import Twist # 导入Twist消息类型，用于控制机器人运动
import serial  # 导入串口模块
import json  # 导入json模块
import struct # 导入struct模块,用于打包数据
import threading  # 导入线程模块
from std_msgs.msg import Int8  # 状态消息类型
class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # 设置串口参数
        self.serial_port = '/dev/ttyUSB0'  # 使用实际存在的串口路径
        self.baud_rate = 115200
        self.get_logger().info(f'Serial port set to: {self.serial_port}')
        self.Status_nav2 = 0
        # 初始化串口
        self.serial_conn = None
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f'Connected to {self.serial_port} at {self.baud_rate} baud rate.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {self.serial_port}: {e}')
            # 错误处理
            self.destroy_node()
            rclpy.shutdown()
        # 创建订阅者，订阅导航数据话题，把计算好的数据发给单片机
        self.subscription = self.create_subscription(Twist, '/red_standard_robot1/cmd_vel', self.SendtoSTM32_callback, 10)
        self.subscription_1 = self.create_subscription(Int8, 'nav2_status',self.Nav2Stat_callback,10)
        # 创建发布者,将接受到的来自单片机的数据发布到/stm32_ros2_data话题
        self.publisher_ = self.create_publisher(Referee, 'stm32_ros2_data', 10)

        # 创建定时器，定期读取串口数据
        self.timer = self.create_timer(0.1, self.read_serial_data)

    def read_serial_data(self):
        if self.serial_conn and self.serial_conn.is_open:
            try:
                data = self.serial_conn.readline().decode('utf-8',errors='ignore').strip()
                if data:
                    try:
                        # 尝试解析JSON数据
                        parsed_data = json.loads(data)
                        self.process_data(parsed_data)
                    except (json.JSONDecodeError, ValueError, TypeError) as e:
                        self.get_logger().error(f'Failed to parse JSON: {e}')
                        return
            except serial.SerialException as e:
                self.get_logger().error(f'Error reading serial data: {e}')
        else:
            self.get_logger().warning('Serial connection is not open.')

    def process_data(self, data):
        # 处理解析后的数据，根据实际需求进行相应操作
        msg = Referee()
        msg.game_type = int(data.get('game_type'))#比赛类型
        msg.game_progress = int(data.get('game_progress'))#比赛阶段——4 比赛进行中
        msg.remain_hp = int(data.get('remain_hp'))#机器人当前血量
        msg.max_hp = int(data.get('max_hp'))#。。。
        msg.stage_remain_time = int(data.get('stage_remain_time'))#当前阶段剩余时间，                     
        msg.bullet_remaining_num_17mm = int(data.get('bullet_remaining_num_17mm'))#剩余发弹量
        msg.red_outpost_hp = int(data.get('red_outpost_hp'))    
        msg.red_base_hp = int(data.get('red_base_hp'))
        msg.blue_outpost_hp = int(data.get('blue_outpost_hp'))
        msg.blue_base_hp = int(data.get('blue_base_hp'))
        msg.rfid_status = int(data.get('rfid_status'))#rfid状态
        # 发布消息
        self.publisher_.publish(msg)
    def Nav2Stat_callback(self,msg):
         self.Status_nav2 = msg.data
    def SendtoSTM32_callback(self, msg):
        # 接收来自ROS2的指令，并发送给单片机
        if self.serial_conn and self.serial_conn.is_open:
            try:
                # 数据字段定义
                header = 0xAA
                checksum = 19
                x_speed = msg.linear.x *0.5
                y_speed = msg.linear.y *0.5
                rotate = msg.angular.z
                yaw_speed = msg.angular.z
                running_state = 0x00
                data_frame = struct.pack(
                    '<BBffffB',  # 格式化字符串：<表示小端，B表示uint8_t，f表示float
                    header,         # uint8_t
                    checksum,       # uint8_t
                    x_speed,        # float
                    y_speed,        # float
                    rotate,         # float
                    yaw_speed,      # float
                    running_state   # uint8_t
                )
                # 发送数据
                self.serial_conn.write(data_frame)
                self.get_logger().info('Sent data to STM32')
            except serial.SerialException as e:
                self.get_logger().error(f'Error sending data to STM32: {e}')
        else:
            self.get_logger().warning('Serial connection is not open.')

    def __del__(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info(f'Serial connection to {self.serial_port} closed.')

def ros_spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    spin_thread = threading.Thread(target=ros_spin_thread, args=(serial_node,))
    spin_thread.start()
    spin_thread.join()
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()