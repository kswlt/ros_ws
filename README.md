# 待办事项

# 血量归零时防止进恢复行为的bug  
# 买弹丸测试  
# 不要在补给区待超过10秒  
# 上坡停止小陀螺         
# 只用栅格地图导航   
# 建图雷达数据  
# 上场开导航的屏幕和键盘拓展坞多准备  
# 开导航时候遥控开不开  
# 哨兵复活机制  


导航失败重试机制改进 (P6)：

当前的重试是固定5秒延迟，无限次重试。
建议：
引入最大重试次数。
使用指数退避（Exponential Backoff）的重试延迟。
在多次重试失败后，可以考虑进入一个“安全模式”（例如，停在原地，或者尝试导航到一个非常安全的已知点），并发出更严重的警告。
可以创建一个辅助函数来处理重试调度，以避免 goal_result_callback 过于复杂。












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











# 老串口通信备份
import rclpy  #ros2的python接口
from rclpy.node import Node  # 导入Node类,用于创建节点
from referee_msg.msg import Referee # 导入自定义消息类型，这个是自己写的裁判系统消息类型
from geometry_msgs.msg import Twist # 导入Twist消息类型，用于控制机器人运动
import serial  # 导入串口模块
import json  # 导入json模块
import struct # 导入struct模块,用于打包数据
import threading  # 导入线程模块

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # 设置串口参数
        self.serial_port = '/dev/ttyUSB0'  # 使用实际存在的串口路径
        self.baud_rate = 115200
        self.get_logger().info(f'Serial port set to: {self.serial_port}')

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
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.SendtoSTM32_callback, 10)

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
        msg.game_type = int(data.get('game_type'))
        msg.game_progress = int(data.get('game_progress'))
        msg.remain_hp = int(data.get('remain_hp'))
        msg.max_hp = int(data.get('max_hp'))
        msg.stage_remain_time = int(data.get('stage_remain_time'))                     
        msg.bullet_remaining_num_17mm = int(data.get('bullet_remaining_num_17mm'))
        msg.red_outpost_hp = int(data.get('red_outpost_hp'))    
        msg.red_base_hp = int(data.get('red_base_hp'))
        msg.blue_outpost_hp = int(data.get('blue_outpost_hp'))
        msg.blue_base_hp = int(data.get('blue_base_hp'))
        msg.rfid_status = int(data.get('rfid_status'))
        # 发布消息
        self.publisher_.publish(msg)
    def SendtoSTM32_callback(self, msg):
        # 接收来自ROS2的指令，并发送给单片机
        if self.serial_conn and self.serial_conn.is_open:
            try:
                # 数据字段定义
                header = 0xAA
                checksum = 19
                x_speed = msg.linear.x
                y_speed = msg.linear.y
                rotate = msg.angular.z
                yaw_speed = msg.angular.z
                running_state = 0x01
                # 打包数据
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





















































































    [serial_node-1] [INFO] [1748016571.815845965] [serial]: Sent data to STM32
[serial_node-1] [ERROR] [1748016571.838230366] [serial]: Failed to parse JSON: Expecting ',' delimiter: line 1 column 15 (char 14)
[serial_node-1] [INFO] [1748016571.865912373] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016571.916220633] [serial]: Sent data to STM32
[serial_node-1] [ERROR] [1748016571.939790568] [serial]: Failed to parse JSON: Expecting ':' delimiter: line 1 column 164 (char 163)
[serial_node-1] [INFO] [1748016571.966023655] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016572.017820900] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016572.068569442] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016572.116128485] [serial]: Sent data to STM32
[serial_node-1] [ERROR] [1748016572.138236153] [serial]: Failed to parse JSON: Expecting ':' delimiter: line 1 column 163 (char 162)
[serial_node-1] [INFO] [1748016572.167150438] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016572.215891714] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016572.266092512] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016572.315987144] [serial]: Sent data to STM32
[serial_node-1] [ERROR] [1748016572.339006685] [serial]: Failed to parse JSON: Expecting property name enclosed in double quotes: line 1 column 168 (char 167)
[serial_node-1] [INFO] [1748016572.366389488] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016572.416106065] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016572.466026380] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016572.516211617] [serial]: Sent data to STM32
[serial_node-1] [ERROR] [1748016572.538237843] [serial]: Failed to parse JSON: Expecting ':' delimiter: line 1 column 152 (char 151)
[serial_node-1] [INFO] [1748016572.566180085] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016572.615995655] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016572.666092952] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016572.715990204] [serial]: Sent data to STM32
[serial_node-1] [ERROR] [1748016572.739469682] [serial]: Failed to parse JSON: Expecting ':' delimiter: line 1 column 162 (char 161)
[serial_node-1] [INFO] [1748016572.766191000] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016572.815804134] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016572.866932370] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016572.916000206] [serial]: Sent data to STM32
[serial_node-1] [ERROR] [1748016572.939261478] [serial]: Failed to parse JSON: Expecting ':' delimiter: line 1 column 162 (char 161)
[serial_node-1] [INFO] [1748016572.966823081] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016573.016071154] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016573.066028876] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016573.116386398] [serial]: Sent data to STM32
[serial_node-1] [ERROR] [1748016573.138970895] [serial]: Failed to parse JSON: Expecting ':' delimiter: line 1 column 162 (char 161)
[serial_node-1] [INFO] [1748016573.168220751] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016573.216090038] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016573.265829502] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016573.316367828] [serial]: Sent data to STM32
[serial_node-1] [ERROR] [1748016573.339244862] [serial]: Failed to parse JSON: Expecting ':' delimiter: line 1 column 164 (char 163)
[serial_node-1] [INFO] [1748016573.365961546] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016573.416159962] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016573.465937947] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016573.516451353] [serial]: Sent data to STM32
[serial_node-1] [ERROR] [1748016573.539536367] [serial]: Failed to parse JSON: Expecting ':' delimiter: line 1 column 163 (char 162)
[serial_node-1] [INFO] [1748016573.566032040] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016573.616070571] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016573.666766201] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016573.716075545] [serial]: Sent data to STM32
[serial_node-1] [ERROR] [1748016573.737797107] [serial]: Failed to parse JSON: Expecting value: line 1 column 14 (char 13)
[serial_node-1] [INFO] [1748016573.766494411] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016573.815907097] [serial]: Sent data to STM32
[serial_node-1] [ERROR] [1748016573.838560012] [serial]: Failed to parse JSON: Expecting ':' delimiter: line 1 column 164 (char 163)
[serial_node-1] [INFO] [1748016573.866047512] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016573.916110974] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016573.966237743] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016574.016087787] [serial]: Sent data to STM32
[serial_node-1] [ERROR] [1748016574.038234640] [serial]: Failed to parse JSON: Expecting ',' delimiter: line 1 column 366 (char 365)
[serial_node-1] [INFO] [1748016574.065928014] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016574.118363418] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016574.165979080] [serial]: Sent data to STM32
[serial_node-1] [INFO] [1748016574.215935616] [serial]: Sent data to STM32
[serial_node-1] [ERROR] [1748016574.238228532] [serial]: Failed to parse JSON: Expecting ':' delimiter: line 1 column 152 (char 151)
[serial_node-1] [ERROR] [1748016574.438318965] [serial]: Failed to parse JSON: Expecting ':' delimiter: line 1 column 163 (char 162)
[serial_node-1] [ERROR] [1748016574.638284913] [serial]: Failed to parse JSON: Expecting ':' delimiter: line 1 column 172 (char 171)
[serial_node-1] [ERROR] [1748016574.838205058] [serial]: Failed to parse JSON: Expecting ':' delimiter: line 1 column 152 (char 151)
[serial_node-1] [ERROR] [1748016575.038437514] [serial]: Failed to parse JSON: Expecting property name enclosed in double quotes: line 1 column 168 (char 167)
[serial_node-1] [ERROR] [1748016575.238687985] [serial]: Failed to parse JSON: Expecting ':' delimiter: line 1 column 151 (char 150)
[serial_node-1] [ERROR] [1748016575.438202769] [serial]: Failed to parse JSON: Expecting ':' delimiter: line 1 column 163 (char 162)
[serial_node-1] Exception in thread Thread-1 (ros_spin_thread):
[serial_node-1] Traceback (most recent call last):
[serial_node-1]   File "/usr/lib/python3.10/threading.py", line 1016, in _bootstrap_inner
[serial_node-1]     self.run()
[serial_node-1]   File "/usr/lib/python3.10/threading.py", line 953, in run
[serial_node-1]     self._target(*self._args, **self._kwargs)
[serial_node-1]   File "/home/adam/ros_ws/build/my_serial_py/my_serial_py/serialpy_node.py", line 109, in ros_spin_thread
[serial_node-1]     rclpy.spin(node)
[serial_node-1]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 226, in spin
[serial_node-1]     executor.spin_once()
[serial_node-1]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 751, in spin_once
[serial_node-1]     self._spin_once_impl(timeout_sec)
[serial_node-1]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 748, in _spin_once_impl
[serial_node-1]     raise handler.exception()
[serial_node-1]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/task.py", line 254, in __call__
[serial_node-1]     self._handler.send(None)
[serial_node-1]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 447, in handler
[serial_node-1]     await call_coroutine(entity, arg)
[serial_node-1]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 361, in _execute_timer
[serial_node-1]     await await_or_execute(tmr.callback)
[serial_node-1]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 107, in await_or_execute
[serial_node-1]     return callback(*args)
[serial_node-1]   File "/home/adam/ros_ws/build/my_serial_py/my_serial_py/serialpy_node.py", line 46, in read_serial_data
[serial_node-1]     self.process_data(parsed_data)
[serial_node-1]   File "/home/adam/ros_ws/build/my_serial_py/my_serial_py/serialpy_node.py", line 58, in process_data
[serial_node-1]     msg.game_type = int(data.get('game_type'))#比赛类型
[serial_node-1]   File "/home/adam/ros_ws/install/referee_msg/local/lib/python3.10/dist-packages/referee_msg/msg/_referee.py", line 292, in game_type
[serial_node-1]     assert value >= 0 and value < 256, \
[serial_node-1] AssertionError: The 'game_type' field must be an unsigned integer in [0, 255]
[serial_node-1] [INFO] [1748016575.670356495] [serial]: Serial connection to /dev/ttyUSB0 closed.
[INFO] [serial_node-1]: process has finished cleanly [pid 6291]
