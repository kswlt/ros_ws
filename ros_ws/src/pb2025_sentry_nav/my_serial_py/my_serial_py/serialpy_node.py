import rclpy  #ros2的python接口
from rclpy.node import Node  # 导入Node类,用于创建节点
from referee_msg.msg import Referee # 导入自定义消息类型，这个是自己写的裁判系统消息类型
from geometry_msgs.msg import Twist # 导入Twist消息类型，用于控制机器人运动
import serial  # 导入串口模块
import json  # 导入json模块
import struct # 导入struct模块,用于打包数据
import threading  # 导入线程模块
import time  # 导入时间模块用于重连延迟

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # 设置串口参数
        self.serial_port = '/dev/ttyUSB0'  # 使用实际存在的串口路径
        self.baud_rate = 115200
        self.get_logger().info(f'Serial port set to: {self.serial_port}')

        # 初始化串口
        self.serial_conn = None
        self.is_connected = False
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5
        self.reconnect_delay = 2.0  # 重连延迟时间（秒）
        
        # 连接串口
        self.connect_serial()
        
        # 创建订阅者，订阅导航数据话题，把计算好的数据发给单片机
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.SendtoSTM32_callback, 10)

        # 创建发布者,将接受到的来自单片机的数据发布到/stm32_ros2_data话题
        self.publisher_ = self.create_publisher(Referee, 'stm32_ros2_data', 10)

        # 创建定时器，定期读取串口数据
        self.timer = self.create_timer(0.1, self.read_serial_data)
        
        # 创建重连定时器（初始不激活）
        self.reconnect_timer = None

    def connect_serial(self):
        """连接串口或重新连接"""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
                
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f'Connected to {self.serial_port} at {self.baud_rate} baud rate.')
            self.is_connected = True
            self.reconnect_attempts = 0
            
            # 清除可能存在的重连定时器
            if self.reconnect_timer:
                self.destroy_timer(self.reconnect_timer)
                self.reconnect_timer = None
                
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {self.serial_port}: {e}')
            self.is_connected = False
            self.reconnect_attempts += 1
            
            if self.reconnect_attempts < self.max_reconnect_attempts:
                self.get_logger().info(f'Attempting to reconnect ({self.reconnect_attempts}/{self.max_reconnect_attempts}) in {self.reconnect_delay} seconds...')
                # 创建重连定时器
                self.reconnect_timer = self.create_timer(self.reconnect_delay, self.connect_serial)
            else:
                self.get_logger().warning(f'Max reconnect attempts ({self.max_reconnect_attempts}) reached. Will continue trying to operate without serial.')

    def read_serial_data(self):
        """读取串口数据并处理"""
        if not self.is_connected or not self.serial_conn or not self.serial_conn.is_open:
            return
            
        try:
            data = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
            if data:
                try:
                    # 尝试解析JSON数据
                    parsed_data = json.loads(data)
                    self.process_data(parsed_data)
                except (json.JSONDecodeError, ValueError, TypeError) as e:
                    self.get_logger().error(f'Failed to parse JSON: {e}')
        except serial.SerialException as e:
            self.get_logger().error(f'Error reading serial data: {e}')
            # 尝试重连
            self.is_connected = False
            if not self.reconnect_timer:
                self.reconnect_attempts = 0  # 重置重试计数
                self.reconnect_timer = self.create_timer(self.reconnect_delay, self.connect_serial)

    def safe_int_convert(self, value, default=0, min_val=0, max_val=255):
        """安全地将值转换为指定范围内的整数"""
        try:
            if value is None:
                return default
                
            int_val = int(value)
            
            # 限制在有效范围内
            if int_val < min_val:
                self.get_logger().warning(f'Value {int_val} below range [{min_val}, {max_val}], using {min_val}')
                return min_val
            elif int_val > max_val:
                self.get_logger().warning(f'Value {int_val} exceeds range [{min_val}, {max_val}], using {max_val}')
                return max_val
                
            return int_val
        except (ValueError, TypeError) as e:
            self.get_logger().warning(f'Failed to convert "{value}" to int: {e}, using default {default}')
            return default

    def process_data(self, data):
        """处理解析后的数据，增强错误处理能力"""
        try:
            # 创建消息
            msg = Referee()
            
            # 使用安全转换方法处理所有字段
            # unsigned int 8 (0-255)
            msg.game_type = self.safe_int_convert(data.get('game_type'), 0, 0, 255)
            msg.game_progress = self.safe_int_convert(data.get('game_progress'), 0, 0, 255)
            
            # int 16 (-32768 to 32767)
            msg.remain_hp = self.safe_int_convert(data.get('remain_hp'), 0, -32768, 32767)
            msg.max_hp = self.safe_int_convert(data.get('max_hp'), 600, 0, 32767)
            
            # int 16
            msg.stage_remain_time = self.safe_int_convert(data.get('stage_remain_time'), 0, -32768, 32767)
            msg.bullet_remaining_num_17mm = self.safe_int_convert(data.get('bullet_remaining_num_17mm'), 0, 0, 32767)
            
            # int 16
            msg.red_outpost_hp = self.safe_int_convert(data.get('red_outpost_hp'), 0, 0, 32767)
            msg.red_base_hp = self.safe_int_convert(data.get('red_base_hp'), 0, 0, 32767)
            msg.blue_outpost_hp = self.safe_int_convert(data.get('blue_outpost_hp'), 0, 0, 32767)
            msg.blue_base_hp = self.safe_int_convert(data.get('blue_base_hp'), 0, 0, 32767)
            
            # 射击热量 (unsigned int 16: 0-65535)
            msg.shooter_heat_17mm = self.safe_int_convert(data.get('shooter_heat_17mm'), 0, 0, 65535)
            
            # unsigned int 8
            msg.rfid_status = self.safe_int_convert(data.get('rfid_status'), 0, 0, 255)
            
            # 发布消息
            self.publisher_.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in process_data: {e}')
            # 即使发生错误也不退出

    def SendtoSTM32_callback(self, msg):
        """接收来自ROS2的指令，并发送给单片机"""
        if not self.is_connected or not self.serial_conn or not self.serial_conn.is_open:
            self.get_logger().warning('Cannot send to STM32: Serial connection not open')
            return
            
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
            # 串口异常，标记为断开连接并尝试重连
            self.is_connected = False
            if not self.reconnect_timer:
                self.reconnect_attempts = 0
                self.reconnect_timer = self.create_timer(self.reconnect_delay, self.connect_serial)
        except Exception as e:
            self.get_logger().error(f'General error in SendtoSTM32_callback: {e}')

    def __del__(self):
        """析构函数，确保关闭串口"""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
                self.get_logger().info(f'Serial connection to {self.serial_port} closed.')
            except Exception as e:
                self.get_logger().error(f'Error closing serial port: {e}')

def ros_spin_thread(node):
    """ROS2 消息循环线程"""
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Exception in ROS spin thread: {e}')
    finally:
        # 确保关闭串口连接
        if node.serial_conn and node.serial_conn.is_open:
            try:
                node.serial_conn.close()
                node.get_logger().info('Serial connection closed due to thread exit.')
            except:
                pass

def main(args=None):
    """主函数"""
    try:
        rclpy.init(args=args)
        serial_node = SerialNode()
        
        # 创建并启动消息处理线程
        spin_thread = threading.Thread(target=ros_spin_thread, args=(serial_node,))
        spin_thread.daemon = True  # 将线程设为守护线程，主线程结束时自动结束
        spin_thread.start()
        
        # 主线程等待，可以在这里添加额外的错误处理或信号处理
        while rclpy.ok():
            try:
                time.sleep(1.0)  # 简单等待
            except KeyboardInterrupt:
                break
            
    except Exception as e:
        print(f'Error in main: {e}')
    finally:
        # 清理资源
        if 'serial_node' in locals():
            try:
                serial_node.destroy_node()
            except:
                pass
        try:
            rclpy.shutdown()
        except:
            pass
        print('Node shut down cleanly')

if __name__ == '__main__':
    main()