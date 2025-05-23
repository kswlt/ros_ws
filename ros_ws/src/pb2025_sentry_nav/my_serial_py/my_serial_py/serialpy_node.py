import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter # 用于声明参数
from rcl_interfaces.msg import SetParametersResult # 用于参数回调 (可选)

from referee_msg.msg import Referee
from geometry_msgs.msg import Twist
import serial
import json
import struct
import threading
import time

# 定义默认常量，方便管理和修改
DEFAULT_SERIAL_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD_RATE = 115200
DEFAULT_SERIAL_TIMEOUT = 1.0  # 串口读写超时时间 (s)
DEFAULT_READ_TIMER_PERIOD = 0.05  # 读取串口数据的周期 (s)
DEFAULT_RECONNECT_DELAY = 3.0 # 串口断开后尝试重连的延时 (s)
DEFAULT_ENCODING = 'utf-8'    # STM32发送JSON数据时使用的编码

# STM32通信协议相关常量 (与STM32端对应)
STM32_FRAME_HEADER = 0xAA         # 发送给STM32的二进制帧头
STM32_RUNNING_STATE_DEFAULT = 0x01 # 发送给STM32的默认运行状态
STM32_PACK_FORMAT = '<BBffffB'    # Python -> STM32 的 struct.pack 格式
                                  # < 小端字节序
                                  # B: uint8_t (header)
                                  # B: uint8_t (checksum)
                                  # f: float (x_speed)
                                  # f: float (y_speed)
                                  # f: float (rotate_value)
                                  # f: float (yaw_speed_value)
                                  # B: uint8_t (running_state)
STM32_EXPECTED_FRAME_LENGTH = struct.calcsize(STM32_PACK_FORMAT) # 预期发送的帧长度

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # --- 1. 参数化配置 ---
        self.declare_parameter('serial_port', DEFAULT_SERIAL_PORT)
        self.declare_parameter('baud_rate', DEFAULT_BAUD_RATE)
        self.declare_parameter('serial_timeout', DEFAULT_SERIAL_TIMEOUT)
        self.declare_parameter('read_timer_period', DEFAULT_READ_TIMER_PERIOD)
        self.declare_parameter('reconnect_delay', DEFAULT_RECONNECT_DELAY)
        self.declare_parameter('stm32_encoding', DEFAULT_ENCODING)
        self.declare_parameter('twist_topic', '/cmd_vel') # 订阅的Twist话题
        self.declare_parameter('referee_topic', 'stm32_ros2_data') # 发布的Referee数据话题

        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.serial_timeout = self.get_parameter('serial_timeout').get_parameter_value().double_value
        self.read_timer_period = self.get_parameter('read_timer_period').get_parameter_value().double_value
        self.reconnect_delay = self.get_parameter('reconnect_delay').get_parameter_value().double_value
        self.stm32_encoding = self.get_parameter('stm32_encoding').get_parameter_value().string_value
        twist_topic_name = self.get_parameter('twist_topic').get_parameter_value().string_value
        referee_topic_name = self.get_parameter('referee_topic').get_parameter_value().string_value

        self.get_logger().info(
            f"Serial port: {self.serial_port}, Baud rate: {self.baud_rate}, "
            f"Timeout: {self.serial_timeout}s, Encoding: {self.stm32_encoding}"
        )

        self.serial_conn = None
        self.reconnect_timer = None # 用于串口重连的定时器
        self.shutdown_requested = threading.Event() # 用于通知线程安全退出的事件

        # --- 2. 健壮的串口连接管理 ---
        if not self._initialize_serial():
            self.get_logger().warn("Initial serial port connection failed. Will attempt to reconnect periodically.")
            self._schedule_reconnect()

        # 创建订阅者
        self.subscription = self.create_subscription(
            Twist,
            twist_topic_name,
            self._send_to_stm32_callback,
            10
        )

        # 创建发布者
        self.publisher_ = self.create_publisher(Referee, referee_topic_name, 10)

        # 创建定时器，定期读取串口数据
        self.read_data_timer = self.create_timer(self.read_timer_period, self._read_serial_data_robust)

        self.get_logger().info("SerialNode initialized successfully.")

    def _initialize_serial(self) -> bool:
        """尝试初始化或重新初始化串口连接。返回True表示成功，False表示失败。"""
        if self.serial_conn and self.serial_conn.is_open:
            self.get_logger().debug("Serial connection is already open.")
            return True
        try:
            if self.serial_conn: # 如果存在旧的连接对象，先尝试关闭
                self.serial_conn.close()
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.serial_timeout,
                write_timeout=self.serial_timeout # 为写入也设置超时
            )
            self.get_logger().info(f'Successfully connected to serial port {self.serial_port}.')
            if self.reconnect_timer is not None: # 如果是通过重连定时器成功的
                self.destroy_timer(self.reconnect_timer) # 销毁一次性重连定时器
                self.reconnect_timer = None
                self.get_logger().info("Reconnect timer destroyed after successful connection.")
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {self.serial_port}: {e}')
            self.serial_conn = None # 确保连接对象为None
            return False
        except Exception as e: # 捕获其他潜在的初始化异常
            self.get_logger().error(f'An unexpected error occurred during serial initialization: {e}', exc_info=True)
            self.serial_conn = None
            return False

    def _schedule_reconnect(self):
        """安排串口重连尝试（如果当前没有正在进行的重连）。"""
        if self.shutdown_requested.is_set():
            self.get_logger().info("Shutdown requested, not scheduling reconnect.")
            return

        if self.reconnect_timer is None or self.reconnect_timer.is_canceled():
            self.get_logger().info(f"Scheduling serial port reconnect in {self.reconnect_delay} seconds.")
            self.reconnect_timer = self.create_timer(self.reconnect_delay, self._attempt_reconnect_callback)
        else:
            self.get_logger().debug("Reconnect attempt already scheduled.")

    def _attempt_reconnect_callback(self):
        """重连定时器的回调函数。"""
        if self.shutdown_requested.is_set():
            self.get_logger().info("Shutdown requested, aborting reconnect attempt.")
            if self.reconnect_timer:
                self.destroy_timer(self.reconnect_timer)
                self.reconnect_timer = None
            return

        self.get_logger().info("Attempting to reconnect to serial port via timer...")
        if self._initialize_serial():
            self.get_logger().info("Successfully reconnected to serial port via timer.")
            # _initialize_serial 内部会处理定时器的取消
        else:
            self.get_logger().warn("Failed to reconnect via timer. Will retry if an operation fails or timer is periodic.")
            # 定时器是一次性的，如果失败，下一次读写失败时会再次调用 _schedule_reconnect
            if self.reconnect_timer:
                self.destroy_timer(self.reconnect_timer) # 确保旧定时器销毁
                self.reconnect_timer = None
            self._schedule_reconnect() # 再次安排重连

    def _read_serial_data_robust(self):
        """更健壮地读取和处理从STM32发送过来的JSON数据。"""
        if self.shutdown_requested.is_set():
            return
        if not self.serial_conn or not self.serial_conn.is_open:
            self.get_logger().warn('Serial connection not open for reading. Scheduling reconnect.')
            self._schedule_reconnect()
            return

        try:
            if self.serial_conn.in_waiting > 0:
                line_bytes = self.serial_conn.readline() # 读取一行数据，直到'\n'或超时
                if not line_bytes: # 超时或只读到空字节串
                    # self.get_logger().debug("Readline returned empty bytes (timeout or no newline).")
                    return

                try:
                    # --- 3. 增强的串口数据读取 (严格解码) ---
                    data_str = line_bytes.decode(self.stm32_encoding, errors='strict').strip()
                except UnicodeDecodeError as e:
                    self.get_logger().error(
                        f"UnicodeDecodeError while decoding serial data: {e}. "
                        f"Raw data (hex): {line_bytes.hex()}. Encoding used: {self.stm32_encoding}"
                    )
                    self.serial_conn.reset_input_buffer() # 清空输入缓冲区，防止错误数据累积
                    return

                if data_str: # 解码并去除空白后仍有数据
                    self.get_logger().debug(f"Received raw string for JSON parsing: '{data_str}'")
                    try:
                        # --- 4. 健壮的JSON解析 ---
                        parsed_data = json.loads(data_str)
                        self._process_incoming_json_data(parsed_data)
                    except json.JSONDecodeError as e:
                        self.get_logger().error(
                            f'JSONDecodeError: {e}. Malformed JSON string received: "{data_str}"'
                        )
                        # 可以考虑记录更详细的错误，或者特定的错误处理逻辑
                # else:
                    # self.get_logger().debug("Received an empty string after decode and strip.")
            # else: self.get_logger().debug("No data in serial waiting.")

        except serial.SerialTimeoutException as e: # 特定于写入超时
            self.get_logger().warn(f"Serial timeout during read: {e}")
        except serial.SerialException as e:
            self.get_logger().error(f'SerialException during read: {e}. Closing connection and scheduling reconnect.')
            self._close_serial_connection()
            self._schedule_reconnect()
        except IOError as e: # 更通用的IO错误，例如设备断开
             self.get_logger().error(f'IOError during read (e.g., device disconnected): {e}')
             self._close_serial_connection()
             self._schedule_reconnect()
        except Exception as e: # 捕获所有其他意外错误
            self.get_logger().error(f'Unexpected error in _read_serial_data_robust: {e}', exc_info=True)
            self._close_serial_connection() # 发生未知错误也尝试重置连接
            self._schedule_reconnect()

    def _process_incoming_json_data(self, parsed_json_data: dict):
        """处理从STM32接收并已解析的JSON数据，填充并发布Referee消息。"""
        try:
            msg = Referee()
            # 使用 .get(key, default_value) 并进行类型转换，提供默认值以增强鲁棒性
            msg.game_type = int(parsed_json_data.get('game_type', 0))
            msg.game_progress = int(parsed_json_data.get('game_progress', 0))
            msg.remain_hp = int(parsed_json_data.get('remain_hp', 0))
            msg.max_hp = int(parsed_json_data.get('max_hp', 0))
            msg.stage_remain_time = int(parsed_json_data.get('stage_remain_time', 0))
            msg.bullet_remaining_num_17mm = int(parsed_json_data.get('bullet_remaining_num_17mm', 0))
            msg.red_outpost_hp = int(parsed_json_data.get('red_outpost_hp', 0))
            msg.red_base_hp = int(parsed_json_data.get('red_base_hp', 0))
            msg.blue_outpost_hp = int(parsed_json_data.get('blue_outpost_hp', 0))
            msg.blue_base_hp = int(parsed_json_data.get('blue_base_hp', 0))
            msg.rfid_status = int(parsed_json_data.get('rfid_status', 0)) # 裁判系统协议中此字段为uint32

            self.publisher_.publish(msg)
            self.get_logger().debug(f"Published Referee data: game_progress={msg.game_progress}, hp={msg.remain_hp}")
        except KeyError as e: # 仅当 .get() 未提供默认值且键不存在时才会发生
            self.get_logger().error(f"KeyError: Missing key {e} in parsed JSON data: {parsed_json_data}")
        except (ValueError, TypeError) as e: # 类型转换失败
            self.get_logger().error(f"Data type error when processing JSON field: {e}. Data: {parsed_json_data}")
        except Exception as e: # 捕获其他意外错误
            self.get_logger().error(f"Unexpected error in _process_incoming_json_data: {e}", exc_info=True)

    def _calculate_xor_checksum(self, data_bytes: bytes) -> int:
        """计算字节串的异或校验和。"""
        checksum = 0
        for byte_val in data_bytes:
            checksum ^= byte_val
        return checksum & 0xFF # 确保是单字节

    def _send_to_stm32_callback(self, msg: Twist):
        """接收Twist指令，打包并添加校验和后发送给STM32。"""
        if self.shutdown_requested.is_set():
            return
        if not self.serial_conn or not self.serial_conn.is_open:
            self.get_logger().warn('Serial connection not open. Cannot send Twist data to STM32. Scheduling reconnect.')
            self._schedule_reconnect()
            return

        try:
            header = STM32_FRAME_HEADER
            x_speed = float(msg.linear.x)
            y_speed = float(msg.linear.y)
            # 假设STM32端期望的 rotate 和 yaw_speed 都来自ROS Twist消息的angular.z
            # 如果不是，需要根据实际情况修改数据来源
            rotate_value = float(msg.angular.z)
            yaw_speed_value = float(msg.angular.z)
            running_state = STM32_RUNNING_STATE_DEFAULT # uint8_t

            # --- 5. 动态校验和 (针对发送给STM32的二进制数据) ---
            # 准备用于计算校验和的数据负载 (不包括帧头和校验和本身)
            # 顺序应与 STM32_PACK_FORMAT 中校验和之后的部分一致
            payload_tuple = (x_speed, y_speed, rotate_value, yaw_speed_value, running_state)
            # 注意：这里的'<ffffB' 应该与 STM32_PACK_FORMAT 中对应部分一致
            packed_payload_for_checksum = struct.pack('<ffffB', *payload_tuple)
            checksum = self._calculate_xor_checksum(packed_payload_for_checksum)

            data_frame = struct.pack(
                STM32_PACK_FORMAT, # '<BBffffB'
                header,
                checksum,
                x_speed,
                y_speed,
                rotate_value,
                yaw_speed_value,
                running_state
            )

            if len(data_frame) != STM32_EXPECTED_FRAME_LENGTH:
                self.get_logger().error(
                    f"Packed data frame length ({len(data_frame)}) does not match "
                    f"expected length ({STM32_EXPECTED_FRAME_LENGTH}). Aborting send."
                )
                return

            bytes_written = self.serial_conn.write(data_frame)
            if bytes_written is None or bytes_written != len(data_frame): # write_timeout可能导致返回None或不足量
                 self.get_logger().warn(
                     f"Serial write operation may have failed or timed out. "
                     f"Attempted to write {len(data_frame)} bytes, but write() returned: {bytes_written}."
                 )
                 # 发生写入不足或超时，可能连接已出问题
                 self._close_serial_connection()
                 self._schedule_reconnect()
                 return

            self.get_logger().debug(
                f'Sent {bytes_written} bytes to STM32. Header: {header:02X}, Checksum: {checksum:02X}, '
                f'X: {x_speed:.2f}, Y: {y_speed:.2f}, Rot: {rotate_value:.2f}, Yaw: {yaw_speed_value:.2f}, State: {running_state}'
            )

        except serial.SerialTimeoutException as e: # 特定于写入超时
            self.get_logger().warn(f"Serial timeout during write: {e}. Closing and scheduling reconnect.")
            self._close_serial_connection()
            self._schedule_reconnect()
        except serial.SerialException as e:
            self.get_logger().error(f'SerialException sending Twist data to STM32: {e}. Closing and scheduling reconnect.')
            self._close_serial_connection()
            self._schedule_reconnect()
        except struct.error as e: # struct.pack 打包错误
            self.get_logger().error(f"Struct packing error for Twist data: {e}. Input Twist: {msg}")
        except Exception as e: # 捕获所有其他意外错误
            self.get_logger().error(f'Unexpected error in _send_to_stm32_callback: {e}', exc_info=True)
            self._close_serial_connection() # 发生未知错误也尝试重置连接
            self._schedule_reconnect()

    def _close_serial_connection(self):
        """安全地关闭串口连接。"""
        if self.serial_conn:
            if self.serial_conn.is_open:
                try:
                    self.serial_conn.reset_input_buffer()  # 清空输入缓冲区
                    self.serial_conn.reset_output_buffer() # 清空输出缓冲区
                    self.serial_conn.close()
                    self.get_logger().info(f'Serial connection to {self.serial_port} closed.')
                except Exception as e: # pyserial close() 很少抛异常，但以防万一
                    self.get_logger().error(f"Exception closing serial port: {e}", exc_info=True)
            self.serial_conn = None # 无论是否打开或关闭是否成功，都清除引用

    def _on_shutdown(self):
        """节点关闭时执行的清理操作。"""
        self.get_logger().info("Node shutdown sequence started...")
        self.shutdown_requested.set() # 通知所有循环和定时器回调停止

        if hasattr(self, 'read_data_timer') and self.read_data_timer:
            if not self.read_data_timer.is_canceled():
                self.read_data_timer.cancel()
            self.get_logger().info("Read data timer cancelled.")

        if hasattr(self, 'reconnect_timer') and self.reconnect_timer:
            if not self.reconnect_timer.is_canceled():
                self.reconnect_timer.cancel()
            self.get_logger().info("Reconnect timer cancelled.")

        self._close_serial_connection()
        self.get_logger().info("SerialNode shutdown cleanup complete.")

# --- 6. 健壮的节点关闭逻辑 ---
def main(args=None):
    rclpy.init(args=args)
    serial_node_instance = None
    try:
        serial_node_instance = SerialNode()
        rclpy.spin(serial_node_instance)
    except KeyboardInterrupt:
        if serial_node_instance:
            serial_node_instance.get_logger().info('KeyboardInterrupt (Ctrl+C) received, shutting down.')
        else:
            print('KeyboardInterrupt (Ctrl+C) received before node initialization, shutting down.')
    except Exception as e:
        # 捕获spin期间或初始化时可能发生的其他未处理异常
        if serial_node_instance:
            serial_node_instance.get_logger().fatal(
                f"Unhandled exception in main execution: {e}", exc_info=True
            )
        else:
            print(f"Unhandled exception during pre-node-spin execution: {e}")
            import traceback
            traceback.print_exc()
    finally:
        if serial_node_instance:
            serial_node_instance.get_logger().info("Main 'finally' block: Initiating node shutdown cleanup.")
            # 调用自定义的清理方法，该方法会处理定时器和串口
            serial_node_instance._on_shutdown()
            # ROS 2 会自动调用节点的 destroy_node()，我们的清理逻辑应放在 _on_shutdown
            # 或者直接重写 Node 类的 destroy_node 方法
            # serial_node_instance.destroy_node() # 通常由 rclpy.shutdown() 触发
        else:
            print("Main 'finally' block: Node instance was not created or already gone.")

        if rclpy.ok(): # 检查rclpy上下文是否仍然有效
            rclpy.shutdown()
            print("rclpy.shutdown() called.")
        else:
            print("rclpy context was already invalid before explicit shutdown call.")

if __name__ == '__main__':
    main()