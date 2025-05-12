import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

sys.path.append(os.path.join(get_package_share_directory('rm_static_tf'), 'launch'))

def generate_launch_description():
    tf_nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.075", "0.0", "-0.071", "0.0", "-0.13062563953108347", "0.0", "0.9914317638128686", "armor_support_frame_0", "armor_0"],
            namespace="red_standard_robot1",
            name="armor_support_frame_0_to_armor_0",
            remappings=[("/tf_static", "/red_standard_robot1/tf_static")]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.075", "0.0", "-0.071", "0.0", "-0.13062563953108347", "0.0", "0.9914317638128686", "armor_support_frame_1", "armor_1"],
            namespace="red_standard_robot1",
            name="armor_support_frame_1_to_armor_1",
            remappings=[("/tf_static", "/red_standard_robot1/tf_static")]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.075", "0.0", "-0.071", "0.0", "-0.13062563953108347", "0.0", "0.9914317638128686", "armor_support_frame_2", "armor_2"],
            namespace="red_standard_robot1",
            name="armor_support_frame_2_to_armor_2",
            remappings=[("/tf_static", "/red_standard_robot1/tf_static")]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.075", "0.0", "-0.071", "0.0", "-0.13062563953108347", "0.0", "0.9914317638128686", "armor_support_frame_3", "armor_3"],
            namespace="red_standard_robot1",
            name="armor_support_frame_3_to_armor_3",
            remappings=[("/tf_static", "/red_standard_robot1/tf_static")]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.197", "0.0", "0.061", "0.0", "0.0", "0.0", "1.0", "chassis", "armor_support_frame_0"],
            namespace="red_standard_robot1",
            name="chassis_to_armor_support_frame_0",
            remappings=[("/tf_static", "/red_standard_robot1/tf_static")]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.0", "0.116", "0.061", "0.0", "0.0", "0.7071080798594737", "0.7071054825112364", "chassis", "armor_support_frame_1"],
            namespace="red_standard_robot1",
            name="chassis_to_armor_support_frame_1",
            remappings=[("/tf_static", "/red_standard_robot1/tf_static")]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["-0.197", "0.0", "0.061", "0.0", "0.0", "0.9999996829318346", "0.0007963267107332633", "chassis", "armor_support_frame_2"],
            namespace="red_standard_robot1",
            name="chassis_to_armor_support_frame_2",
            remappings=[("/tf_static", "/red_standard_robot1/tf_static")]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.0", "-0.116", "0.061", "0.0", "0.0", "-0.7071080798594737", "0.7071054825112364", "chassis", "armor_support_frame_3"],
            namespace="red_standard_robot1",
            name="chassis_to_armor_support_frame_3",
            remappings=[("/tf_static", "/red_standard_robot1/tf_static")]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.0", "0.0", "0.076", "0.0", "0.0", "0.0", "1.0", "base_footprint", "chassis"],
            namespace="red_standard_robot1",
            name="base_footprint_to_chassis",
            remappings=[("/tf_static", "/red_standard_robot1/tf_static")]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.1", "0.0", "0.045", "0.0", "0.0", "0.0", "1.0", "gimbal_pitch", "front_industrial_camera"],
            namespace="red_standard_robot1",
            name="gimbal_pitch_to_front_industrial_camera",
            remappings=[("/tf_static", "/red_standard_robot1/tf_static")]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.0", "0.0", "0.0", "0.5", "-0.4999999999999999", "0.5", "-0.5000000000000001", "front_industrial_camera", "front_industrial_camera_optical_frame"],
            namespace="red_standard_robot1",
            name="front_industrial_camera_to_optical_frame",
            remappings=[("/tf_static", "/red_standard_robot1/tf_static")]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.16", "0.0", "0.18", "1.0", "0.0", "0.0", "0.0", "chassis", "front_mid360"],
            namespace="red_standard_robot1",
            name="chassis_to_front_mid360",
            remappings=[("/tf_static", "/red_standard_robot1/tf_static")]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.155", "0.0", "0.1", "1.0", "0.0", "0.0", "0.0", "chassis", "front_rplidar_a2"],
            namespace="red_standard_robot1",
            name="chassis_to_front_rplidar_a2",
            remappings=[("/tf_static", "/red_standard_robot1/tf_static")]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["-0.207", "0.0", "0.1", "0.0", "0.0", "0.0", "1.0", "chassis", "light_indicator"],
            namespace="red_standard_robot1",
            name="chassis_to_light_indicator",
            remappings=[("/tf_static", "/red_standard_robot1/tf_static")]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.07", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0", "gimbal_pitch", "speed_monitor"],
            namespace="red_standard_robot1",
            name="gimbal_pitch_to_speed_monitor",
            remappings=[("/tf_static", "/red_standard_robot1/tf_static")]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.07", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0", "chassis", "gimbal_yaw"],
            namespace="red_standard_robot1",
            name="chassis_to_gimbal_yaw",
            remappings=[("/tf_static", "/red_standard_robot1/tf_static")]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.07", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0", "gimbal_yaw", "gimbal_yaw_fake"],
            namespace="red_standard_robot1",
            name="gimbal_yaw_to_gimbal_yaw_fake",
            remappings=[("/tf_static", "/red_standard_robot1/tf_static")]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.0", "0.0", "0.0", "0.01", "0.0", "0.0", "0.0", "gimbal_yaw", "gimbal_pitch"],
            namespace="red_standard_robot1",
            name="gimbal_yaw_to_gimbal_pitch",
            remappings=[("/tf_static", "/red_standard_robot1/tf_static")]
        ),
    ]

    return LaunchDescription(tf_nodes)
