from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    gps_node = Node(
        package="gps_rtk2",
        executable="gps_read",
    )

    imu_node = Node(
        package="imu",
        executable="imu_read"
    )

    ld.add_action(gps_node)
    ld.add_action(imu_node)

    return ld
