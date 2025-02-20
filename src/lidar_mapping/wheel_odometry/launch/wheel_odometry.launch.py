import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取当前 launch 文件所在目录，然后构造参数文件的路径
    param_file: str = os.path.join(
        get_package_share_directory("wheel_odometry"),
        "param/wheel_odometry.yaml"
    )

    odometry_node = Node(
        package='wheel_odometry',
        executable='wheel_odometry',
        name='wheel_odometry',
        output='screen',
        parameters=[param_file]
    )

    return LaunchDescription([odometry_node])
