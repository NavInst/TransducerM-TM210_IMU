import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('tm_imu')

    imu_1_params = os.path.join(
        pkg_share, 'config', 'tm_imu_1.yaml'
    )

    imu_2_params = os.path.join(
        pkg_share, 'config', 'tm_imu_2.yaml'
    )

    imu_3_params = os.path.join(
        pkg_share, 'config', 'tm_imu_3.yaml'
    )
    imu_4_params = os.path.join(
    	pkg_share, 'config', 'tm_imu_4.yaml'
)

    imu_1 = Node(
        package='tm_imu',
        executable='transducer_m_imu',
        name='tm_imu',
        namespace='tm_imu_1',
        parameters=[imu_1_params],
        output='screen'
    )

    imu_2 = Node(
        package='tm_imu',
        executable='transducer_m_imu',
        name='tm_imu',
        namespace='tm_imu_2',
        parameters=[imu_2_params],
        output='screen'
    )

    imu_3 = Node(
        package='tm_imu',
        executable='transducer_m_imu',
        name='tm_imu',
        namespace='tm_imu_3',
        parameters=[imu_3_params],
        output='screen'
    )
    
    imu_4 = Node(
    package='tm_imu',
    executable='transducer_m_imu',
    name='tm_imu',
    namespace='tm_imu_4',
    parameters=[imu_4_params],
    output='screen'
)

    return LaunchDescription([
        imu_1,
        imu_2,
        imu_3,
        imu_4
    ])
