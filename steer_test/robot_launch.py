# 명령어 : ros2 launch your_package_name robot_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 1. 메인 제어 노드(swerve_to_can_node3)를 실행하는 설정
    main_control_node = Node(
        package='my_motor_controller',
        executable='swerve_to_can_node3',
        name='swerve_to_can_node',
        output='screen'
    )

    # 2. CAN 자동 복구 스크립트를 실행하는 설정
    #    (ROS 노드가 아니므로 ExecuteProcess 사용)
    can_recovery_process = ExecuteProcess(
        cmd=['sudo', 'python3', '/path/to/your/can_auto_recovery.py'],
        name='can_auto_recovery',
        output='screen'
    )

    # 위 두 프로세스를 하나의 LaunchDescription으로 묶어서 반환
    return LaunchDescription([
        main_control_node,
        can_recovery_process,
    ])