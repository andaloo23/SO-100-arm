from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare the DOF argument
    dof_arg = DeclareLaunchArgument(
        'dof',
        default_value='5',
        description='DOF configuration - either 5 or 7'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    def launch_setup(context, *args, **kwargs):
        dof = LaunchConfiguration('dof').perform(context)
        use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
        use_sim_time = use_sim_time_str.lower() in ['true', '1', 'yes']

        pkg_share = FindPackageShare('so_100_arm').find('so_100_arm')

        urdf_xacro_file = 'so_100_arm.urdf.xacro'
        urdf_path = os.path.join(pkg_share, 'config', urdf_xacro_file)

        controllers_yaml_path = os.path.join(pkg_share, 'config', f'controllers_{dof}dof.yaml')

        rviz_config_path = os.path.join(pkg_share, 'config', 'urdf.rviz')

        robot_description_content = Command(
            f'xacro {urdf_path} use_fake_hardware:=true'
        )
        robot_description = {'robot_description': robot_description_content}

        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}]
        )

        controller_manager_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                robot_description,
                controllers_yaml_path,
                {'use_sim_time': use_sim_time},
                {'log_level': 'debug'}
            ],
            output='screen',
            remappings=[
                ('/controller_manager/robot_description', '/robot_description')
            ]
        )

        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--ros-args', '-p', f'use_sim_time:={use_sim_time_str}'],
            output='screen'
        )

        joint_trajectory_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '--ros-args', '-p', f'use_sim_time:={use_sim_time_str}'],
            output='screen'
        )

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rvzi2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': use_sim_time}]
        )
        
        nodes = [
            robot_state_publisher_node,
            controller_manager_node,
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner,
            rviz_node
        ]
        return nodes

    return LaunchDescription([
        dof_arg,
        use_sim_time_arg,
        OpaqueFunction(function=launch_setup)
    ])