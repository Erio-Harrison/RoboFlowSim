import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 获取当前启动文件的目录
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 构建 RViz2 配置文件的完整路径
    rviz_config_path = os.path.join(current_dir, '..', 'rviz2', 'gas_flow_config.rviz')

    return LaunchDescription([
        
        # GPS 发布器节点
        Node(
            package='gps_location',
            executable='gps_publisher',
            name='gps_publisher'
        ),

        # 轨迹规划节点
        Node(
            package='trajectory_plan',
            executable='trajectory_planner',
            name='trajectory_planner'
        ),

        # 气体流可视化节点
        Node(
            package='visualization',
            executable='gas_flow_visualizer',
            name='gas_flow_visualizer'
        ),

        # GPS 可视化节点
        Node(
            package='visualization',
            executable='gps_visualizer',
            name='gps_visualizer'
        ),
        
        Node(
            package='visualization',
            executable='realistic_gas_diffusion',
            name='realistic_gas_diffusion'
        ),

        Node(
            package='visualization',
            executable='gas_diffusion_2d',
            name='gas_diffusion_2d'
        ),

        Node(
            package='gas_sonsor',
            executable='gas_dispersion_node',
            name='gas_dispersion_node'
        ),
        Node(
            package='gas_sonsor',
            executable='gas_sensor_node',
            name='gas_sensor_node'
        ),
        Node(
            package='gas_sonsor',
            executable='visualization_node',
            name='visualization_node'
        ),

        # 启动 RViz2 并加载配置文件
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        )
    ])