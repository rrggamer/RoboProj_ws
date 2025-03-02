import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    package_name='robot_model' 

    slam_params = os.path.join(
        get_package_share_directory(package_name), 
        'config', 
        'mapper_params_online_async.yaml'  # Ensure this file exists in the specified path
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': os.path.join(get_package_share_directory(package_name), 'world', 'map_b_red_block.world')}.items()
             )#map_b_static.world

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot',
                                   '-x', '0', '-y', '0', '-z', '0.8',  
                                   '-R', '0', '-P', '0', '-Y', '1.57', 
                                   ],
                        output='screen')
    
    slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py'
                )]), 
                launch_arguments={'use_sim_time': 'true', 'params_file': slam_params}.items()
    )


    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory(package_name), 
                'config', 
                'twist_mux.yaml'  # Ensure this file exists in the specified path
            )
        ],
        remappings=[
            ('cmd_vel_out', 'diff_cont/cmd_vel_unstamped')  # Remap output topic
        ]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "gripper_controller",
        ],
    )
    

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        # slam,
        twist_mux,
        controller_manager,
    ])