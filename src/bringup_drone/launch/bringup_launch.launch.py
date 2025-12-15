from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # # PX4 node: my_frame

        Node(
            package='control',
            executable='tf_node',
            name='tf_node',
            output='screen'
        ),

        # Static transform from map -> world
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_world',
            arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'map', 'world'],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='image_bridge',
            arguments=[
                '/world/default/model/x500_depth_0/link/realsense/base_link/sensor/realsense_d435/image@sensor_msgs/msg/Image@gz.msgs.Image'
            ],
            output='screen'
        ),

        # depth bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='depth_bridge',
            arguments=[
                '/world/default/model/x500_depth_0/link/realsense/base_link/sensor/realsense_d435/depth_image@sensor_msgs/msg/Image@gz.msgs.Image'
            ],
            output='screen'
        ),
        
        # CameraInfo bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='depth_camera_bridge',
            arguments=[
                '/world/default/model/x500_depth_0/link/realsense/base_link/sensor/realsense_d435/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
            ],
            output='screen'
        ),       


        Node(
            package='control',
            executable='px4_visualizer',
            name='visualizer',
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ]
    )
