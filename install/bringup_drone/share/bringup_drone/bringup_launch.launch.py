from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # # PX4 node: my_frame

        # Node(
        #     package='control',
        #     executable='tf_node',
        #     name='tf',
        #     parameters=[{'use_sim_time': True}],
        #     output='screen'
        # ),


        Node(
            package='control',
            executable='tf_node',
            name='tf',
            output='screen'
        ),
        Node(
            package='control',
            executable='static_tf',
            name='tf_static',
            output='screen'
        ),

        # # Static transform from map -> world
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_map_world',
        #     arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'map', 'world'],
        #     output='screen'
        # ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_map_world',
        #     arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'camera_link', 'yinbot_0/realsense/base_link/realsense_d435'],
        #     output='screen'
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_base_to_camera',
        #     arguments=[
        #         '0.066', '0.0', '-0.053',
        #         '0', '1.570796', '0',
        #         'base_link', 'camera_link'
        #     ],
        # ),

        # Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     name='clock',
        #     arguments=[
        #         '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
        #     ],
        #     output='screen'
        # ),

        # Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     name='image_bridge',
        #     arguments=[
        #         '/depth_camera/image@sensor_msgs/msg/Image@gz.msgs.Image'
        #     ],
        #     output='screen'
        # ),

        # # depth bridge
        # Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     name='depth_bridge',
        #     arguments=[
        #         '/depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image'
        #     ],
        #     output='screen'
        # ),
        
        # # CameraInfo bridge
        # Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     name='depth_camera_bridge',
        #     arguments=[
        #         '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        #     ],
        #     output='screen'
        # ),

        # Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     name='depth_camera_bridge',
        #     arguments=[
        #         '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        #     ],
        #     parameters=[{
        #         'frame_name': 'camera_depth_frame'
        #     }],
        #     output='screen'
        # ),


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
