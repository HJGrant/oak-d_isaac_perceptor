import yaml
from typing import Any, List, Tuple
import os

from isaac_ros_launch_utils.all_types import (
    Action,
    ComposableNode,
    ComposableNodeContainer,
    LaunchDescription,
    LoadComposableNodes
)
import isaac_ros_launch_utils as lu
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_ros.actions import Node
import launch_ros.descriptions

NVBLOX_CONFIG = 'nvblox_config'
CUVSLAM_CONFIG = 'cuvslam_config'
COMMON_CONFIG = 'common_config'

def convert_bgr_to_rgb():
    return LaunchDescription([
        Node(
            package='my_oakd_launch',           # Name of your package
            executable='image_converter',       # The name of your node
            name='image_converter_node',        # The name of the node
            output='screen',                    # Output to the screen
            remappings=[                       # Optional: remap topics if necessary
                ('/oak/rgb/image_raw', '/oak/rgb/image_raw'),
                ('/oak/rgb/image_raw_rgb8', '/oak/rgb/image_raw_rgb8')
            ]
        )
    ]) 

def launch_setup_oakd_rgbd(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file")
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    name = LaunchConfiguration("name").perform(context)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, "launch", "camera.launch.py")
            ),
            launch_arguments={
                "name": name,
                "params_file": params_file,
                "parent_frame": LaunchConfiguration("parent_frame"),
                "cam_pos_x": LaunchConfiguration("cam_pos_x"),
                "cam_pos_y": LaunchConfiguration("cam_pos_y"),
                "cam_pos_z": LaunchConfiguration("cam_pos_z"),
                "cam_roll": LaunchConfiguration("cam_roll"),
                "cam_pitch": LaunchConfiguration("cam_pitch"),
                "cam_yaw": LaunchConfiguration("cam_yaw"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "pointcloud.enable": "false",
                "rs_compat": LaunchConfiguration("rs_compat"),
            }.items(),
        ),
    ]

def generate_launch_description_oakd_rgbd():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("camera_model", default_value="OAK-D"),
        DeclareLaunchArgument("parent_frame", default_value="oak-d-base-frame"),
        DeclareLaunchArgument("cam_pos_x", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_z", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(depthai_prefix, "config", "rgbd.yaml"),
        ),
        DeclareLaunchArgument("use_rviz", default_value="False"),
        DeclareLaunchArgument("rectify_rgb", default_value="False"),
        DeclareLaunchArgument("rs_compat", default_value="False"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup_oakd_rgbd)]
    )

def get_nvblox_remappings(cameras: dict) -> List[Tuple[str, str]]:
    remappings = []

    for i in range(len(cameras)):
        depth = cameras[i].get('depth')
        color = cameras[i].get('color')
        remappings.append((f'camera_{i}/depth/image', depth.get('image')))
        remappings.append((f'camera_{i}/depth/camera_info', depth.get('info')))
        remappings.append((f'camera_{i}/color/image', color.get('image')))
        remappings.append((f'camera_{i}/color/camera_info', color.get('info')))
    return remappings

def get_nvblox_params(num_cameras: int, nvblox_config: dict, common_config: dict) -> List[Any]:

    parameters = []
    for config_file in nvblox_config.get('config_files', []):
        assert 'path' in config_file, 'No `path` provided in config_files'
        parameters.append(lu.get_path(config_file.get('package', 'isaac_ros_perceptor_bringup'),
                                      config_file.get('path')))
    parameters.extend(nvblox_config.get('parameters', []))
    parameters.append({'num_cameras': num_cameras})

    if 'robot_frame' in common_config:
        parameters.append({'map_clearing_frame_id': common_config.get('robot_frame')})
        parameters.append({'esdf_slice_bounds_visualization_attachment_frame_id':
                           common_config.get('robot_frame')})
        parameters.append({'workspace_height_bounds_visualization_attachment_frame_id':
                           common_config.get('robot_frame')})
    if 'odom_frame' in common_config:
        parameters.append({'global_frame': common_config.get('odom_frame')})

    parameters.append(
        lu.get_path('nvblox_examples_bringup',
                    'config/nvblox/specializations/nvblox_dynamics.yaml'))

    return parameters

def start_nvblox(args: lu.ArgumentContainer, nvblox_config: dict,
                 common_config: dict) -> List[Action]:

    cameras = nvblox_config.get('remappings', [])
    num_cameras = len(cameras)
    assert num_cameras != 0, 'You need to provide at least one input channel'
    assert num_cameras <= 4, 'No more than 4 input channels must be provided'

    parameters = get_nvblox_params(num_cameras, nvblox_config, common_config)

    remappings = get_nvblox_remappings(cameras)

    nvblox_node = ComposableNode(
        name=nvblox_config.get('node_name', 'nvblox_node'),
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=remappings,
        parameters=parameters,
    )

    actions = []

    container_name = nvblox_config.get('container_name', 'nvblox_container')

    if nvblox_config.get('attach_to_container', False):
        actions.append(
            LoadComposableNodes(
                target_container=container_name,
                composable_node_descriptions=[
                    nvblox_node
                ]
            ))
    else:
        actions.append(
            ComposableNodeContainer(
                name=container_name,
                package='rclcpp_components',
                namespace='',
                executable='component_container_mt',
                arguments=['--ros-args', '--log-level', args.log_level],
                composable_node_descriptions=[
                    nvblox_node,
                ],
            ))
        
    return actions

def start_cuvslam(args: lu.ArgumentContainer, cuvslam_config: dict, common_config: dict) -> Action:
    parameters = []
    remappings = []
    stereo_images = cuvslam_config.get('remappings', {}).get('stereo_images', [])
    assert len(stereo_images) != 0, 'You need to provide at least one input image pair'

    optical_frames = []
    for i in range(len(stereo_images)):
        idx = 2 * i
        left = stereo_images[i].get('left')
        right = stereo_images[i].get('right')
        assert 'image' in left, '`image` is missing in the left field'
        assert 'info' in left, '`info` is missing in the left field'
        assert 'optical_frame' in left, '`optical_frame` is missing in the left field'
        assert 'image' in right, '`image` is missing in the right field'
        assert 'info' in right, '`info` is missing in the right field'
        assert 'optical_frame' in right, '`optical_frame` is missing in the right field'
        remappings.append((f'visual_slam/image_{idx}', left.get('image')))
        remappings.append((f'visual_slam/camera_info_{idx}', left.get('info')))
        remappings.append((f'visual_slam/image_{idx + 1}', right.get('image')))
        remappings.append((f'visual_slam/camera_info_{idx + 1}', right.get('info')))
        optical_frames.append(left.get('optical_frame'))
        optical_frames.append(right.get('optical_frame'))

    # Add the imu topic if provided
    imu_topic = cuvslam_config.get('remappings', {}).get('imu')
    if imu_topic is not None:
        remappings.append(('visual_slam/imu', imu_topic))
        parameters.append({'enable_imu_fusion': True})

    parameters.append({'num_cameras': 2 * len(stereo_images)})
    parameters.append({'min_num_images': 2 * len(stereo_images)})
    parameters.append({'camera_optical_frames': optical_frames})
    for config_file in cuvslam_config.get('config_files', []):
        assert 'path' in config_file, 'No `path` provided in config_files'
        parameters.append(lu.get_path(config_file.get('package', 'isaac_ros_perceptor_bringup'),
                                      config_file.get('path')))
    parameters.extend(cuvslam_config.get('parameters', []))
    if 'odom_frame' in common_config:
        parameters.append({'odom_frame': common_config.get('odom_frame')})
    if 'map_frame' in common_config:
        parameters.append({'map_frame': common_config.get('map_frame')})
    if 'robot_frame' in common_config:
        parameters.append({'base_frame': common_config.get('robot_frame')})

    cuvslam_node = ComposableNode(
        name=cuvslam_config.get('node_name', 'cuvslam_node'),
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        remappings=remappings,
        parameters=parameters,
    )

    if cuvslam_config.get('attach_to_container', False):
        cuvslam_container = LoadComposableNodes(
            target_container=cuvslam_config.get('container_name', 'cuvslam_container'),
            composable_node_descriptions=[
                cuvslam_node
            ]
        )
    else:
        cuvslam_container = ComposableNodeContainer(
            name=cuvslam_config.get('container_name', 'cuvslam_container'),
            package='rclcpp_components',
            namespace='',
            executable='component_container_mt',
            arguments=['--ros-args', '--log-level', args.log_level],
            composable_node_descriptions=[
                cuvslam_node,
            ],
        )
    return cuvslam_container

def start_recording(args: lu.ArgumentContainer, config: dict) -> List[Action]:
    # Bag recording
    topics = config.get('extra_topics', [])

    if NVBLOX_CONFIG in config:
        cameras = config.get(NVBLOX_CONFIG).get('remappings', [])
        for i in range(len(cameras)):
            depth = cameras[i].get('depth')
            color = cameras[i].get('color')
            topics.append(depth.get('image'))
            topics.append(depth.get('info'))
            topics.append(color.get('image'))
            topics.append(color.get('info'))

    if CUVSLAM_CONFIG in config:
        remappings = config.get(CUVSLAM_CONFIG).get('remappings', {})
        if 'imu' in remappings:
            topics.append(remappings.get('imu'))
        cameras = remappings.get('stereo_images', [])
        for i in range(len(cameras)):
            left = cameras[i].get('left')
            right = cameras[i].get('right')
            topics.append(left.get('image'))
            topics.append(left.get('info'))
            topics.append(right.get('image'))
            topics.append(right.get('info'))

    topics = list(set(topics))
    record_action = lu.record_rosbag(topics=" ".join(topics), bag_path=args.rosbag_output)
    recording_started_msg =\
        '''\n\n\n
        -----------------------------------------------------
                    BAG RECORDING IS STARTING NOW

                 (make sure the rgbd cameras are up)
        -----------------------------------------------------

        List of topics:\n - ''' + "\n - ".join(topics)

    return [record_action, lu.log_info(recording_started_msg)]

def generate_launch_description_impl(args: lu.ArgumentContainer) -> List[Action]:
    config_file = lu.get_path('isaac_ros_perceptor_bringup', args.config_file)
    with open(config_file, "r", encoding="utf-8") as file:
        config = yaml.safe_load(file)
    actions = []
    if len(args.rosbag_output) > 0:
        actions.extend(start_recording(args, config))
    else:
        common_config = config.get(COMMON_CONFIG, {})
        if NVBLOX_CONFIG in config and lu.is_false(args.disable_nvblox):
            nvblox_config = config.get(NVBLOX_CONFIG)
            actions.extend(start_nvblox(args, nvblox_config, common_config))

        if CUVSLAM_CONFIG in config and lu.is_false(args.disable_cuvslam):
            actions.append(start_cuvslam(args, config.get(CUVSLAM_CONFIG), common_config))
    if 'urdf_transforms' in config:
        actions.append(
            lu.add_robot_description(robot_calibration_path=config.get('urdf_transforms')))

    actions.append(
        lu.include(
            'isaac_ros_perceptor_bringup',
            'launch/tools/visualization.launch.py',
            launch_arguments={'use_foxglove_whitelist': args.use_foxglove_whitelist},
        ))

    print(f"########Actions List: {actions}#############")
    
    actions.append(generate_launch_description_oakd_rgbd())

    actions.append(convert_bgr_to_rgb())

    return actions

def generate_launch_description() -> LaunchDescription:
    package_name = 'isaac_ros_nvblox'  # Change this to match your package name

    # Get the path to the config file inside the package's "config" directory
    default_config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'my_rgbd_perceptor.yaml'
    )

    # Define arguments
    args = lu.ArgumentContainer()
    args.add_arg('rosbag_output', '', description='Path to the output', cli=True)
    args.add_arg('disable_nvblox', False, description='Disable nvblox', cli=True)
    args.add_arg('disable_cuvslam', False, description='Disable cuvslam', cli=True)
    args.add_arg('log_level', 'info', choices=['debug', 'info', 'warn'], cli=True)
    
    # Use the dynamically found config path instead of hardcoding it
    args.add_arg('config_file', default_config_path,
                 description='Path to the config file', cli=True)

    args.add_arg('use_foxglove_whitelist', False, cli=True)

    args.add_opaque_function(generate_launch_description_impl)

    return LaunchDescription(args.get_launch_actions())
