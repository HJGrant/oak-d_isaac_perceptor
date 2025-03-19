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
            package='oakd_isaac_ros',           # Name of your package
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
    package_prefix = get_package_share_directory("oakd_isaac_ros")
    print("### OAK-D LAUNCH FILE: " + str(os.path.join(package_prefix, "launch", "tools", "camera.launch.py")) + " ###")

    name = LaunchConfiguration("name").perform(context)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_prefix, "launch", "tools", "camera.launch.py")
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
    package_prefix = get_package_share_directory("oakd_isaac_ros")
    print("### OAK-D PARAM FILE: " + str(os.path.join(package_prefix, "config", "oak_d", "oakd_rgbd.yaml")) + " ###")
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
            default_value=os.path.join(package_prefix, "config", "oak_d", "oakd_rgbd.yaml"),
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
        print("### NVBLOX MAIN CONFIG FILE: " + str(config_file) + " ###" )
        parameters.append(lu.get_path(config_file.get('package', 'oakd_isaac_ros'),
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
        lu.get_path('oakd_isaac_ros',
                    'config/nvblox/specializations/nvblox_dynamics.yaml'))

    print("### APPENDED NVBLOX PARAMS: " + str(lu.get_path('oakd_isaac_ros', 'config/nvblox/specializations/nvblox_dynamics.yaml')) + " ###")

    return parameters

def start_nvblox(args: lu.ArgumentContainer, nvblox_config: dict,
                 common_config: dict) -> List[Action]:

    cameras = nvblox_config.get('remappings', [])
    num_cameras = len(cameras)
    assert num_cameras != 0, 'You need to provide at least one input channel'
    assert num_cameras <= 4, 'No more than 4 input channels must be provided'

    parameters = get_nvblox_params(num_cameras, nvblox_config, common_config)

    ## set the voxel size dynamically on launch
    voxel_size = args.voxel_size
    parameters.append({'voxel_size': voxel_size})

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

def generate_static_transforms():
    return LaunchDescription([
        Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"]),

        Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "0", "0", "0", "0", "0", "odom", "oak-d-base-frame"])
    ])

def generate_launch_description_impl(args: lu.ArgumentContainer) -> List[Action]:
    config_file = lu.get_path('oakd_isaac_ros', args.config_file)
    print("### LAUNCH FILE CONFIG :" + str(config_file))
    with open(config_file, "r", encoding="utf-8") as file:
        config = yaml.safe_load(file)
    actions = []

    common_config = config.get(COMMON_CONFIG, {})
    if NVBLOX_CONFIG in config and lu.is_false(args.disable_nvblox):
        nvblox_config = config.get(NVBLOX_CONFIG)
        actions.extend(start_nvblox(args, nvblox_config, common_config))

    actions.append(
        lu.include(
            'oakd_isaac_ros',
            'launch/tools/visualization.launch.py',
            launch_arguments={'use_foxglove_whitelist': args.use_foxglove_whitelist},
        ))
    
    actions.append(generate_launch_description_oakd_rgbd())

    actions.append(convert_bgr_to_rgb())

    actions.append(generate_static_transforms())

    return actions

def generate_launch_description() -> LaunchDescription:
    package_name = 'oakd_isaac_ros'  # Change this to match your package name

    # Get the path to the config file inside the package's "config" directory
    default_config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'nvblox_dynamics.yaml'
    )

    # Define arguments
    args = lu.ArgumentContainer()
    args.add_arg('disable_nvblox', False, description='Disable nvblox', cli=True)
    args.add_arg('log_level', 'info', choices=['debug', 'info', 'warn'], cli=True)
    
    # Use the dynamically found config path instead of hardcoding it
    args.add_arg('config_file', default_config_path,
                 description='Path to the config file', cli=True)

    args.add_arg('voxel_size', 0.05, description='Set nvBlox Voxel Size', cli=True)

    args.add_arg('use_foxglove_whitelist', False, cli=True)

    args.add_opaque_function(generate_launch_description_impl)

    return LaunchDescription(args.get_launch_actions())
