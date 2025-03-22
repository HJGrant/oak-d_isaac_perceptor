import yaml
from typing import Any, List, Tuple

from isaac_ros_launch_utils.all_types import (
    Action,
    ComposableNode,
    ComposableNodeContainer,
    LaunchDescription,
    LoadComposableNodes
)
import isaac_ros_launch_utils as lu

NVBLOX_CONFIG = 'nvblox_config'
CUVSLAM_CONFIG = 'cuvslam_config'
COMMON_CONFIG = 'common_config'

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
import launch_ros.actions
import launch_ros.descriptions


def oak_d_launch_description():
    depthai_examples_path = get_package_share_directory('depthai_examples')
    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_descriptions'), 'launch')

    aligned_rviz = os.path.join(depthai_examples_path,
                                'rviz', 'stereoInertialDepthAlignROS2.rviz')
    
    rectify_rviz = os.path.join(depthai_examples_path,
                                'rviz', 'stereoInertial.rviz')
    default_resources_path = os.path.join(depthai_examples_path,
                                'resources')

    mxId         = LaunchConfiguration('mxId',      default = 'x')
    usb2Mode     = LaunchConfiguration('usb2Mode',  default = False)
    poeMode      = LaunchConfiguration('poeMode',   default = False)

    camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D')
    tf_prefix    = LaunchConfiguration('tf_prefix',     default = 'oak')
    mode         = LaunchConfiguration('mode', default = 'depth')
    base_frame   = LaunchConfiguration('base_frame',    default = 'oak-d_frame')
    parent_frame = LaunchConfiguration('parent_frame',  default = 'oak-d-base-frame')
    imuMode      = LaunchConfiguration('imuMode', default = '1')

    cam_pos_x    = LaunchConfiguration('cam_pos_x',     default = '0.0')
    cam_pos_y    = LaunchConfiguration('cam_pos_y',     default = '0.0')
    cam_pos_z    = LaunchConfiguration('cam_pos_z',     default = '0.0')
    cam_roll     = LaunchConfiguration('cam_roll',      default = '0.0')
    cam_pitch    = LaunchConfiguration('cam_pitch',     default = '0.0')
    cam_yaw      = LaunchConfiguration('cam_yaw',       default = '0.0')

    lrcheck        = LaunchConfiguration('lrcheck', default = True)
    extended       = LaunchConfiguration('extended', default = False)
    subpixel       = LaunchConfiguration('subpixel', default = True)
    rectify        = LaunchConfiguration('rectify', default = True)
    depth_aligned  = LaunchConfiguration('depth_aligned', default = False)
    manualExposure = LaunchConfiguration('manualExposure', default = False)
    expTime        = LaunchConfiguration('expTime', default = 20000)
    sensIso        = LaunchConfiguration('sensIso', default = 800)

    enableSpatialDetection  = LaunchConfiguration('enableSpatialDetection', default = True)
    syncNN                  = LaunchConfiguration('syncNN', default = True)
    detectionClassesCount   = LaunchConfiguration('detectionClassesCount', default = 80)
    nnName                  = LaunchConfiguration('nnName', default = 'x')
    resourceBaseFolder      = LaunchConfiguration('resourceBaseFolder', default = default_resources_path)


    stereo_fps            = LaunchConfiguration('stereo_fps', default = 30)
    confidence            = LaunchConfiguration('confidence', default = 200)
    LRchecktresh          = LaunchConfiguration('LRchecktresh', default = 5)
    monoResolution        = LaunchConfiguration('monoResolution', default = '400p')
    
    rgbResolution           = LaunchConfiguration('rgbResolution',  default = '1080p')
    rgbScaleNumerator       = LaunchConfiguration('rgbScaleNumerator',  default = 2)
    rgbScaleDinominator     = LaunchConfiguration('rgbScaleDinominator',    default = 3)
    previewWidth            = LaunchConfiguration('previewWidth',   default = 416)
    previewHeight           = LaunchConfiguration('previewHeight',  default = 416)
    
    
    angularVelCovariance  = LaunchConfiguration('angularVelCovariance', default = 0.0)
    linearAccelCovariance = LaunchConfiguration('linearAccelCovariance', default = 0.0)

    enableDotProjector = LaunchConfiguration('enableDotProjector', default = False)
    enableFloodLight   = LaunchConfiguration('enableFloodLight', default = False)
    dotProjectorIntensity     = LaunchConfiguration('dotProjectorIntensity', default = 0.5)
    floodLightIntensity       = LaunchConfiguration('floodLightIntensity', default = 0.5)
    enableRosBaseTimeUpdate       = LaunchConfiguration('enableRosBaseTimeUpdate', default = False)
    enableRviz         = LaunchConfiguration('enableRviz', default = True)


    declare_mxId_cmd = DeclareLaunchArgument(
        'mxId',
        default_value=mxId,
        description='select the device by passing the MxID of the device. It will connect to first available device if left empty.')

    declare_usb2Mode_cmd = DeclareLaunchArgument(
        'usb2Mode',
        default_value=usb2Mode,
        description='To revert and use usb2 Mode. Set this parameter to false')

    declare_poeMode_cmd = DeclareLaunchArgument(
        'poeMode',
        default_value=poeMode,
        description='When MxID is set and the device is a POE model then set the poeMode to \"true\" to connect properly.')

    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value=camera_model,
        description='The model of the camera. Using a wrong camera model can disable camera features. Valid models: `OAK-D, OAK-D-LITE`.')

    declare_tf_prefix_cmd = DeclareLaunchArgument(
        'tf_prefix',
        default_value=tf_prefix,
        description='your custom name for the prefix of camera TF frames')
 
    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value=mode,
        description='set to  \"depth\" or \"disparity\". Setting to depth will publish depth or else will publish disparity.')

    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value=base_frame,
        description='Name of the base link in the TF Tree.')

    declare_parent_frame_cmd = DeclareLaunchArgument(
        'parent_frame',
        default_value=parent_frame,
        description='Name of the parent link from an another robot TF that can be connected to the base of the OAK device.')

    declare_imu_mode_cmd = DeclareLaunchArgument(
        'imuMode',
        default_value=imuMode,
        description=' set to 0 -> COPY, 1 -> LINEAR_INTERPOLATE_GYRO, 2 -> LINEAR_INTERPOLATE_ACCEL')


    declare_pos_x_cmd = DeclareLaunchArgument(
        'cam_pos_x',
        default_value=cam_pos_x,
        description='Position X of the camera with respect to the base frame.')

    declare_pos_y_cmd = DeclareLaunchArgument(
        'cam_pos_y',
        default_value=cam_pos_y,
        description='Position Y of the camera with respect to the base frame.')

    declare_pos_z_cmd = DeclareLaunchArgument(
        'cam_pos_z',
        default_value=cam_pos_z,
        description='Position Z of the camera with respect to the base frame.')

    declare_roll_cmd = DeclareLaunchArgument(
        'cam_roll',
        default_value=cam_roll,
        description='Roll orientation of the camera with respect to the base frame.')

    declare_pitch_cmd = DeclareLaunchArgument(
        'cam_pitch',
        default_value=cam_pitch,
        description='Pitch orientation of the camera with respect to the base frame.')

    declare_yaw_cmd = DeclareLaunchArgument(
        'cam_yaw',
        default_value=cam_yaw,
        description='Yaw orientation of the camera with respect to the base frame.')
    

    declare_lrcheck_cmd = DeclareLaunchArgument(
        'lrcheck',
        default_value=lrcheck,
        description='LR-Check is used to remove incorrectly calculated disparity pixels due to occlusions at object borders. Set to true to enable it')

    declare_extended_cmd = DeclareLaunchArgument(
        'extended',
        default_value=extended,
        description='Extended disparity mode allows detecting closer distance objects for the given baseline. Set this parameter to true to enable it')

    declare_subpixel_cmd = DeclareLaunchArgument(
        'subpixel',
        default_value=subpixel,
        description='Subpixel mode improves the precision and is especially useful for long range measurements. It also helps for better estimating surface normals. Set this parameter to true to enable it')

    declare_rectify_cmd = DeclareLaunchArgument(
        'rectify',
        default_value=rectify,
        description='enable this to publish rectified images used for depth estimation')

    declare_depth_aligned_cmd = DeclareLaunchArgument(
        'depth_aligned',
        default_value=depth_aligned,
        description='When depth_aligned is enabled depth map from stereo will be aligned to the RGB camera in the center.')

    declare_manualExposure_cmd = DeclareLaunchArgument(
        'manualExposure',
        default_value=manualExposure,
        description='When manualExposure is enabled, you can set the exposure time(expTime) and ISO(sensIso) of the stereo camera.')
    
    declare_expTime_cmd = DeclareLaunchArgument(
        'expTime',
        default_value=expTime,
        description='Set the exposure time of the stereo camera. Default value is 20000')

    declare_sensIso_cmd = DeclareLaunchArgument(
        'sensIso',
        default_value=sensIso,
        description='Set the ISO of the stereo camera. Default value is 800')

    declare_enableSpatialDetection_cmd = DeclareLaunchArgument(
        'enableSpatialDetection',
        default_value=enableSpatialDetection,
        description='When enableSpatialDetection is enabled NN Object detection with Spatial Positioning will be run.')

    declare_syncNN_cmd = DeclareLaunchArgument(
        'syncNN',
        default_value=syncNN,
        description='When syncNN is enabled Preview Image will be synced with the Detections.')

    declare_detectionClassesCount_cmd = DeclareLaunchArgument(
        'detectionClassesCount',
        default_value=detectionClassesCount,
        description='When detectionClassesCount is number of classes the NN contains. Default is set to 80.')

    declare_nnName_cmd = DeclareLaunchArgument(
        'nnName',
        default_value=nnName,
        description='Name of the NN blob being used to load. By default the one in resources folder will be used.')

    declare_resourceBaseFolder_cmd = DeclareLaunchArgument(
        'resourceBaseFolder',
        default_value=resourceBaseFolder,
        description='Path to the folder where NN Blob is stored.')

    declare_stereo_fps_cmd = DeclareLaunchArgument(
        'stereo_fps',
        default_value=stereo_fps,
        description='Sets the FPS of the cameras used in the stereo setup.')

    declare_confidence_cmd = DeclareLaunchArgument(
        'confidence',
        default_value=confidence,
        description='Set the confidence of the depth from 0-255. Max value means allow depth of all confidence. Default is set to 200')

    declare_LRchecktresh_cmd = DeclareLaunchArgument(
        'LRchecktresh',
        default_value=LRchecktresh,
        description='Set the LR threshold from 1-10 to get more accurate depth. Default value is 5.')

    declare_monoResolution_cmd = DeclareLaunchArgument(
        'monoResolution',
        default_value=monoResolution,
        description='Set the resolution of the mono/Stereo setup. Choose between 720p, 400p, 480p, 800p.')
 
    declare_rgbResolution_cmd = DeclareLaunchArgument(
        'rgbResolution',
        default_value=rgbResolution,
        description='Set the resolution of the RGB setup. Choose between 1080p, 4k, 12MP.')

    declare_rgbScaleNumerator_cmd = DeclareLaunchArgument(
        'rgbScaleNumerator',
        default_value=rgbScaleNumerator,
        description='Number of the scale Factor Numberator on top of RGB resolution selection.')

    declare_rgbScaleDinominator_cmd = DeclareLaunchArgument(
        'rgbScaleDinominator',
        default_value=rgbScaleDinominator,
        description='Number of the scale Factor Dinominator on top of RGB resolution selection.')

    declare_previewWidth_cmd = DeclareLaunchArgument(
        'previewWidth',
        default_value=previewWidth,
        description='Set the width of the preview window used for the NN detection.')

    declare_previewHeight_cmd = DeclareLaunchArgument(
        'previewHeight',
        default_value=previewHeight,
        description='Set the height of the preview window used for the NN detection.')

    declare_angularVelCovariance_cmd = DeclareLaunchArgument(
        'angularVelCovariance',
        default_value=angularVelCovariance,
        description='Set the angular velocity covariance of the IMU.')

    declare_linearAccelCovariance_cmd = DeclareLaunchArgument(
        'linearAccelCovariance',
        default_value=linearAccelCovariance,
        description='Set the Linear acceleration covariance of the IMU.')

    declare_enableDotProjector_cmd = DeclareLaunchArgument(
        'enableDotProjector',
        default_value=enableDotProjector,
        description='set this to true to enable the dot projector structure light (Available only on Pro models).')
    
    declare_enableFloodLight_cmd = DeclareLaunchArgument(
        'enableFloodLight',
        default_value=enableFloodLight,
        description='Set this to true to enable the flood light for night vision (Available only on Pro models).')
   
    declare_dotProjectorIntensity_cmd = DeclareLaunchArgument(
        'dotProjectorIntensity',
        default_value=dotProjectorIntensity,
        description='Set the mA at which you intend to drive the dotProjector. Default is set to 0.5.')

    declare_floodLightIntensity_cmd = DeclareLaunchArgument(
        'floodLightIntensity',
        default_value=floodLightIntensity,
        description='Set the mA at which you intend to drive the FloodLight. Default is set to 0.5.')
    declare_enableRosBaseTimeUpdate_cmd = DeclareLaunchArgument(
        'enableRosBaseTimeUpdate',
        default_value=enableRosBaseTimeUpdate,
        description='Whether to update ROS time on each message.')


    declare_enableRviz_cmd = DeclareLaunchArgument(
        'enableRviz',
        default_value=enableRviz,
        description='When True create a RVIZ window.')
    


    urdf_launch = IncludeLaunchDescription(
                            launch_description_sources.PythonLaunchDescriptionSource(
                                    os.path.join(urdf_launch_dir, 'urdf_launch.py')),
                            launch_arguments={'tf_prefix'   : tf_prefix,
                                              'camera_model': camera_model,
                                              'base_frame'  : base_frame,
                                              'parent_frame': parent_frame,
                                              'cam_pos_x'   : cam_pos_x,
                                              'cam_pos_y'   : cam_pos_y,
                                              'cam_pos_z'   : cam_pos_z,
                                              'cam_roll'    : cam_roll,
                                              'cam_pitch'   : cam_pitch,
                                              'cam_yaw'     : cam_yaw}.items())


    stereo_node = launch_ros.actions.Node(
            package='depthai_examples', executable='stereo_inertial_node',
            output='screen',
            parameters=[{'mxId':                    mxId},
                        {'usb2Mode':                usb2Mode},
                        {'poeMode':                 poeMode},
                        {'resourceBaseFolder':      resourceBaseFolder},

                        {'tf_prefix':               tf_prefix},
                        {'mode':                    mode},
                        {'imuMode':                 imuMode},

                        {'lrcheck':                 lrcheck},
                        {'extended':                extended},
                        {'subpixel':                subpixel},
                        {'rectify':                 rectify},

                        {'depth_aligned':           depth_aligned},
                        {'manualExposure':          manualExposure},
                        {'expTime':                 expTime},
                        {'sensIso':                 sensIso},
                        {'stereo_fps':              stereo_fps},
                        {'confidence':              confidence},
                        {'LRchecktresh':            LRchecktresh},
                        {'monoResolution':          monoResolution},
                        {'rgbResolution':           rgbResolution},

                        {'rgbScaleNumerator':       rgbScaleNumerator},
                        {'rgbScaleDinominator':     rgbScaleDinominator},
                        {'previewWidth':            previewWidth},
                        {'previewHeight':           previewHeight},

                        {'angularVelCovariance':    angularVelCovariance},
                        {'linearAccelCovariance':   linearAccelCovariance},
                        {'enableSpatialDetection':  enableSpatialDetection},
                        {'detectionClassesCount':   detectionClassesCount},
                        {'syncNN':                  syncNN},
                        {'nnName':                  nnName},
                        
                        {'enableDotProjector':      enableDotProjector},
                        {'enableFloodLight':        enableFloodLight},
                        {'dotProjectorIntensity':          dotProjectorIntensity},
                        {'floodLightIntensity':            floodLightIntensity},
                        {'enableRosBaseTimeUpdate': enableRosBaseTimeUpdate}
                        ])
    
    depth_metric_converter = launch_ros.descriptions.ComposableNode(
                                package='depth_image_proc',
                                plugin='depth_image_proc::ConvertMetricNode',
                                name='convert_metric_node',
                                remappings=[('image_raw', 'stereo/depth'),
                                            ('camera_info', 'stereo/camera_info'),
                                            ('image', 'stereo/converted_depth')]
                                )
    pointcloud_topic = '/stereo/points'
    point_cloud_creator = None
    rviz_node = None
    if LaunchConfigurationEquals('depth_aligned', 'True'): 
        point_cloud_creator = launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='point_cloud_xyzrgb_node',
                    remappings=[('depth_registered/image_rect', 'stereo/converted_depth'),
                                ('rgb/image_rect_color', 'color/image'),
                                ('rgb/camera_info', 'color/camera_info'),
                                ('points', pointcloud_topic )]
                )

        rviz_node = launch_ros.actions.Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['--display-config', aligned_rviz],
            condition=IfCondition(enableRviz))

    else:
        point_cloud_creator = launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyziNode',
                    name='point_cloud_xyzi',

                    remappings=[('depth/image_rect', 'stereo/converted_depth'),
                                ('intensity/image_rect', 'right/image_rect'),
                                ('intensity/camera_info', 'right/camera_info'),
                                ('points', pointcloud_topic)]
                )


        rviz_node = launch_ros.actions.Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['--display-config', rectify_rviz],
            condition=IfCondition(enableRviz))



    if point_cloud_creator is not None:
        point_cloud_container = launch_ros.actions.ComposableNodeContainer(
                name='container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    # Driver itself
                    depth_metric_converter,
                    point_cloud_creator
                ],
                output='screen',)
    marker_node = launch_ros.actions.Node(
            package='depthai_examples', executable='rviz2', output='screen',
            arguments=['--display-config', rectify_rviz],
            condition=IfCondition(enableRviz))

    ld = LaunchDescription()

    ld.add_action(declare_mxId_cmd)
    ld.add_action(declare_usb2Mode_cmd)
    ld.add_action(declare_poeMode_cmd)
    ld.add_action(declare_camera_model_cmd)

    ld.add_action(declare_tf_prefix_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_base_frame_cmd)
    ld.add_action(declare_parent_frame_cmd)
    ld.add_action(declare_imu_mode_cmd)

    ld.add_action(declare_pos_x_cmd)
    ld.add_action(declare_pos_y_cmd)
    ld.add_action(declare_pos_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)
    
    ld.add_action(declare_lrcheck_cmd)
    ld.add_action(declare_extended_cmd)
    ld.add_action(declare_subpixel_cmd)
    ld.add_action(declare_rectify_cmd)
    ld.add_action(declare_depth_aligned_cmd)
    ld.add_action(declare_manualExposure_cmd)
    ld.add_action(declare_expTime_cmd)
    ld.add_action(declare_sensIso_cmd)

    ld.add_action(declare_enableSpatialDetection_cmd)
    ld.add_action(declare_syncNN_cmd)
    ld.add_action(declare_detectionClassesCount_cmd)
    ld.add_action(declare_nnName_cmd)
    ld.add_action(declare_resourceBaseFolder_cmd)

    ld.add_action(declare_stereo_fps_cmd)
    ld.add_action(declare_confidence_cmd)
    ld.add_action(declare_LRchecktresh_cmd)
    ld.add_action(declare_monoResolution_cmd)

    ld.add_action(declare_rgbResolution_cmd)
    ld.add_action(declare_rgbScaleNumerator_cmd)
    ld.add_action(declare_rgbScaleDinominator_cmd)
    ld.add_action(declare_previewWidth_cmd)
    ld.add_action(declare_previewHeight_cmd)

    ld.add_action(declare_angularVelCovariance_cmd)
    ld.add_action(declare_linearAccelCovariance_cmd)

    ld.add_action(declare_enableDotProjector_cmd)
    ld.add_action(declare_enableFloodLight_cmd)
    ld.add_action(declare_dotProjectorIntensity_cmd)
    ld.add_action(declare_floodLightIntensity_cmd)

    ld.add_action(declare_enableRviz_cmd)

    ld.add_action(urdf_launch)
    ld.add_action(stereo_node)

    if LaunchConfigurationEquals('depth_aligned', 'True') and LaunchConfigurationEquals('rectify', 'True'):
        ld.add_action(point_cloud_container)
    
    # ld.add_action(point_cloud_node)
    if LaunchConfigurationEquals('enableRviz', 'True') and rviz_node is not None:
        ld.add_action(rviz_node)
    return ld

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
    print(f"########IMU TOPIC############={imu_topic}")
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

def generate_launch_description_impl(args: lu.ArgumentContainer) -> List[Action]:true
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
    
    actions.append(oak_d_launch_description())

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
