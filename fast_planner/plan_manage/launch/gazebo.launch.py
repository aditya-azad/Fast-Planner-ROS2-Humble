"""
Integrated launch file for Fast-Planner + Bridge + DQNMPC control
This launches everything needed to test Fast-Planner with the DQ-NMPC controller
"""

from ament_index_python.packages import \
    get_package_share_directory  # type: ignore
from launch import LaunchDescription  # type: ignore
from launch.actions import (DeclareLaunchArgument,  # type: ignore
                            ExecuteProcess, GroupAction)
from launch.conditions import IfCondition, UnlessCondition  # type: ignore
from launch.substitutions import (LaunchConfiguration,  # type: ignore
                                  PathJoinSubstitution, TextSubstitution)
from launch_ros.actions import ComposableNodeContainer, Node  # type: ignore
from launch_ros.descriptions import ComposableNode  # type: ignore


def generate_launch_description():
    # Declare arguments - using new conventions
    launch_args = [
        DeclareLaunchArgument("quad_name", default_value="quadrotor"),
        DeclareLaunchArgument("platform_type", default_value="eagle"),
        DeclareLaunchArgument("world_name", default_value="lawn"),
        DeclareLaunchArgument("mass_sitl", default_value="2.06"),
        DeclareLaunchArgument("t_hover_sitl", default_value="0.728"),
        DeclareLaunchArgument("world_frame_id", default_value="world"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        # Fast-Planner arguments
        DeclareLaunchArgument("map_size_x", default_value="40.0"),
        DeclareLaunchArgument("map_size_y", default_value="20.0"),
        DeclareLaunchArgument("map_size_z", default_value="5.0"),
        DeclareLaunchArgument("max_vel", default_value="0.8"),
        DeclareLaunchArgument("max_acc", default_value="2.0"),
        # Waypoint arguments
        DeclareLaunchArgument("point0_x", default_value="5.0"),
        DeclareLaunchArgument("point0_y", default_value="0.0"),
        DeclareLaunchArgument("point0_z", default_value="1.5"),
        DeclareLaunchArgument("point1_x", default_value="5.0"),
        DeclareLaunchArgument("point1_y", default_value="5.0"),
        DeclareLaunchArgument("point1_z", default_value="1.5"),
        DeclareLaunchArgument("point2_x", default_value="0.0"),
        DeclareLaunchArgument("point2_y", default_value="0.0"),
        DeclareLaunchArgument("point2_z", default_value="1.5"),
        # Nvblox integration (optional)
        DeclareLaunchArgument(
            "use_nvblox",
            default_value="false",
            description="Enable nvblox GPU-accelerated mapping",
        ),
        DeclareLaunchArgument("use_mapping", default_value="false"),
    ]

    # Use the LaunchConfiguration for each argument
    quad_name = LaunchConfiguration("quad_name")
    platform_type = LaunchConfiguration("platform_type")
    world_name = LaunchConfiguration("world_name")
    mass_sitl = LaunchConfiguration("mass_sitl")
    t_hover_sitl = LaunchConfiguration("t_hover_sitl")
    world_frame_id = LaunchConfiguration("world_frame_id")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_nvblox_arg = LaunchConfiguration("use_nvblox")
    use_mapping = LaunchConfiguration("use_mapping")
    # Conditions
    nvblox_condition = IfCondition(use_nvblox_arg)
    no_nvblox_condition = UnlessCondition(use_nvblox_arg)

    # Config paths - using acp_autonomy package
    control_config = PathJoinSubstitution(
        [
            get_package_share_directory("acp_autonomy"),
            TextSubstitution(text="config"),
            platform_type,
            TextSubstitution(text="default"),
            TextSubstitution(text="dq_control.yaml"),
        ]
    )

    nvblox_config_base = PathJoinSubstitution(
        [
            get_package_share_directory("acp_autonomy"),
            TextSubstitution(text="config"),
            platform_type,
            TextSubstitution(text="default"),
            TextSubstitution(text="perception"),
            TextSubstitution(text="nvblox_base.yaml"),
        ]
    )

    # ========== PX4 / Gazebo Nodes ==========
    # NOTE: If using fastplanner_test.sh, PX4 and MicroXRCEAgent are started
    # separately. Comment these out to avoid conflicts, or run this launch
    # file standalone.

    # px4_gazebo_command = ExecuteProcess(
    #     cmd=[
    #         "bash",
    #         "-c",
    #         "$PX4_PATH/build/px4_sitl_default/bin/px4",
    #     ],
    #     additional_env={
    #         "PX4_UXRCE_DDS_NS": quad_name,
    #         "PX4_SITL_WORLD": world_name,
    #         "PX4_GZ_WORLD": world_name,
    #     },
    #     output="screen",
    # )

    # microxrce = ExecuteProcess(
    #     cmd=["bash", "-c", "MicroXRCEAgent udp4 -p 8888"],
    # )

    gz_depth_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=quad_name,
        arguments=[
            "/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image",
            "/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    # ========== PX4 Interface Nodes ==========

    px4_odom_transform = Node(
        namespace=quad_name,
        name="px4_odom_transform_node",
        package="px4_odom_transform",
        parameters=[{"use_sim_time": use_sim_time}],
        executable="px4_odom_transform_node",
        remappings=[
            ("odom", "odom"),
            # Remap to global PX4 topics (not namespaced)
            ("fmu/out/vehicle_odometry", "/fmu/out/vehicle_odometry"),
            ("fmu/out/sensor_combined", "/fmu/out/sensor_combined"),
            ("fmu/out/vehicle_attitude", "/fmu/out/vehicle_attitude"),
        ],
    )

    px4_offboard_node = Node(
        name="px4_offboard_node",
        namespace=quad_name,
        package="px4_interface",
        executable="px4_offboard_node",
        parameters=[
            control_config,
            {
                "use_sim_time": use_sim_time,
                "control_type": True,  # NMPC enabled
                "topic_is_1": False,  # start with mav manager
                "is_sitl": True,
                "mass_sitl": mass_sitl,
                "t_hover_sitl": t_hover_sitl,
            },
        ],
        remappings=[
            ("odom", "odom"),
            # Remap to global PX4 topics (not namespaced). idk why this is needed but gazebo won't arm wihtout it
            ("fmu/in/offboard_control_mode", "/fmu/in/offboard_control_mode"),
            ("fmu/in/vehicle_command", "/fmu/in/vehicle_command"),
            ("fmu/in/vehicle_rates_setpoint", "/fmu/in/vehicle_rates_setpoint"),
            ("fmu/in/vehicle_thrust_setpoint", "/fmu/in/vehicle_thrust_setpoint"),
            ("fmu/in/vehicle_torque_setpoint", "/fmu/in/vehicle_torque_setpoint"),
            ("fmu/in/trajectory_setpoint", "/fmu/in/trajectory_setpoint"),
            ("fmu/out/vehicle_status_v1", "/fmu/out/vehicle_status_v1"),
            ("fmu/out/vehicle_control_mode", "/fmu/out/vehicle_control_mode"),
            ("fmu/out/vehicle_command_ack", "/fmu/out/vehicle_command_ack"),
        ],
        output="screen",
    )

    # ========== Control Stack (DQNMPC) ==========

    tracker_manager_node = ComposableNode(
        package="trackers_manager",
        plugin="trackers_manager::TrackersManager",
        namespace=quad_name,
        name="trackers_manager_node",
        parameters=[
            control_config,
            {
                "mass": mass_sitl,
                "use_sim_time": use_sim_time,
            },
        ],
        remappings=[
            ("cmd", "position_cmd"),
            ("odom", "odom"),  # Added odom remapping
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    mav_service_node = ComposableNode(
        package="mav_manager",
        plugin="mav_manager::MAVManager",
        namespace=quad_name,
        name="mav_manager_service_node",
        parameters=[
            control_config,
            {
                "mass": mass_sitl,
                "use_sim_time": use_sim_time,
            },
        ],
        remappings=[
            ("~/odom", "odom"),
            ("~/imu", "imu"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # DQ-NMPC Controller (C++ composable node)
    # dq_cpp subscribes to: odom, position_cmd, imu, motors
    # dq_cpp publishes to: trpy_cmd, reference_path, predicted_path, reference_pose
    dqnmpc_control_node = ComposableNode(
        package="dq_cpp",
        plugin="dq_nmpc_control_nodelet::NMPCControlNodelet",
        namespace=quad_name,
        name="nmpc_control_nodelet",
        parameters=[
            control_config,
            {
                "use_sim_time": use_sim_time,
                "mass": mass_sitl,
            },
        ],
        remappings=[
            ("odom", "odom"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # Control container (all composable nodes including NMPC)
    control_container = ComposableNodeContainer(
        name="control_container",
        namespace=quad_name,
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            tracker_manager_node,
            mav_service_node,
            dqnmpc_control_node,
        ],
        output="screen",
    )

    # ========== Point Cloud Processing ==========

    # Transform point cloud to world frame (only when nvblox is disabled)
    pc2_reheader_transform_node = Node(
        package="pc2_tools_cpp",
        executable="pc2reheader_transform",
        name="pc2_reheader_transform_node",
        namespace=quad_name,
        parameters=[
            {
                "in": "/depth_camera/points",
                "out": "/depth_camera/points_world",
                "source_frame": "depth_camera_link",
                "target_frame": "world",
                "startup_delay": 3.0,
                "skip_points": 8,
                "use_sim_time": use_sim_time,
            }
        ],
        output="screen",
        condition=no_nvblox_condition,
    )

    # Static TF for camera depth frame
    stf_camera_depth_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="stf_camera_depth_frame",
        namespace=quad_name,
        arguments=[
            "0",
            "0",
            "0",
            "3.14159",
            "-1.5708",
            "-1.5708",
            "camera_link",
            "depth_camera_link",
        ],
        output="screen",
    )

    # Static TF for depth camera base link (Gazebo convention)
    depth_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace=quad_name,
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "1",
            "base_link",
            "x500_depth_0/OakD-Lite/base_link/StereoOV7251",
        ],
        output="screen",
    )

    # ========== Fast-Planner Node ==========

    fast_planner_node = Node(
        package="plan_manage",
        executable="fast_planner_node",
        name="fast_planner_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                # Topology-based replanning
                "planner_node/planner": 2,
                # Manager parameters
                "manager/max_vel": LaunchConfiguration("max_vel"),
                "manager/max_acc": LaunchConfiguration("max_acc"),
                "manager/max_jerk": 4.0,
                "manager/dynamic_environment": 0,
                "manager/clearance_threshold": 0.4,
                "manager/local_segment_length": 6.0,
                "manager/control_points_distance": 0.5,
                "manager/use_geometric_path": False,
                "manager/use_kinodynamic_path": False,
                "manager/use_topo_path": True,
                "manager/use_optimization": True,
                # Planning FSM
                "fsm/flight_type": 3,  # REFERENCE_PATH mode
                "fsm/waypoint_num": 0,
                "fsm/thresh_replan": 1.0,
                "fsm/thresh_no_replan": 3.0,
                # SDF map
                "sdf_map/resolution": 0.15,
                "sdf_map/map_size_x": LaunchConfiguration("map_size_x"),
                "sdf_map/map_size_y": LaunchConfiguration("map_size_y"),
                "sdf_map/map_size_z": LaunchConfiguration("map_size_z"),
                "sdf_map/local_update_range_x": 4.0,
                "sdf_map/local_update_range_y": 4.0,
                "sdf_map/local_update_range_z": 3.0,
                "sdf_map/obstacles_inflation": 0.05,
                "sdf_map/local_bound_inflate": 0.5,
                "sdf_map/local_map_margin": 30,
                "sdf_map/ground_height": -1.0,
                "sdf_map/virtual_ceil_height": 2.5,
                "sdf_map/visualization_truncate_height": 15.0,
                "sdf_map/esdf_slice_height": 1.5,
                # Camera parameters
                "sdf_map/cx": 320.0,
                "sdf_map/cy": 240.0,
                "sdf_map/fx": 426.4,
                "sdf_map/fy": 426.4,
                "sdf_map/use_depth_filter": False,
                "sdf_map/depth_filter_tolerance": 0.15,
                "sdf_map/depth_filter_maxdist": 4.5,
                "sdf_map/depth_filter_mindist": 0.2,
                "sdf_map/depth_filter_margin": 2,
                "sdf_map/k_depth_scaling_factor": 1000.0,
                "sdf_map/skip_pixel": 8,
                # Nvblox integration
                "sdf_map/use_nvblox": use_nvblox_arg,
                "nvblox/esdf_topic": "/nvblox_node/esdf_slice",
                "nvblox/occupancy_topic": "/nvblox_node/static_map_slice",
                # Topology-based planning
                "topo_prm/sample_inflate_x": 1.0,
                "topo_prm/sample_inflate_y": 2.5,
                "topo_prm/sample_inflate_z": 0.2,
                "topo_prm/clearance": 0.5,
                "topo_prm/max_sample_time": 0.003,
                "topo_prm/max_sample_num": 500,
                "topo_prm/max_raw_path": 300,
                "topo_prm/max_raw_path2": 10,
                "topo_prm/short_cut_num": 1,
                "topo_prm/reserve_num": 3,
                "topo_prm/ratio_to_short": 5.5,
                "topo_prm/parallel_shortcut": True,
                # B-spline optimization
                "optimization/lambda1": 10.0,
                "optimization/lambda2": 10.0,
                "optimization/lambda3": 0.0,
                "optimization/lambda4": 0.001,
                "optimization/lambda5": 1.5,
                "optimization/lambda6": 10.0,
                "optimization/lambda7": 20.0,
                "optimization/lambda8": 0.0,
                "optimization/dist0": 0.6,
                "optimization/max_vel": LaunchConfiguration("max_vel"),
                "optimization/max_acc": LaunchConfiguration("max_acc"),
                "optimization/visib_min": 0.2,
                "optimization/wnl": 1.5,
                "optimization/dlmin": 0.1,
                "optimization/algorithm1": 15,
                "optimization/algorithm2": 11,
                "optimization/max_iteration_num1": 2,
                "optimization/max_iteration_num2": 50,
                "optimization/max_iteration_num3": 30,
                "optimization/max_iteration_num4": 30,
                "optimization/max_iteration_time1": 0.0001,
                "optimization/max_iteration_time2": 0.01,
                "optimization/max_iteration_time3": 0.01,
                "optimization/max_iteration_time4": 0.01,
                "optimization/order": 3,
                # B-spline limits
                "bspline/limit_vel": LaunchConfiguration("max_vel"),
                "bspline/limit_acc": LaunchConfiguration("max_acc"),
                "bspline/limit_ratio": 1.1,
            }
        ],
        remappings=[
            # Fast-Planner expects global topics, remap to namespaced quadrotor topics
            ("/odom_world", "/quadrotor/odom"),
            ("/sdf_map/odom", "/quadrotor/odom"),
            ("/sdf_map/cloud", "/depth_camera/points_world"),
            ("/sdf_map/pose", "/camera_pose"),
            ("/sdf_map/depth", "/depth_camera"),
        ],
    )

    # ========== Fast-Planner to DQNMPC Bridge ==========

    bridge_node = Node(
        package="fastplanner_to_nmpc_bridge",
        executable="bridge_node",
        name="fastplanner_nmpc_bridge",
        namespace=quad_name,
        parameters=[
            control_config,
            {
                "use_sim_time": use_sim_time,
                "publish_topic": "position_cmd",  # Matches dq_nmpc subscription
                "frame_id": "world",
            },
        ],
        remappings=[
            ("/planning/bspline", "/planning/bspline"),  # Subscribe to Fast-Planner
        ],
        output="screen",
    )

    # ========== Nvblox (optional) ==========

    nvblox_node = ComposableNode(
        package="nvblox_ros",
        plugin="nvblox::NvbloxNode",
        namespace=quad_name,
        name="nvblox_node",
        remappings=[
            (["/", quad_name, "/camera_0/depth/image"], "/depth_camera"),
            (["/", quad_name, "/camera_0/depth/camera_info"], "/camera_info"),
        ],
        parameters=[
            nvblox_config_base,
            {
                "global_frame": "world",
                "use_sim_time": use_sim_time,
                "use_tf_transforms": True,
                "use_topic_transforms": False,
                "use_depth": True,
                "use_lidar": False,
            },
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    mapping_container = ComposableNodeContainer(
        name="map_container",
        namespace=quad_name,
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            nvblox_node,
        ],
        output="screen",
    )

    # ========== Launch Description ==========

    ld = LaunchDescription(launch_args)

    # PX4/Gazebo - NOTE: microxrce and px4 are started by fastplanner_test.sh
    # Uncomment if running this launch file standalone
    # ld.add_action(microxrce)
    # ld.add_action(px4_gazebo_command)
    ld.add_action(px4_odom_transform)
    ld.add_action(px4_offboard_node)

    # Depth camera
    ld.add_action(gz_depth_bridge_node)
    ld.add_action(stf_camera_depth_frame)
    ld.add_action(depth_static_tf)

    # Control stack (dq_cpp NMPC is inside the container)
    ld.add_action(control_container)

    # Point cloud processing (when nvblox disabled)
    ld.add_action(pc2_reheader_transform_node)

    # Fast-Planner and bridge
    ld.add_action(fast_planner_node)
    ld.add_action(bridge_node)

    # Nvblox mapping (optional)
    ld.add_action(
        GroupAction(condition=IfCondition(use_mapping), actions=[mapping_container]),
    )

    return ld
