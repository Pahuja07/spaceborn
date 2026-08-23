"""RTAB-Map RGB-D SLAM for the Clearpath Husky A200, plus optional Nav2.

Pipeline
--------
    Gazebo RealSense  ->  rgbd_sync  ->  rgbd_odometry  ->  rtabmap
                                             ^   |                 |
                        IMU (orientation) ---+   |                 +-> /a200_0000/map       (2D grid, for Nav2)
                                                 |                 +-> /a200_0000/cloud_map (3D cloud)
                                                 +-> /a200_0000/odom (+ odom->base_link TF)

Where the IMU fits
------------------
Translation is estimated purely from vision.  The IMU contributes ORIENTATION
only, in two places:

* ``rgbd_odometry`` -- gravity-aligns the initial pose, and substitutes the
  measured rotation for the constant-velocity model's rotation in each frame's
  guess.  Requires ``wait_imu_to_init: true``; without it rtabmap_odom never
  creates the IMU subscription at all and the ``imu`` remapping is inert.
* ``rtabmap`` -- one gravity link per graph node, used by the optimizer to keep
  poses upright (``Optimizer/GravitySigma``).

This is *not* tightly-coupled VIO: there is no IMU preintegration and no bias
estimation, and linear acceleration is never integrated.  The upside is that an
IMU dropout degrades to plain visual odometry instead of diverging.  RTAB-Map's
true VIO backends (``Odom/Strategy`` 6/8/9/10 = OKVIS / MSCKF_VIO / VINS-Fusion
/ OpenVINS) are NOT compiled into the apt build on this machine -- the library
answers "RTAB-Map is not built with <X> support!" for every one of them -- so
using one means building rtabmap from source.

Prerequisite: TF ``base_link -> imu_0_link`` must exist, or every IMU sample is
dropped with "A valid TF between base_link and imu_0_link is required to
initialize IMU" and, because ``wait_imu_to_init`` is true, odometry never
starts.  Clearpath's ``robot_state_publisher`` provides it from the
``microstrain_imu`` block in ``~/clearpath/robot.yaml``.

Frames
------
Topics are namespaced (``/a200_0000/...``) but TF frame *names* are BARE
(``map``, ``odom``, ``base_link``, ``camera_0_link``) because Clearpath's
``robot_state_publisher`` runs without ``frame_prefix``.  TF itself travels on
the namespaced ``/a200_0000/tf`` topics, hence the remappings on every node.

``camera_0_color_optical_frame`` is *not* a URDF link -- it only exists as the
``<optical_frame_id>`` inside the Gazebo ``<sensor>`` block, i.e. it is the
frame_id Gazebo stamps onto the images.  Nothing publishes it, so the static TF
below supplies it, parented to the real link ``camera_0_link``.

Database
--------
The map is written to ``~/clearpath_ws/rtabmap_3d.db``.  Pass
``delete_db_on_start:=true`` to start a fresh map instead of resuming.

Usage
-----
    # terminal 1
    ros2 launch clearpath_gz simulation.launch.py
    # terminal 2 -- SLAM + dashboard + RViz + Nav2 + autonomous exploration
    ros2 launch clearpath_config rtabmap_rgbd_sync_launch.py nav2:=true explore:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = 'a200_0000'

    database_path = '/media/jaydhake/DATA_DRIVE/clearpath_ws/rtabmap_3d.db'

    nav2 = LaunchConfiguration('nav2')
    explore = LaunchConfiguration('explore')
    dashboard = LaunchConfiguration('dashboard')
    rviz = LaunchConfiguration('rviz')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')

    args = [
        DeclareLaunchArgument(
            'nav2', default_value='false',
            description='Also bring up Nav2 (spaceborn_nav/nav2.launch.py).'),
        DeclareLaunchArgument(
            'explore', default_value='true',
            description='With nav2:=true, run frontier_explorer for autonomous '
                        'exploration. Set false to send goals manually from RViz.'),
        DeclareLaunchArgument(
            'dashboard', default_value='true',
            description='Run the OpenCV dashboard (RGB / depth / 2D grid / 3D cloud).'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Run RViz with the rtabmap config.'),
        DeclareLaunchArgument(
            'delete_db_on_start', default_value='false',
            description=f'Delete {database_path} and start a fresh map.'),
    ]

    # qos 1 = Reliable, matching the Gazebo bridge publisher QoS (confirmed RELIABLE)
    common_params = {'use_sim_time': True, 'qos_image': 1, 'qos_camera_info': 1}

    sync_params = {
        'approx_sync': True,
        'approx_sync_max_interval': 0.05,
        'sync_queue_size': 20,
    }

    # BARE frame names -- robot_state_publisher sets no frame_prefix.
    frame_params = {
        'frame_id': 'base_link',
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        # Give TF a moment; the Gazebo clock and the sensor stamps drift slightly.
        'wait_for_transform': 0.3,
    }

    # Occupancy grid tuning.  RTAB-Map's grid is what Nav2's global costmap and
    # the frontier explorer consume, so it has to be a proper 2D map with
    # ray-traced free space -- not just a sprinkle of obstacle cells.
    grid_params = {
        # 1 = build the grid from the depth camera (no lidar on this robot).
        'Grid/Sensor': '1',
        # Keep a real 3D occupancy cloud for /cloud_map and the dashboard; the
        # published 2D /map is the ground projection of it.
        'Grid/3D': 'true',
        'Grid/CellSize': '0.05',
        'Grid/RangeMin': '0.4',
        'Grid/RangeMax': '8.0',
        # Height passthrough beats normal segmentation here: the camera is
        # mounted level and the sim floor is flat, so plain height thresholds
        # are far more stable than estimated normals.
        'Grid/NormalsSegmentation': 'false',
        'Grid/MaxGroundHeight': '0.10',
        'Grid/MaxObstacleHeight': '1.50',
        # Carve out known-free space between the camera and each hit. Without
        # this the map is all obstacles and unknown, every cell looks like a
        # frontier, and Nav2 has nothing to plan through.
        'Grid/RayTracing': 'true',
        # Never let the robot's own body appear as an obstacle.
        'Grid/FootprintLength': '1.10',
        'Grid/FootprintWidth': '0.72',
        'Grid/FootprintHeight': '0.50',
        # Start the global map big enough that the costmap has room to grow into.
        'GridGlobal/MinSize': '20.0',
        'GridGlobal/Eroded': 'false',
        'GridGlobal/OccupancyThr': '0.55',
    }

    # The Husky drives on a plane, so constrain the graph to 2D: it removes
    # roll/pitch drift that would otherwise tilt the projected grid.
    # NOTE: for the drone this must be dropped -- see the 6-DoF notes.
    slam_2d_params = {
        'Reg/Force3DoF': 'true',
        'Optimizer/Slam2D': 'true',
    }

    # Visual odometry robustness.  Everything here previously ran at stock
    # defaults, which lose tracking during rotation and then never recover.
    odom_params = {
        # THE important one.  Gazebo's depth far clip is 100 m, so with the
        # default MaxDepth=0 (unlimited) RTAB-Map triangulates features 30-60 m
        # out from a 72 deg FOV camera.  Those points have near-infinite depth
        # uncertainty; when the robot yaws they sweep across the image and the
        # solver fits to them, producing 40 m per-frame translation estimates.
        # A real D435 returns nothing past ~10 m, so this cap is what the
        # hardware would have enforced for us.
        'Vis/MaxDepth': '6.0',
        'Vis/MinDepth': '0.4',
        # Default 0 = never auto-reset.  Once odometry is lost it stays lost,
        # odom->base_link freezes and the dashboard loses map->base_link.
        # 5 frames at 30 Hz = recover in under 0.2 s, but still ride out a
        # single bad frame without throwing the local map away.
        'Odom/ResetCountdown': '5',
        # Default 40 px restricts matching to a window around the projected
        # guess.  When the guess is wrong that guarantees zero correspondences
        # and locks in the failure.  0 = match across the whole image, so a bad
        # guess is recoverable.
        'Vis/CorGuessWinSize': '0',
        # 72 deg FOV plus no wheel odometry means rotation is the hard case.
        # More features and a larger local map keep enough overlap alive.
        'Vis/MaxFeatures': '1500',
        'OdomF2M/MaxSize': '3000',
        # Default 20 inliers is strict for a low-texture sim world; 12 still
        # geometrically constrains the pose but tolerates sparser scenes.
        'Vis/MinInliers': '12',
        # Differential drive -- it cannot strafe.  Telling the motion model
        # this removes a whole family of bogus lateral guesses.
        # NOTE: for the drone this must be dropped -- a quadrotor IS holonomic.
        'Odom/Holonomic': 'false',
    }

    # --- IMU -------------------------------------------------------------
    # These are ROS node parameters, not Rtabmap/ config keys, so they are kept
    # separate from odom_params.
    #
    # wait_imu_to_init is the ON/OFF SWITCH for the whole IMU path, not merely a
    # startup delay: rtabmap_odom only ever calls create_subscription<Imu> when
    # this is true.  With the default false the `imu` remapping in this file was
    # dead -- the topic existed, the remap resolved, and nothing subscribed.
    # Verified by running rgbd_odometry both ways: true prints
    # "odometry: Subscribing to IMU topic /a200_0000/sensors/imu_0/data",
    # false prints nothing and creates no subscription.
    #
    # What the IMU actually buys us (F2M is NOT tightly-coupled VIO -- there is
    # no preintegration and no bias estimation, see the drone note below):
    #   1. gravity-aligned initial orientation instead of assuming identity;
    #   2. the IMU rotation replaces the constant-velocity model's rotation in
    #      the per-frame guess ("Adjusting guess from motion with IMU").  That
    #      directly targets our failure mode -- rotation is where this 72 deg
    #      FOV camera loses tracking, and a measured yaw beats an extrapolated
    #      one whenever angular velocity changes.
    #
    # The IMU is only consulted for ORIENTATION; linear acceleration is never
    # integrated, so translation stays purely visual.  A dropout therefore
    # degrades to today's behaviour rather than diverging.
    #
    # qos_imu is deliberately left at its default: rtabmap_conversions casts it
    # straight to rmw_qos_reliability_policy_t, where 0 = SYSTEM_DEFAULT
    # (reliable), and the Gazebo bridge's IMU publisher is reliable -- the
    # previous run's rtabmap received it at that default ("receiving imu data
    # (buffer=21)").  imu_queue_size is likewise already 200 by default, which
    # is ~2 s of history at the sensor's 100 Hz.
    imu_params = {
        'wait_imu_to_init': True,
    }

    # Loop-closure tuning for a warehouse -- an environment full of repeated
    # structure (identical racking, identical aisles), which is the worst case
    # for appearance-based place recognition.  The previous run accepted 16
    # loop closures and rejected 362, and some of the 16 were wrong: the robot
    # matched one aisle to another and teleported.
    loop_params = {
        # 1 Hz, not 2.  At 2 Hz the budget per iteration is 0.5 s and RTAB-Map
        # was measured at 0.30-0.67 s with 5 s spikes, i.e. permanently over
        # budget.  That backlog is what produced 299 "Lookup would require
        # extrapolation into the future" warnings: rtabmap was registering
        # against poses that had already gone stale.
        'Rtabmap/DetectionRate': '1',
        'Kp/MaxFeatures': '750',
        'RGBD/AngularUpdate': '0.1',
        'RGBD/LinearUpdate': '0.1',

        # --- rejecting false places ---------------------------------------
        # Inliers must be spread across the image (2nd PCA eigenvalue of the
        # keypoint distribution), not clustered on one object.  A repeated
        # rack face gives a tight cluster of geometrically consistent matches
        # and nothing else -- exactly the signature of matching aisle 3 to
        # aisle 7.  Range is 0..0.5; 0 disables the check entirely.
        'Vis/MinInliersDistribution': '0.05',
        # Loop-closure side only (rgbd_odometry keeps 12 via odom_params).
        # A loop closure is far more expensive to get wrong than an odometry
        # frame, so it pays to be strict here.
        'Vis/MinInliers': '25',

        # --- accepting true places ----------------------------------------
        # Ratio is abs error over the link's std dev, and 236 of the rejections
        # cited std dev = 0.031623 (= sqrt(0.001), RTAB-Map's covariance floor,
        # because odometry reports an implausible 0.0001 m).  At 3.0 that floor
        # caps the admissible correction at ~9.5 cm of translation and ~5.4 deg
        # of rotation, so 205 of 498 rejections landed in the 3-4 band -- real
        # loops thrown out for correcting real drift.  5.0 roughly doubles the
        # allowance while still rejecting the >3 m / >45 deg outliers.
        'RGBD/OptimizeMaxError': '5.0',
        # Vertigo switchable constraints.  Lets the optimizer down-weight a
        # single bad edge instead of RTAB-Map discarding the whole batch --
        # "Rejecting all added loop closures in this iteration" means one bad
        # candidate was taking good ones down with it.  Needs g2o or GTSAM;
        # Optimizer/Strategy defaults to 2 (GTSAM) and libgtsam is linked in.
        'Optimizer/Robust': 'true',
        # A wrong loop closure that gets accepted is permanent at the default
        # 0.0: it stays in the graph and keeps bending the map. Non-zero lets
        # RTAB-Map delete an old link that two later closures disagree with.
        'RGBD/OptimizeMaxErrorRepairRadius': '1.0',

        # --- gravity constraints ------------------------------------------
        # Left at defaults on purpose, both of them:
        #
        # Mem/UseOdomGravity stays FALSE so gravity links keep coming from the
        # raw IMU.  Switching it on looks tempting now that rgbd_odometry is
        # gravity-aligned, but Reg/Force3DoF plus RGBD/ForceOdom3DoF (true by
        # default) zero roll and pitch in the odometry pose, so gravity read off
        # that pose would be a degenerate "always perfectly upright" link -- and
        # it would stay silently wrong on the drone, where real tilt matters.
        # The raw-IMU path already works: the database has one gravity link per
        # node (144/144).  The occasional "cannot interpolate imu transform"
        # warning is a startup-window race, not a lost constraint.
        #
        # Optimizer/GravitySigma stays at 0.3.  It does almost nothing today
        # because Optimizer/Slam2D already pins roll and pitch to zero, but it
        # is what will hold the graph upright on the drone once slam_2d_params
        # is dropped.
    }

    # All TF lives on the namespaced topics, not global /tf
    tf_remappings = [
        ('/tf', f'/{namespace}/tf'),
        ('/tf_static', f'/{namespace}/tf_static'),
    ]

    rgbd_sync_node = Node(
        package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync',
        namespace=namespace, output='screen',
        parameters=[common_params, sync_params],
        remappings=tf_remappings + [
            ('rgb/image',       f'/{namespace}/sensors/camera_0/color/image'),
            ('depth/image',     f'/{namespace}/sensors/camera_0/depth/image'),
            ('rgb/camera_info', f'/{namespace}/sensors/camera_0/color/camera_info'),
        ]
    )

    # rgbd_odometry: RGB-D visual odometry with IMU-aided orientation (no wheel
    # odom, so this transfers to the drone unchanged).  Translation comes purely
    # from vision; the IMU supplies gravity alignment and the rotation half of
    # the per-frame guess -- see imu_params.
    #
    # It owns the odom->base_link TF, so the platform EKF must not publish TF
    # (robot.yaml keeps enable_ekf: false for exactly this reason).  That also
    # means there is NO wheel-odometry fallback: if this node loses tracking,
    # nothing else is estimating the pose.
    rgbd_odometry_node = Node(
        package='rtabmap_odom', executable='rgbd_odometry', name='rgbd_odometry',
        namespace=namespace, output='screen',
        parameters=[common_params, frame_params, slam_2d_params, odom_params,
                    imu_params, {'subscribe_rgbd': True}],
        remappings=tf_remappings + [
            ('odom', f'/{namespace}/odom'),
            ('imu',  f'/{namespace}/sensors/imu_0/data'),
        ]
    )

    rtabmap_params = [
        common_params, frame_params, grid_params, slam_2d_params, loop_params,
        {'subscribe_rgbd': True},
        {'database_path': database_path},
    ]
    rtabmap_remappings = tf_remappings + [
        ('odom', f'/{namespace}/odom'),
        ('imu',  f'/{namespace}/sensors/imu_0/data'),
    ]

    # Two variants of the same node, differing only in the '-d' argument, which
    # wipes the database so the next run starts a fresh map.  It has to be done
    # this way because there is no "resume" flag to pass in the other case:
    # rtabmap only accepts '--udebug' and friends, so the resume variant must
    # carry no arguments at all rather than a substituted no-op.
    rtabmap_node_fresh = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        namespace=namespace, output='screen',
        condition=IfCondition(delete_db_on_start),
        parameters=rtabmap_params,
        arguments=['-d'],
        remappings=rtabmap_remappings
    )

    rtabmap_node_resume = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        namespace=namespace, output='screen',
        condition=UnlessCondition(delete_db_on_start),
        parameters=rtabmap_params,
        remappings=rtabmap_remappings
    )

    # camera_0_color_optical_frame is the frame_id Gazebo stamps on the images
    # but it is not a URDF link, so publish it here off the real camera link.
    # Args are x y z yaw pitch roll parent child: the standard link->optical
    # rotation (x fwd/y left/z up  ->  z fwd/x right/y down).
    static_tf_cam = Node(
        package='tf2_ros', executable='static_transform_publisher', name='static_tf_cam',
        namespace=namespace, output='screen',
        arguments=['0', '0', '0', '-1.570796', '0', '-1.570796',
                   'camera_0_link', 'camera_0_color_optical_frame'],
        remappings=tf_remappings
    )

    dashboard_node = Node(
        package='spaceborn_nav', executable='rtabmap_dashboard',
        name='rtabmap_dashboard', namespace=namespace, output='screen',
        condition=IfCondition(dashboard),
        parameters=[{
            'use_sim_time': True,
            'robot_namespace': namespace,
            'map_frame': 'map',
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'depth_min': 0.3,
            'depth_max': 8.0,
            'gps_topic': f'/{namespace}/sensors/gps_0/fix_jammed',
        }],
        remappings=tf_remappings
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('clearpath_config'),
        'rviz', 'rtabmap.rviz'
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen',
        condition=IfCondition(rviz),
        arguments=['-d', rviz_config_path],
        remappings=tf_remappings
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('spaceborn_nav'), 'launch', 'nav2.launch.py')),
        condition=IfCondition(nav2),
        launch_arguments={
            'use_sim_time': 'true',
            'explore': explore,
        }.items(),
    )

    signal_jammer_node = Node(
        package='spaceborn_nav', executable='signal_jammer',
        name='signal_jammer', namespace=namespace, output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription(args + [
        static_tf_cam,
        rgbd_sync_node,
        rgbd_odometry_node,
        rtabmap_node_fresh,
        rtabmap_node_resume,
        dashboard_node,
        rviz_node,
        nav2_launch,
        signal_jammer_node,
    ])
