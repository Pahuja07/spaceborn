"""Nav2 bring-up for the Husky A200 on top of RTAB-Map RGB-D SLAM.

What this launch file starts
----------------------------
1. ``depthimage_to_laserscan`` -- the robot has no lidar, so the single 2D
   obstacle source for the Nav2 costmaps is a scan synthesised from the
   RealSense depth image.
2. Nav2's ``navigation_launch.py`` (controller / planner / behaviours /
   bt_navigator / smoother / velocity_smoother / waypoint_follower +
   lifecycle manager) wrapped in ``PushRosNamespace``.
3. Optionally ``frontier_explorer`` -- picks frontier goals off RTAB-Map's
   occupancy grid and drives the robot there via ``NavigateToPose``, so the
   robot maps autonomously and then keeps navigating the finished map.

What it deliberately does NOT start
-----------------------------------
* ``localization_launch.py`` / AMCL / ``map_server``.  RTAB-Map already owns
  the ``map -> odom`` transform and publishes ``/a200_0000/map``.  Running AMCL
  as well would give two publishers of ``map -> odom`` and the TF tree would
  flap.
* Anything that publishes ``odom -> base_link``.  That comes from
  ``rgbd_odometry`` in ``rtabmap_rgbd_sync_launch.py``.

Namespacing
-----------
Topics are namespaced (``/a200_0000/...``); TF frame *names* are bare
(``map``, ``odom``, ``base_link``) because Clearpath's ``robot_state_publisher``
runs without ``frame_prefix``.  ``navigation_launch.py`` remaps ``/tf`` ->
``tf``, which resolves to ``/a200_0000/tf`` once the namespace is pushed, so
the nav stack reads the same TF topic the rest of the robot uses.

Typical use::

    # terminal 1 -- simulator
    ros2 launch clearpath_gz simulation.launch.py
    # terminal 2 -- SLAM + dashboard + RViz + Nav2 + exploration
    ros2 launch clearpath_config rtabmap_rgbd_sync_launch.py nav2:=true explore:=true

or Nav2 on its own against an already-running RTAB-Map::

    ros2 launch spaceborn_nav nav2.launch.py explore:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    namespace = 'a200_0000'

    nav_share = get_package_share_directory('spaceborn_nav')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    default_params = os.path.join(nav_share, 'config', 'nav2_husky.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    explore = LaunchConfiguration('explore')
    autostart = LaunchConfiguration('autostart')
    scan = LaunchConfiguration('scan')

    args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo /clock.'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Nav2 parameter file.'),
        DeclareLaunchArgument(
            'explore', default_value='true',
            description='Start frontier_explorer for autonomous exploration. '
                        'Set false to drive Nav2 manually from RViz 2D Goal Pose.'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Let the lifecycle manager configure+activate Nav2 on start.'),
        DeclareLaunchArgument(
            'scan', default_value='true',
            description='Start depthimage_to_laserscan. Set false if a real lidar '
                        'or another scan source already publishes on scan.'),
    ]

    # All TF lives on the namespaced topics, never global /tf.
    tf_remappings = [
        ('/tf', f'/{namespace}/tf'),
        ('/tf_static', f'/{namespace}/tf_static'),
    ]

    # --- depth image -> 2D laser scan -------------------------------------
    # Gazebo stamps the RGB-D images with the <optical_frame_id> from the URDF,
    # camera_0_color_optical_frame (z forward, x right, y down).  A LaserScan
    # sweeps about the frame's z axis, so the scan must be stamped in the
    # ROS-convention sibling frame camera_0_link (x forward, y left, z up) --
    # otherwise the costmap would place every obstacle in a vertical plane.
    # scan_height=10 takes the closest return over 10 rows around the image
    # centre instead of a single row, which is far less brittle.  The camera is
    # mounted level (rpy 0 0 0), so those rows never graze the floor.
    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        namespace=namespace,
        output='screen',
        condition=IfCondition(scan),
        parameters=[{
            'use_sim_time': use_sim_time,
            'output_frame': 'camera_0_link',
            'scan_height': 10,
            'scan_time': 0.033,
            # Gazebo's near clip is 0.30 m; stay just outside it so the near
            # plane is not reported as a wall right in front of the robot.
            'range_min': 0.35,
            'range_max': 8.0,
        }],
        remappings=tf_remappings + [
            ('depth', f'/{namespace}/sensors/camera_0/depth/image'),
            ('depth_camera_info', f'/{namespace}/sensors/camera_0/depth/camera_info'),
            ('scan', f'/{namespace}/scan'),
        ],
    )

    # --- Nav2 -------------------------------------------------------------
    # navigation_launch.py declares no namespace on its nodes and contains no
    # PushRosNamespace, so wrap it.  The namespace argument is still needed
    # because RewrittenYaml uses it as the params root_key and the lifecycle
    # manager builds node names from it.
    nav2 = GroupAction([
        PushRosNamespace(namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
                'use_composition': 'False',
                'use_respawn': 'False',
            }.items(),
        ),
    ])

    # --- autonomous exploration ------------------------------------------
    frontier_explorer = Node(
        package='spaceborn_nav',
        executable='frontier_explorer',
        name='frontier_explorer',
        namespace=namespace,
        output='screen',
        condition=IfCondition(explore),
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_topic': 'map',
            'nav_action': 'navigate_to_pose',
            'map_frame': 'map',
            'base_frame': 'base_link',
            # Ignore specks: RTAB-Map's projected grid is noisy at the edges.
            'min_frontier_cells': 12,
            # Keep goals off walls -- half the Husky width plus a margin.
            'obstacle_clearance': 0.45,
            'gain_weight': 1.0,
            'distance_weight': 0.55,
            # Give up on a frontier after 40 s and move to the next one. At the
            # 0.40 m/s speed cap that is still ~16 m of driving, so it only
            # fires on goals the robot genuinely cannot reach.
            'goal_timeout': 40.0,
            # Radius around an abandoned goal that becomes off-limits. 0.8 m was
            # smaller than the Husky is long, so after a timeout the next-best
            # frontier was often another cell on the same unreachable shelf face
            # and the robot retried the same spot. 1.2 m forces it elsewhere.
            'blacklist_radius': 1.2,
            'min_goal_distance': 0.5,
            'planning_period': 2.0,
            # Raised with blacklist_radius: a bigger exclusion zone can mask
            # every frontier for a tick or two, and 3 dry rounds would then
            # declare the map finished while unexplored space remained.
            'dry_rounds_before_done': 5,
            # Once no frontier is left, keep driving the finished map.
            'patrol_when_done': True,
            'publish_markers': True,
        }],
        remappings=tf_remappings,
    )

    return LaunchDescription(args + [
        depth_to_scan,
        nav2,
        frontier_explorer,
    ])
