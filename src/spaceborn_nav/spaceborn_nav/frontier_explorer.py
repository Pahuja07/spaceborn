#!/usr/bin/env python3
"""Frontier-based autonomous exploration on top of Nav2 + RTAB-Map.

The robot has no lidar and no prior map, so exploration is driven purely by the
occupancy grid RTAB-Map projects from the RGB-D camera:

  1. find *frontiers* -- free cells that touch unknown cells,
  2. cluster them, score each cluster by (size gained) vs (distance to drive),
  3. send the best cluster centroid to Nav2 as a NavigateToPose goal,
  4. repeat until no frontier is left, then (optionally) patrol the finished map
     forever so the robot keeps "driving the map freely".

Goals that Nav2 aborts, or that time out, are blacklisted so the robot does not
livelock on an unreachable frontier.
"""

import math
import random

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('nav_action', 'navigate_to_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        # A frontier cluster smaller than this (in cells) is sensor noise, not a doorway.
        self.declare_parameter('min_frontier_cells', 12)
        # Half the Husky's diagonal plus margin: frontier goals closer than this to
        # an obstacle are unreachable in practice.
        self.declare_parameter('obstacle_clearance', 0.45)
        self.declare_parameter('gain_weight', 1.0)
        self.declare_parameter('distance_weight', 0.55)
        self.declare_parameter('goal_timeout', 60.0)
        self.declare_parameter('blacklist_radius', 0.8)
        self.declare_parameter('min_goal_distance', 0.5)
        self.declare_parameter('planning_period', 2.0)
        self.declare_parameter('dry_rounds_before_done', 3)
        self.declare_parameter('patrol_when_done', True)
        self.declare_parameter('publish_markers', True)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.min_cells = int(self.get_parameter('min_frontier_cells').value)
        self.clearance = float(self.get_parameter('obstacle_clearance').value)
        self.gain_w = float(self.get_parameter('gain_weight').value)
        self.dist_w = float(self.get_parameter('distance_weight').value)
        self.goal_timeout = float(self.get_parameter('goal_timeout').value)
        self.blacklist_radius = float(self.get_parameter('blacklist_radius').value)
        self.min_goal_distance = float(self.get_parameter('min_goal_distance').value)
        self.dry_limit = int(self.get_parameter('dry_rounds_before_done').value)
        self.patrol = bool(self.get_parameter('patrol_when_done').value)

        self.grid = None
        self.blacklist = []
        self.goal_handle = None
        self.goal_xy = None
        self.goal_sent_at = None
        self.dry_rounds = 0
        self.exploring = True
        self.goals_reached = 0
        self.rng = random.Random(0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value, self._on_map, map_qos)

        self.markers = None
        if bool(self.get_parameter('publish_markers').value):
            self.markers = self.create_publisher(MarkerArray, 'frontiers', 1)

        self.nav = ActionClient(self, NavigateToPose, self.get_parameter('nav_action').value)

        self.create_timer(float(self.get_parameter('planning_period').value), self._tick)
        self.get_logger().info('frontier explorer started -- waiting for Nav2 and /map')

    # ---------------------------------------------------------------- input

    def _on_map(self, msg: OccupancyGrid):
        self.grid = msg

    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001 - TF is simply not ready yet
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    # ------------------------------------------------------------ frontiers

    def _analyse(self, grid: OccupancyGrid):
        """Return (frontier candidates, free mask, index->world helper)."""
        w, h = grid.info.width, grid.info.height
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        data = np.asarray(grid.data, dtype=np.int8).reshape(h, w)

        unknown = (data < 0).astype(np.uint8)
        free = ((data >= 0) & (data <= 25)).astype(np.uint8)
        occupied = (data > 65).astype(np.uint8)

        # A frontier cell is free and 4-adjacent to unknown space.
        cross = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], np.uint8)
        touching_unknown = cv2.dilate(unknown, cross)
        frontier = ((free > 0) & (touching_unknown > 0)).astype(np.uint8)

        # Drop frontier cells the robot body could never occupy.
        pad = max(1, int(round(self.clearance / max(res, 1e-3))))
        disk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * pad + 1, 2 * pad + 1))
        frontier[cv2.dilate(occupied, disk) > 0] = 0

        n, labels, stats, centroids = cv2.connectedComponentsWithStats(frontier, 8)
        candidates = []
        for i in range(1, n):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < self.min_cells:
                continue
            ys, xs = np.nonzero(labels == i)
            cx, cy = centroids[i]
            # The centroid of a curved frontier can land off the frontier itself;
            # snap to the member cell nearest to it.
            k = int(np.argmin((xs - cx) ** 2 + (ys - cy) ** 2))
            wx = ox + (xs[k] + 0.5) * res
            wy = oy + (ys[k] + 0.5) * res
            candidates.append({'xy': (wx, wy), 'area': area * res * res})
        return candidates, free, (ox, oy, res)

    def _blacklisted(self, xy):
        return any((xy[0] - bx) ** 2 + (xy[1] - by) ** 2 < self.blacklist_radius ** 2
                   for bx, by in self.blacklist)

    def _pick_frontier(self, candidates, robot):
        best, best_score = None, -math.inf
        for c in candidates:
            if self._blacklisted(c['xy']):
                continue
            d = math.hypot(c['xy'][0] - robot[0], c['xy'][1] - robot[1])
            if d < self.min_goal_distance:
                continue
            score = self.gain_w * c['area'] - self.dist_w * d
            if score > best_score:
                best, best_score = c, score
        return best

    def _pick_patrol_goal(self, grid, free, origin, robot):
        """Random reachable free cell, biased away from where the robot already is."""
        ox, oy, res = origin
        pad = max(1, int(round(self.clearance / max(res, 1e-3))))
        disk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * pad + 1, 2 * pad + 1))
        data = np.asarray(grid.data, dtype=np.int8).reshape(grid.info.height, grid.info.width)
        safe = free.copy()
        safe[cv2.dilate((data > 65).astype(np.uint8), disk) > 0] = 0
        ys, xs = np.nonzero(safe)
        if xs.size == 0:
            return None
        for _ in range(40):
            k = self.rng.randrange(xs.size)
            wx = ox + (xs[k] + 0.5) * res
            wy = oy + (ys[k] + 0.5) * res
            if self._blacklisted((wx, wy)):
                continue
            if math.hypot(wx - robot[0], wy - robot[1]) > 2.0:
                return {'xy': (wx, wy), 'area': 0.0}
        return None

    # --------------------------------------------------------------- goals

    def _send_goal(self, xy, robot):
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(xy[0])
        pose.pose.position.y = float(xy[1])
        yaw = math.atan2(xy[1] - robot[1], xy[0] - robot[0])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = pose

        self.goal_xy = xy
        self.goal_sent_at = self.get_clock().now()
        self.nav.send_goal_async(goal).add_done_callback(self._on_goal_response)
        self.get_logger().info(f'goal -> ({xy[0]:+.2f}, {xy[1]:+.2f})')

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal; blacklisting it')
            self._finish_goal(success=False)
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        status = future.result().status
        ok = status == GoalStatus.STATUS_SUCCEEDED
        if ok:
            self.goals_reached += 1
            self.get_logger().info(f'goal reached ({self.goals_reached} total)')
        else:
            self.get_logger().warn(f'goal failed (status {status}); blacklisting it')
        self._finish_goal(success=ok)

    def _finish_goal(self, success):
        if not success and self.goal_xy is not None:
            self.blacklist.append(self.goal_xy)
            if len(self.blacklist) > 200:
                self.blacklist.pop(0)
        self.goal_handle = None
        self.goal_xy = None
        self.goal_sent_at = None

    def _goal_active(self):
        if self.goal_xy is None:
            return False
        if self.goal_sent_at is not None:
            elapsed = (self.get_clock().now() - self.goal_sent_at).nanoseconds * 1e-9
            if elapsed > self.goal_timeout:
                self.get_logger().warn(f'goal timed out after {elapsed:.0f} s; cancelling')
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                self._finish_goal(success=False)
                return False
        return True

    # ---------------------------------------------------------------- loop

    def _tick(self):
        if self.grid is None:
            self.get_logger().info('waiting for the occupancy grid ...', once=True)
            return
        if not self.nav.server_is_ready():
            self.get_logger().info('waiting for the Nav2 action server ...', throttle_duration_sec=5.0)
            return
        robot = self._robot_xy()
        if robot is None:
            self.get_logger().info(
                f'waiting for TF {self.map_frame} -> {self.base_frame} ...',
                throttle_duration_sec=5.0)
            return
        if self._goal_active():
            return

        grid = self.grid
        candidates, free, origin = self._analyse(grid)
        self._publish_markers(candidates)

        if self.exploring:
            target = self._pick_frontier(candidates, robot)
            if target is not None:
                self.dry_rounds = 0
                self._send_goal(target['xy'], robot)
                return
            self.dry_rounds += 1
            self.get_logger().info(
                f'no reachable frontier ({self.dry_rounds}/{self.dry_limit})')
            if self.dry_rounds < self.dry_limit:
                return
            self.exploring = False
            self.get_logger().info(
                'EXPLORATION COMPLETE -- the map is closed. '
                'The database is written to the configured database_path on shutdown.')
            if not self.patrol:
                return

        target = self._pick_patrol_goal(grid, free, origin, robot)
        if target is None:
            self.get_logger().info('no patrol goal available', throttle_duration_sec=10.0)
            return
        self._send_goal(target['xy'], robot)

    def _publish_markers(self, candidates):
        if self.markers is None:
            return
        array = MarkerArray()
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'frontiers'
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.a = 0.9
        m.color.g = 1.0
        m.pose.orientation.w = 1.0
        for c in candidates:
            p = Point()
            p.x, p.y, p.z = float(c['xy'][0]), float(c['xy'][1]), 0.1
            m.points.append(p)
        array.markers.append(m)
        self.markers.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
