#!/usr/bin/env python3
"""OpenCV dashboard for an RTAB-Map RGB-D SLAM run.

Runs as an external node alongside rtabmap and draws a single 2x2 mosaic:

    +---------------------+---------------------+
    | RGB + SLAM status   | Depth (colormapped) |
    +---------------------+---------------------+
    | 2D occupancy grid   | 3D cloud (orbit)    |
    +---------------------+---------------------+

Keys (focus the window first):
    q / ESC   quit                a / d   orbit yaw
    w / s     orbit pitch         + / -   zoom
    space     toggle auto-orbit   r       reset the 3D view
    c         cycle depth colormap
    t         toggle cloud colouring (RGB <-> height)
    f         freeze / unfreeze the 3D cloud (stop accepting new clouds)

The dashboard is read-only: it never publishes anything, so it can be started
and stopped at any point during a mapping run.
"""

import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, NavSatFix
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener

try:
    from rtabmap_msgs.msg import Info as RtabmapInfo
except ImportError:  # rtabmap_msgs not sourced -- degrade gracefully
    RtabmapInfo = None


PANEL_W = 480
PANEL_H = 360
BAR_H = 46

COLORMAPS = [
    ('JET', cv2.COLORMAP_JET),
    ('TURBO', cv2.COLORMAP_TURBO),
    ('INFERNO', cv2.COLORMAP_INFERNO),
    ('BONE', cv2.COLORMAP_BONE),
]

WHITE = (255, 255, 255)
GREY = (150, 150, 150)
GREEN = (0, 220, 0)
AMBER = (0, 190, 255)
RED = (0, 60, 255)


def _put(img, text, org, colour=WHITE, scale=0.45, thick=1):
    """Draw text with a dark outline so it stays readable on any background."""
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, colour, thick, cv2.LINE_AA)


def _placeholder(text):
    img = np.full((PANEL_H, PANEL_W, 3), 32, np.uint8)
    _put(img, text, (14, PANEL_H // 2), GREY, 0.5)
    return img


def _fit(img):
    """Letterbox an image into a PANEL_W x PANEL_H panel."""
    h, w = img.shape[:2]
    s = min(PANEL_W / w, PANEL_H / h)
    out = np.full((PANEL_H, PANEL_W, 3), 32, np.uint8)
    rw, rh = max(1, int(w * s)), max(1, int(h * s))
    resized = cv2.resize(img, (rw, rh), interpolation=cv2.INTER_AREA)
    x, y = (PANEL_W - rw) // 2, (PANEL_H - rh) // 2
    out[y:y + rh, x:x + rw] = resized
    return out


class RateTracker:
    """Sliding-window message rate estimate (wall clock, so it works while paused)."""

    def __init__(self, window=20):
        self._stamps = []
        self._window = window

    def tick(self):
        self._stamps.append(time.monotonic())
        if len(self._stamps) > self._window:
            self._stamps.pop(0)

    def hz(self):
        if len(self._stamps) < 2:
            return 0.0
        span = self._stamps[-1] - self._stamps[0]
        if span <= 0.0 or time.monotonic() - self._stamps[-1] > 3.0:
            return 0.0
        return (len(self._stamps) - 1) / span


class RtabmapDashboard(Node):

    def __init__(self):
        super().__init__('rtabmap_dashboard')

        ns = self.declare_parameter('robot_namespace', 'a200_0000').value
        self.declare_parameter('rgb_topic', f'/{ns}/sensors/camera_0/color/image')
        self.declare_parameter('depth_topic', f'/{ns}/sensors/camera_0/depth/image')
        self.declare_parameter('map_topic', f'/{ns}/map')
        self.declare_parameter('cloud_topic', f'/{ns}/cloud_map')
        self.declare_parameter('odom_topic', f'/{ns}/odom')
        self.declare_parameter('info_topic', f'/{ns}/info')
        self.declare_parameter('gps_topic', f'/{ns}/sensors/gps_0/fix')
        # Frame *names* are bare on this robot -- only topics are namespaced,
        # because robot_state_publisher is launched without a frame_prefix.
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        # Depth range used for the colour ramp; also the "valid" window for stats.
        self.declare_parameter('depth_min', 0.3)
        self.declare_parameter('depth_max', 8.0)
        self.declare_parameter('max_cloud_points', 250000)
        self.declare_parameter('refresh_rate', 15.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.depth_min = float(self.get_parameter('depth_min').value)
        self.depth_max = float(self.get_parameter('depth_max').value)
        self.max_cloud_points = int(self.get_parameter('max_cloud_points').value)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- latest data -------------------------------------------------
        self.rgb = None
        self.rgb_frame = ''
        self.depth = None
        self.depth_frame = ''
        self.depth_encoding = ''
        self.grid = None
        self.cloud_xyz = None
        self.cloud_bgr = None
        self.odom = None
        self.info = None
        self.gps = None
        self.trail = []

        self.rate_rgb = RateTracker()
        self.rate_depth = RateTracker()
        self.rate_cloud = RateTracker()
        self.rate_map = RateTracker()
        self.rate_gps = RateTracker()

        # --- 3D view state -----------------------------------------------
        self.yaw = math.radians(-60.0)
        self.pitch = math.radians(35.0)
        self.zoom = 1.0
        self.auto_orbit = True
        self.cloud_frozen = False
        self.cmap_idx = 0
        self.cloud_colour_rgb = True

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # rtabmap latches the grid and the assembled cloud.
        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            Image, self.get_parameter('rgb_topic').value, self._on_rgb, sensor_qos)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value, self._on_depth, sensor_qos)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value, self._on_map, latched_qos)
        self.create_subscription(
            PointCloud2, self.get_parameter('cloud_topic').value, self._on_cloud, latched_qos)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom, sensor_qos)
        self.create_subscription(
            NavSatFix, self.get_parameter('gps_topic').value, self._on_gps, sensor_qos)
        if RtabmapInfo is not None:
            self.create_subscription(
                RtabmapInfo, self.get_parameter('info_topic').value, self._on_info, sensor_qos)

        self.window = 'RTAB-Map dashboard'
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window, PANEL_W * 3, PANEL_H * 2 + BAR_H)

        period = 1.0 / max(1.0, float(self.get_parameter('refresh_rate').value))
        # Wall-clock timer: the dashboard keeps redrawing even if the sim is paused.
        self.timer = self.create_timer(period, self._draw, clock=rclpy.clock.Clock())
        self.get_logger().info('dashboard up -- focus the window and press "q" to quit')

    # ---------------------------------------------------------------- subs

    def _on_rgb(self, msg: Image):
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001 - never let a bad frame kill the node
            self.get_logger().warn(f'rgb convert failed: {exc}', throttle_duration_sec=5.0)
            return
        self.rgb_frame = msg.header.frame_id
        self.rate_rgb.tick()

    def _on_depth(self, msg: Image):
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'depth convert failed: {exc}', throttle_duration_sec=5.0)
            return
        if raw.dtype == np.uint16:  # 16UC1 is millimetres
            depth = raw.astype(np.float32) / 1000.0
        else:
            depth = raw.astype(np.float32)
        self.depth = depth
        self.depth_frame = msg.header.frame_id
        self.depth_encoding = msg.encoding
        self.rate_depth.tick()

    def _on_map(self, msg: OccupancyGrid):
        self.grid = msg
        self.rate_map.tick()

    def _on_cloud(self, msg: PointCloud2):
        if self.cloud_frozen:
            return
        names = {f.name for f in msg.fields}
        if not {'x', 'y', 'z'}.issubset(names):
            return
        try:
            xyz = pc2.read_points_numpy(
                msg, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float32)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'cloud parse failed: {exc}', throttle_duration_sec=5.0)
            return
        if xyz.size == 0:
            return
        xyz = xyz.reshape(-1, 3)

        bgr = None
        if 'rgb' in names:
            try:
                packed = pc2.read_points_numpy(
                    msg, field_names=('rgb',), skip_nans=True).reshape(-1)
                packed = np.ascontiguousarray(packed, dtype=np.float32).view(np.uint32)
                bgr = np.stack([packed & 0xFF, (packed >> 8) & 0xFF, (packed >> 16) & 0xFF],
                               axis=-1).astype(np.uint8)
                if bgr.shape[0] != xyz.shape[0]:
                    bgr = None
            except Exception:  # noqa: BLE001 - colour is optional
                bgr = None

        # Uniform subsample keeps the render cheap on large maps.
        if xyz.shape[0] > self.max_cloud_points:
            step = int(math.ceil(xyz.shape[0] / self.max_cloud_points))
            xyz = xyz[::step]
            if bgr is not None:
                bgr = bgr[::step]

        self.cloud_xyz = xyz
        self.cloud_bgr = bgr
        self.rate_cloud.tick()

    def _on_odom(self, msg: Odometry):
        self.odom = msg

    def _on_info(self, msg):
        self.info = msg

    def _on_gps(self, msg: NavSatFix):
        self.gps = msg
        self.rate_gps.tick()

    # ------------------------------------------------------------- helpers

    def _robot_pose_in_map(self):
        """(x, y, yaw) of base_frame in map_frame, or None if TF is not ready."""
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001 - TF gaps are normal at startup
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    # -------------------------------------------------------------- panels

    def _panel_rgb(self):
        if self.rgb is None:
            return _placeholder('waiting for RGB ...')
        panel = _fit(self.rgb)
        cv2.rectangle(panel, (0, 0), (PANEL_W, 20), (0, 0, 0), -1)
        _put(panel, f'RGB  {self.rate_rgb.hz():4.1f} Hz  frame={self.rgb_frame or "?"}',
             (6, 14), GREEN, 0.4)
        return panel

    def _panel_depth(self):
        if self.depth is None:
            return _placeholder('waiting for depth ...')
        d = self.depth
        valid = np.isfinite(d) & (d > self.depth_min) & (d < self.depth_max)
        norm = np.zeros(d.shape, np.uint8)
        if valid.any():
            span = max(1e-3, self.depth_max - self.depth_min)
            scaled = (d - self.depth_min) / span
            norm[valid] = np.clip(scaled[valid] * 255.0, 0, 255).astype(np.uint8)
        coloured = cv2.applyColorMap(norm, COLORMAPS[self.cmap_idx][1])
        coloured[~valid] = (40, 40, 40)  # no return -> flat grey
        panel = _fit(coloured)

        h, w = d.shape[:2]
        centre = d[h // 2, w // 2]
        centre_txt = f'{centre:.2f} m' if np.isfinite(centre) and centre > 0 else 'n/a'
        # Nearest obstacle inside the central band -- the number that matters for nav.
        band = d[int(h * 0.35):int(h * 0.65), int(w * 0.30):int(w * 0.70)]
        band = band[np.isfinite(band) & (band > self.depth_min)]
        nearest = f'{band.min():.2f} m' if band.size else 'n/a'
        cover = 100.0 * valid.sum() / valid.size

        cx, cy = PANEL_W // 2, PANEL_H // 2
        cv2.drawMarker(panel, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)
        cv2.rectangle(panel, (0, 0), (PANEL_W, 20), (0, 0, 0), -1)
        cv2.rectangle(panel, (0, PANEL_H - 38), (PANEL_W, PANEL_H), (0, 0, 0), -1)
        _put(panel, f'DEPTH  {self.rate_depth.hz():4.1f} Hz  {self.depth_encoding}  '
                    f'[{COLORMAPS[self.cmap_idx][0]}]', (6, 14), AMBER, 0.4)
        _put(panel, f'centre {centre_txt}   nearest-ahead {nearest}',
             (6, PANEL_H - 22), WHITE, 0.44)
        _put(panel, f'valid {cover:4.1f}%   ramp {self.depth_min:.1f}-{self.depth_max:.1f} m',
             (6, PANEL_H - 6), GREY, 0.4)
        return panel

    def _panel_grid(self):
        if self.grid is None:
            return _placeholder('waiting for /map ...')
        g = self.grid
        w, h = g.info.width, g.info.height
        if w == 0 or h == 0:
            return _placeholder('/map is empty')
        data = np.asarray(g.data, dtype=np.int8).reshape(h, w)

        img = np.full((h, w, 3), 128, np.uint8)          # unknown
        img[data == 0] = (235, 235, 235)                 # free
        img[data > 50] = (25, 25, 25)                    # occupied
        img = cv2.flip(img, 0)                           # grid origin is bottom-left

        res = g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y

        def to_px(x, y):
            return int((x - ox) / res), int(h - 1 - (y - oy) / res)

        pose = self._robot_pose_in_map()
        if pose is not None:
            x, y, yaw = pose
            if not self.trail or (x - self.trail[-1][0]) ** 2 + (y - self.trail[-1][1]) ** 2 > 0.04:
                self.trail.append((x, y))
                if len(self.trail) > 4000:
                    self.trail.pop(0)
            pts = np.array([to_px(px, py) for px, py in self.trail], np.int32)
            if len(pts) > 1:
                cv2.polylines(img, [pts.reshape(-1, 1, 2)], False, (255, 120, 0), 1, cv2.LINE_AA)
            px, py = to_px(x, y)
            nx, ny = to_px(x + 0.6 * math.cos(yaw), y + 0.6 * math.sin(yaw))
            cv2.circle(img, (px, py), max(2, int(0.18 / res)), (0, 0, 255), -1, cv2.LINE_AA)
            cv2.arrowedLine(img, (px, py), (nx, ny), (0, 0, 255), 2, cv2.LINE_AA, tipLength=0.4)

        panel = _fit(img)
        known = int(np.count_nonzero(data >= 0))
        area = known * res * res
        cv2.rectangle(panel, (0, 0), (PANEL_W, 20), (0, 0, 0), -1)
        cv2.rectangle(panel, (0, PANEL_H - 20), (PANEL_W, PANEL_H), (0, 0, 0), -1)
        _put(panel, f'2D GRID  {w}x{h} @ {res:.2f} m  {self.rate_map.hz():4.1f} Hz',
             (6, 14), GREEN, 0.4)
        _put(panel, f'mapped {area:7.1f} m^2   frame={g.header.frame_id or "?"}',
             (6, PANEL_H - 6), GREY, 0.4)
        return panel

    def _panel_cloud(self):
        if self.cloud_xyz is None or self.cloud_xyz.shape[0] == 0:
            return _placeholder('waiting for /cloud_map ... (drive to build the 3D map)')
        pts = self.cloud_xyz
        centre = pts.mean(axis=0)
        rel = pts - centre
        extent = float(np.percentile(np.linalg.norm(rel[:, :2], axis=1), 95)) or 1.0

        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        # yaw about world Z, then pitch about the camera's right axis.
        rot = np.array([
            [cy, sy, 0.0],
            [-sy * cp, cy * cp, sp],
            [sy * sp, -cy * sp, cp],
        ], np.float32)
        cam = rel @ rot.T                       # x=right, y=forward(depth), z=up

        dist = extent * 2.6 / max(0.05, self.zoom)
        depth = cam[:, 1] + dist
        ok = depth > 0.05
        if not ok.any():
            return _placeholder('3D view: nothing in front of the camera (press "r")')
        cam, depth = cam[ok], depth[ok]

        f = 0.75 * PANEL_W
        u = (PANEL_W * 0.5 + f * cam[:, 0] / depth).astype(np.int32)
        v = (PANEL_H * 0.5 - f * cam[:, 2] / depth).astype(np.int32)
        inside = (u >= 0) & (u < PANEL_W) & (v >= 0) & (v < PANEL_H)

        if self.cloud_colour_rgb and self.cloud_bgr is not None:
            colours = self.cloud_bgr[ok][inside]
        else:
            z = pts[ok][inside][:, 2]
            lo, hi = np.percentile(z, 2), np.percentile(z, 98)
            ramp = np.clip((z - lo) / max(1e-3, hi - lo) * 255.0, 0, 255).astype(np.uint8)
            colours = cv2.applyColorMap(ramp.reshape(-1, 1), cv2.COLORMAP_TURBO).reshape(-1, 3)

        panel = np.full((PANEL_H, PANEL_W, 3), 18, np.uint8)
        # Painter's algorithm: far points first so near points win the pixel.
        order = np.argsort(-depth[inside])
        panel[v[inside][order], u[inside][order]] = colours[order]

        mode = 'RGB' if (self.cloud_colour_rgb and self.cloud_bgr is not None) else 'height'
        cv2.rectangle(panel, (0, 0), (PANEL_W, 20), (0, 0, 0), -1)
        cv2.rectangle(panel, (0, PANEL_H - 20), (PANEL_W, PANEL_H), (0, 0, 0), -1)
        _put(panel, f'3D CLOUD  {pts.shape[0]/1000:.0f}k pts  {self.rate_cloud.hz():4.1f} Hz  '
                    f'[{mode}]', (6, 14), AMBER, 0.4)
        _put(panel, 'a/d yaw  w/s pitch  +/- zoom  space orbit  r reset'
                    + ('  [FROZEN]' if self.cloud_frozen else ''),
             (6, PANEL_H - 6), GREY, 0.38)
        return panel

    def _panel_gps(self):
        if self.gps is None:
            return _placeholder('waiting for GPS ...')
        
        panel = np.full((PANEL_H, PANEL_W, 3), 32, np.uint8)
        cv2.rectangle(panel, (0, 0), (PANEL_W, 20), (0, 0, 0), -1)
        _put(panel, f'GPS FIX  {self.rate_gps.hz():4.1f} Hz', (6, 14), GREEN, 0.4)
        
        status = 'Fix' if self.gps.status.status >= 0 else 'No Fix'
        colour = GREEN if self.gps.status.status >= 0 else RED
        
        _put(panel, f'Status: {status}', (20, 80), colour, 0.8)
        _put(panel, f'Lat: {self.gps.latitude:.6f}', (20, 140), WHITE, 0.7)
        _put(panel, f'Lon: {self.gps.longitude:.6f}', (20, 180), WHITE, 0.7)
        _put(panel, f'Alt: {self.gps.altitude:.2f} m', (20, 220), WHITE, 0.7)
        
        return panel

    def _status_bar(self):
        bar = np.full((BAR_H, PANEL_W * 3, 3), 24, np.uint8)

        loc = 'no map->base TF yet'
        colour = RED
        pose = self._robot_pose_in_map()
        if pose is not None:
            loc = f'pose  x={pose[0]:+6.2f}  y={pose[1]:+6.2f}  yaw={math.degrees(pose[2]):+6.1f} deg'
            colour = GREEN
        _put(bar, loc, (8, 18), colour, 0.46)

        if self.info is not None:
            stats = dict(zip(self.info.stats_keys, self.info.stats_values))
            loops = int(stats.get('Loop/Accepted_hypothesis_id/', 0)) or self.info.loop_closure_id
            slam = (f'nodes(WM) {len(self.info.wm_state):4d}   '
                    f'ref {self.info.ref_id:5d}   '
                    f'loop {loops:5d}   '
                    f'prox {self.info.proximity_detection_id:5d}')
            if 'Timing/Total/ms' in stats:
                slam += f'   cycle {stats["Timing/Total/ms"]:6.1f} ms'
            _put(bar, slam, (8, 38), WHITE, 0.44)
        elif RtabmapInfo is None:
            _put(bar, 'rtabmap_msgs not available -- SLAM stats hidden', (8, 38), GREY, 0.44)
        else:
            _put(bar, 'waiting for rtabmap /info ...', (8, 38), GREY, 0.44)

        if self.odom is not None:
            tw = self.odom.twist.twist
            _put(bar, f'v={tw.linear.x:+5.2f} m/s   w={tw.angular.z:+5.2f} rad/s',
                 (PANEL_W * 3 - 300, 18), WHITE, 0.46)
        _put(bar, f'depth frame: {self.depth_frame or "?"}',
             (PANEL_W * 3 - 300, 38), GREY, 0.42)
        return bar

    # ---------------------------------------------------------------- draw

    def _draw(self):
        if self.auto_orbit:
            self.yaw += math.radians(0.35)

        top = np.hstack([self._panel_rgb(), self._panel_depth(), self._panel_gps()])
        bottom = np.hstack([self._panel_grid(), self._panel_cloud(), _placeholder('Empty Slot')])
        canvas = np.vstack([top, bottom, self._status_bar()])
        cv2.imshow(self.window, canvas)
        self._handle_key(cv2.waitKey(1) & 0xFF)

    def _handle_key(self, key):
        if key in (ord('q'), 27):
            self.get_logger().info('quit requested from the dashboard window')
            raise KeyboardInterrupt
        elif key == ord('a'):
            self.yaw -= math.radians(5)
        elif key == ord('d'):
            self.yaw += math.radians(5)
        elif key == ord('w'):
            self.pitch = min(math.radians(89), self.pitch + math.radians(4))
        elif key == ord('s'):
            self.pitch = max(math.radians(-89), self.pitch - math.radians(4))
        elif key in (ord('+'), ord('=')):
            self.zoom = min(12.0, self.zoom * 1.15)
        elif key in (ord('-'), ord('_')):
            self.zoom = max(0.1, self.zoom / 1.15)
        elif key == ord(' '):
            self.auto_orbit = not self.auto_orbit
        elif key == ord('r'):
            self.yaw, self.pitch, self.zoom = math.radians(-60), math.radians(35), 1.0
        elif key == ord('c'):
            self.cmap_idx = (self.cmap_idx + 1) % len(COLORMAPS)
        elif key == ord('t'):
            self.cloud_colour_rgb = not self.cloud_colour_rgb
        elif key == ord('f'):
            self.cloud_frozen = not self.cloud_frozen


def main(args=None):
    rclpy.init(args=args)
    node = RtabmapDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
