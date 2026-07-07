#!/usr/bin/env python3
"""Per-truck LiDAR perception — TTSK12 paddock, Stage 3 (retuned + colored cloud).
Params: truck_id (1/2/3), lidar_height (sensor height above ground, m).
Sub:  /truck_N/lidar/points
Pub:  /truck_N/perception/objects, /truck_N/perception/verdict,
      /truck_N/classified/points (colored by class for RViz)."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, String
import numpy as np
import struct

GROUND_H = 0.30
ROCK_H   = 0.60
PILE_H   = 2.25
CELL     = 1.5
MIN_PTS  = 5
MOUND_CORR = 0.05
LANE_HALF = 3.0
LOOK = 35.0
FRONT_OFF = 0.5
SLOPE_BOX = 1.6
LANE_MIN = 4

NAME = {1:'ROCK', 2:'PILE', 3:'DUMP', 4:'OBSTACLE'}
RGB  = {0:(0.35,0.35,0.35), 1:(0.0,0.8,0.0), 2:(0.92,0.84,0.0),
        3:(0.86,0.12,0.12), 4:(0.6,0.0,0.55)}


class LidarPerception(Node):
    def __init__(self):
        super().__init__('lidar_perception')
        self.declare_parameter('truck_id', 1)
        self.declare_parameter('lidar_height', 6.5)
        self.tid = self.get_parameter('truck_id').value
        self.lh = float(self.get_parameter('lidar_height').value)
        ns = f'/truck_{self.tid}'
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(PointCloud2, f'{ns}/lidar/points', self.cb, qos)
        self.obj_pub = self.create_publisher(Float32MultiArray, f'{ns}/perception/objects', 10)
        self.verdict_pub = self.create_publisher(String, f'{ns}/perception/verdict', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, f'{ns}/classified/points', qos)
        self.frame_id = None
        self.frame = 0
        self.get_logger().info(f'Perception truck_{self.tid}: lidar_height={self.lh}m on {ns}/lidar/points')

    def cb(self, msg):
        self.frame_id = msg.header.frame_id
        off = {f.name: f.offset for f in msg.fields}
        if not {'x', 'y', 'z'} <= set(off):
            return
        dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4']*3,
                       'offsets': [off['x'], off['y'], off['z']], 'itemsize': msg.point_step})
        a = np.frombuffer(bytes(msg.data), dtype=dt)
        p = np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64)
        p = p[np.isfinite(p).all(1)]
        if p.shape[0] == 0:
            return

        # self-truck filter — exclude everything within the truck's own footprint,
        # regardless of height (cab/hood sits near sensor level and must be removed).
        # Sensor is at front; body extends backward ~13m, half-width ~5.5m, cab ahead ~3m.
        truck = (p[:, 0] > -15.0) & (p[:, 0] < 4.0) & (np.abs(p[:, 1]) < 6.0)
        p = p[~truck]
        if p.shape[0] == 0:
            return

        # world height = point z + sensor height above ground
        h = p[:, 2] + self.lh
        verdict, gap = self.decide_ahead(p, h)
        self.verdict_pub.publish(String(data=f'{verdict} {gap:.2f}'))

        cls = np.zeros(p.shape[0], dtype=np.int64)
        ridx = np.where(h >= GROUND_H)[0]
        objects = []
        for grp in self.cluster(p[ridx, 0], p[ridx, 1]):
            if len(grp) < MIN_PTS:
                continue
            gi = ridx[grp]
            px, py, ph = p[gi, 0], p[gi, 1], h[gi]
            top = float(ph.max())
            cxc = float(px.mean()); cyc = float(py.mean())
            rr = np.hypot(px - cxc, py - cyc)
            rad = float(np.percentile(rr, 90) + 0.2)
            if top <= ROCK_H:
                c = 1
            else:
                corr = float(np.corrcoef(rr, ph)[0, 1]) if rr.std() > 1e-3 and ph.std() > 1e-3 else 0.0
                # cone (radius grows as height drops -> negative corr) = pile; box/vertical = obstacle
                if corr < -MOUND_CORR:
                    c = 2 if top <= PILE_H else 3
                else:
                    c = 4
            cls[gi] = c
            objects.append((c, cxc, cyc, top, rad))

        self.publish_objects(objects)
        self.publish_colored(p[ridx], cls[ridx], msg.header)
        self.frame += 1
        if self.frame % 10 == 0:
            counts = {}
            for c, *_ in objects:
                counts[NAME[c]] = counts.get(NAME[c], 0) + 1
            self.get_logger().info(f'truck_{self.tid} AHEAD={verdict} gap={gap:.1f}m objs={counts}')

    def decide_ahead(self, p, h):
        x, y = p[:, 0], p[:, 1]
        lane = (x > 1.0) & (x < LOOK) & (np.abs(y) < LANE_HALF) & (h > GROUND_H)
        idx = np.where(lane)[0]
        if idx.size < LANE_MIN:
            return 'CLEAR', 99.0
        lx, ly, lh = x[idx], y[idx], h[idx]
        near = float(lx.min())
        sel = lx < near + 4.0
        cx, cy, ch = lx[sel], ly[sel], lh[sel]
        H = float(ch.max())
        gap = near - FRONT_OFF
        if H <= ROCK_H:
            return 'ROCK', gap
        rng = np.hypot(cx, cy)
        r0 = float(rng.min())
        rpk = float(rng[ch >= 0.9*H].min())
        slope = (0.9*H) / max(rpk - r0, 0.3)
        if slope > SLOPE_BOX:
            return 'OBSTACLE', gap
        if H <= PILE_H:
            return 'PILE', gap
        return 'DUMP', gap

    def cluster(self, x, y):
        n = x.size
        if n == 0:
            return []
        cx = np.floor(x / CELL).astype(int)
        cy = np.floor(y / CELL).astype(int)
        cells = {}
        for i in range(n):
            cells.setdefault((cx[i], cy[i]), []).append(i)
        keys = list(cells.keys())
        kid = {k: i for i, k in enumerate(keys)}
        par = list(range(len(keys)))
        def find(a):
            while par[a] != a:
                par[a] = par[par[a]]; a = par[a]
            return a
        for (gx, gy), i in kid.items():
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    j = kid.get((gx+dx, gy+dy))
                    if j is not None:
                        ra, rb = find(i), find(j)
                        if ra != rb: par[ra] = rb
        groups = {}
        for k, i in kid.items():
            groups.setdefault(find(i), []).extend(cells[k])
        return list(groups.values())

    def publish_objects(self, objects):
        m = Float32MultiArray()
        d = [float(len(objects))]
        for c, x, y, top, rad in objects:
            d += [float(c), x, y, top, rad]
        m.data = d
        self.obj_pub.publish(m)

    def publish_colored(self, pts, cls, header):
        if pts.shape[0] == 0:
            return
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        buf = bytearray()
        for i in range(pts.shape[0]):
            r, g, b = RGB[int(cls[i])]
            rgb = struct.unpack('f', struct.pack('I',
                (int(r*255) << 16) | (int(g*255) << 8) | int(b*255)))[0]
            buf += struct.pack('ffff', pts[i, 0], pts[i, 1], pts[i, 2], rgb)
        out = PointCloud2()
        out.header = header
        out.height = 1
        out.width = pts.shape[0]
        out.fields = fields
        out.is_bigendian = False
        out.point_step = 16
        out.row_step = 16 * pts.shape[0]
        out.is_dense = True
        out.data = bytes(buf)
        self.cloud_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    n = LidarPerception()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
