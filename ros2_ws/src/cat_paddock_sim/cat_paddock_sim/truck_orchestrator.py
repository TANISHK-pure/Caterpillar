#!/usr/bin/env python3
"""truck_orchestrator.py — Cat Tech Challenge 2026 · Team TTSK12
Bridges Gazebo trucks to the MATLAB NFP+MILP brain.
"""
import math
import os
import subprocess
import tempfile
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseArray, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Int32

PADDOCK_GATE   = (0.0, 0.0)
PAD_ENTRY      = (50.0, 60.0)

# (truck_id, class_id, pad_index)
FLEET = [(1, 1, 1), (2, 2, 2), (3, 3, 3)]

# Per-truck hardcoded corridor route (world frame). The corridor between
# the gate and dump pad is now obstacle-free (gen_paddock_world.py), so
# these routes don't need to dodge anything -- they just fan out into 3
# visibly distinct lanes for realism, all funneling through the single
# pad-gate gap (x in [35,65] @ y=115) since that's the only way into the
# dump pad, then re-diverging to each truck's own dump target:
#  - T1: west lane through the corridor, west dump target.
#  - T2: center lane, center dump target.
#  - T3: east lane, east dump target.
# Geometrically verified (half-width clearance on straight legs,
# half-diagonal clearance at turn waypoints with >=60 deg heading change):
# all 3 routes clear the corridor walls and dump-pad berms by >=14m
# everywhere, with >=10m inter-route separation except for two brief
# tapers at the two intentionally-shared convergence points (the south
# gate, and the pad-gate crossing at x=44) -- both protected by timing
# (gate_holder mutex / staggered entry_grants), same as the existing
# south-gate design.
# (t.target_x, t.target_y) is appended per-truck once the NFP plan arrives.
ROUTES = {
    1: [(-5, -50), (-5, 15), (0, 35), (-12, 60), (-5, 95), (44.0, 108), (44.0, 132), (46, 148), (68, 143)],
    2: [(-5, -50), (-5, 15), (0, 35), (5, 60),   (20, 95), (44.0, 108), (44.0, 132), (60, 145), (82, 141)],
    # T3 (truck_400t, 60.75t chassis, 10.4m wheel separation) gets a smooth,
    # gently-curving path instead of the sharp corners T1/T2 can handle --
    # confirmed via wheel joint_state telemetry that any turn requiring a
    # heading change beyond roughly 15-20 deg makes T3 (uniquely, not
    # T1/T2) hit a skid-steer contact-solver slip: wheels keep spinning at
    # exactly the commanded kinematic rate but the chassis stops responding
    # for tens of seconds. Neither more wheel friction nor more wheel
    # torque changed this (it's a contact-solver issue, not a grip/power
    # one), so the route itself keeps every heading change <=18 deg.
    #
    # This same-direction-gentle path used to track almost the same x as
    # T2's lane through the open corridor (both near x=5-24 for y=55-90),
    # leaving as little as 0.10-2.24m centerline clearance -- nowhere near
    # enough for these trucks (T2 12x7m + T3 15x9m have a combined half-
    # diagonal reach of ~15.7m), and confirmed by an actual T2<->T3
    # collision in testing. Re-solved (numeric search over <=18deg-per-
    # turn paths) to curve further east earlier, crossing the one
    # physical gap in the corridor's north wall (x in [35,65] @ y=115) at
    # x=41.9, well clear of either jamb.
    #
    # A second pass at this same redesign drove straight through
    # pad_berm_0 (the dump pad's own south perimeter wall, a SEPARATE
    # collision boundary from the corridor's north wall above, further
    # north at x:60-140,y:120-125) -- missed because the corridor wall
    # and the dump-pad berms had been treated as one boundary. The dump
    # pad polygon's only open (un-bermed) edges face southwest, roughly
    # (70,180)-(40,140)-(60,120); this path now stays >=13m clear of all
    # four berms and enters through that opening instead of through
    # berm_0, ending pointed generally east (heading ~19deg) into open
    # pad ground so the dynamic DUMP_STANDOFF leg to the real NFP target
    # isn't starting from a sharp reversal.
    3: [(-5, -50), (-5, 15), (0, 35), (12.0, 55.1), (21.1, 75.3), (26.7, 89.3),
        (38.0, 106.8), (46.1, 124.0), (60.3, 139.9), (78.3, 152.1),
        (98.3, 161.8), (118.8, 168.7)],
}

# NOTE: tried offsetting DRIVING_OUT's shared-segment points laterally
# (south gate approach, T1/T2's pad-gate crossing) to stop outbound
# traffic meeting inbound traffic nose-to-nose on the identical line.
# It did stop that exact collision, but the offset return leg cut
# diagonally across another truck's inbound lane in the open corridor,
# trading one collision for a different one -- reverted. The
# zone_holder/gate_holder zone mutexes (both now cover all directions)
# are the current line of defense for these shared segments.
#
# gate_holder was originally DRIVING_OUT-only -- DRIVING_IN relied on
# entry-grant stagger alone to keep trucks apart on the identical shared
# waypoints (-5,-50)->(-5,15)->(0,35). That's not enough: T3 (no speed
# cap, unlike T2) caught up to T2 there and collided. Extended to a
# single-token mutex covering BOTH directions, same claim-by-intent/
# release-on-both-clear pattern as zone_holder -- except an inbound
# truck must release incrementally as soon as it clears the zone (it
# still has ~100m of route left), unlike an outbound truck, which holds
# it all the way to BAY since the zone IS its final stretch.

# Real haul trucks stop short of a dump location and tip into it rather than
# driving the chassis onto the exact target point. With dump targets now
# repositioned well clear of any pad berm (see ROUTES design above), a
# realistic close-approach distance is reachable for all 3 truck sizes.
DUMP_STANDOFF = 3.5

# pad_zone/fan_zone only cover the narrow gate-crossing/fan-out area near
# the gate itself (x roughly 35-90) -- but a real NFP target can place a
# truck anywhere out on the open pad (confirmed: an actual run put one at
# (149,166), nowhere near either box). A truck dumping out there holds no
# lock at all, so a second truck passing nearby has nothing stopping it
# from driving straight into the stationary first one -- confirmed by an
# actual T1<->T2 collision while T1 was dumping, 313 consecutive overlap
# ticks. This is a live distance check against any truck CURRENTLY in
# DUMPING, not a claim/release token like zone_holder -- simpler, and it
# self-resolves the instant the dumping truck leaves DUMPING, with no
# stale-token risk. 18m covers the worst-case combined half-diagonal of
# any two trucks (T2+T3 ~15.7m) with margin for the brief reaction tick.
DUMP_SAFETY_RADIUS = 18.0

# Normal south-gate transit (BAY<->the merge point) takes ~20-30s based on
# observed runs. If the same truck has held gate_holder far longer than
# that without clearing, it's not just slow -- it's the same kind of
# physics freeze seen elsewhere in this sim -- so force-release rather
# than let one stuck truck permanently block this single-lane chokepoint
# for the other two trucks forever.
GATE_TIMEOUT = 60.0

# Without a release-on-stall safety valve here, a truck that hits the
# chronic wheel-spin/contact-solver freeze (see ROUTES[3] comment) while
# its TARGET has already advanced into the zone holds zone_holder forever:
# the release check requires the holder's target (not just its actual
# position) to also clear the zone, but a frozen truck's wp_idx never
# advances, so its target never moves. Confirmed by a real run: T2 froze
# at (17,90), well short of the zone, but reached()'s 6m tolerance was
# satisfied against that frozen position, advancing its target to
# (44,108) -- inside the zone. zone_holder stuck on T2 for the rest of the
# run, permanently blocking T1's return through the same gate (T1 stopped
# dead at (43.7,116.8), cmd_vel=0, waiting on a lock that could never
# clear) -- this was reported as "T1 halts in the mid-corridor area on its
# return".
#
# A flat elapsed-time timeout on the claim (the first attempt here, same
# shape as GATE_TIMEOUT) backfired: this zone's transit legitimately
# involves several sharp turns (each one a slow creep-and-turn, not a
# drive-through) plus a possible wait for the corridor direction-gate, so
# a truck that's making completely normal progress can still hold the
# token upwards of 60-80s with zero malfunction. The flat timeout fired
# mid-transit, force-releasing a perfectly healthy holder and letting a
# second truck claim the now-"free" zone while the first was still
# physically inside it -- confirmed by an actual T1<->T2 0.00m-clearance
# overlap, both trucks tracking through the same crossing in lockstep.
# Tracking the holder's own position instead (resetting the timer on any
# >1m movement) only fires for genuine immobility, which is what
# distinguishes a real freeze from merely-slow-but-progressing -- same
# "use actual position, not just intent" principle as gate_holder's
# release-on-clear check above.
ZONE_TIMEOUT = 60.0

# /milp_lock is a real-time emergency brake from MATLAB's MILP safety
# checker (two trucks within its critical_zone) -- it's meant to be brief
# (long enough for the trailing truck to stop and the lead truck to pull
# away), with MATLAB expected to send a matching release once clearance is
# restored. But there is a window where it never does (a missed message,
# a MATLAB-side crash/restart, or simply a bug) -- and on_lock's `if
# t.locked: stop(); continue` runs before any other tick() logic, so a
# truck stuck locked never reaches DONE and never stops needing the lock
# checked. Same "don't trust a remote release that might never come"
# reasoning as GATE_TIMEOUT/ZONE_TIMEOUT above -- 30s is far more than the
# safety brake should ever need to resolve on its own.
LOCK_TIMEOUT = 30.0

# Chassis half-length per truck (100t/250t/400t: 9x5, 12x7, 15x9 m) -- used
# to push the spawned dump pile clear of the truck's own collision box.
TRUCK_HALF_LEN = {1: 4.5, 2: 6.0, 3: 7.5}

# Each truck's own staging bay (matches the bay() markers in
# gen_paddock_world.py). DRIVING_OUT ends here rather than at the shared
# merge point (-5,-50) -- with all 3 routes reversing onto that single
# point, whichever truck finishes first parks there and blocks the lane,
# wedging the next truck that arrives (observed: T1 stuck nose-to-tail
# against a parked T2 for the rest of the run).
BAY = {1: (-30.0, -50.0), 2: (-30.0, -75.0), 3: (-30.0, -100.0)}

# Bounding box around ONLY the literal dump-pad gate crossing
# ((44,108)<->(44,132), the one line T1/T2 actually share). Originally
# sized to also cover the fan-out point just past the gate (y up to 150)
# to catch a return-trip collision there -- but joint_state telemetry
# showed that just made the lock too sticky: a truck holds it from its
# final approach through DUMPING (pad_holder isn't touched in that state)
# through several outbound legs, since T1's own fan point (46,148) and
# dump target (>=140) both still counted as "in the zone". That left T2
# commanded to a dead stop (verified: wheel velocity exactly 0.0, not
# slip) for ~65s waiting on a lock T1 didn't need anymore. Once a truck
# is past y=132 it's on its own distinct fan lane (verified clear of the
# other trucks' lanes), so it doesn't need the lock past that point --
# shrunk to just the gate crossing itself, with a small margin.
PAD_ZONE_X = (35.0, 65.0)
PAD_ZONE_Y = (100.0, 140.0)

# The open stretch between the south gate (y<40, gate_holder's turf) and
# the pad-gate crossing (pad_holder's turf): each truck has its own fan
# lane through here, so same-direction trucks can share it fine, but
# nothing serialized OPPOSING directions -- this is exactly where T1
# (DRIVING_OUT) met T3 (DRIVING_IN) head-on near (-11,60). Modeled as a
# direction-exclusive resource rather than a single-token lock like
# gate_holder/pad_holder: any number of trucks going the SAME direction
# may hold it at once (their fan lanes already keep them apart), but a
# truck needing to go the opposite way must wait for it to fully clear
# first -- a corridor traffic light, which is MILP's job in spirit even
# though it's enforced locally here.
CORRIDOR_Y = (35.0, 100.0)

# Past the pad-gate crossing, each truck's route fans out onto its own
# dump-target lane ((46,148)->(68,143) for T1, (60,145)->(82,141) for T2,
# (51,126)->(66,137)->(82,144) for T3) -- but those lanes are close enough
# together (verified: T1's and T2's tail segments pass within ~0.2m of
# each other near x=60) that an outbound truck retracing its lane in
# reverse can still meet an inbound truck head-on here, same failure mode
# as the open corridor just further out. Confirmed by collision: T1
# (DRIVING_OUT) x T2 (DRIVING_IN) at (60.2,153.6)/(53.2,143.8).
#
# Tried the same direction-exclusive pattern as CORRIDOR_Y/corridor_trucks
# first -- it made things worse. That model only stops a NEW entrant from
# joining the wrong direction; it does nothing to protect a truck already
# moving from a truck that's STOPPED (dumping) inside the zone, and this
# zone, unlike the corridor, is also where trucks park to dump. Result:
# T1 correctly waited at its dump spot, but T2 (never blocked, since a
# blocked truck never registers as "holding" a direction) drove straight
# into T1's stationary chassis and stayed wedged there for the rest of
# the run (110 overlap events, 0.00m clearance, neither truck ever
# finished -- a real sustained collision, strictly worse than the
# near-miss it was meant to fix). Single-token mutex instead (fan_holder,
# same claim-by-intent/release-on-both-clear pattern as pad_holder) --
# only one truck total may be in this box at a time, any direction, so a
# stopped truck can never be run into by one still moving.
#
# First attempt sized this to cover each truck's ENTIRE tail lane plus its
# dump target (x 40-95, y 125-160) -- same "too sticky" mistake as the
# original PAD_ZONE: T2 grabbed the lock at (44,132) and held it through
# its whole approach, dump, AND return leg (200+s) because its own dump
# target sat inside the box, leaving T1 frozen the entire time just
# trying to pass through once on its way out. Shrunk to bound only the
# actual crossing geometry -- where T1's (46,148)->(68,143) and T2's
# (44,132)->(60,145) tail segments pass within ~0.2m of each other, plus
# T3's (51,126)->(66,137) leg -- so a truck releases as soon as its
# target moves past the crossing onto its own diverged lane (x>75),
# rather than only once it's fully done with its whole dump cycle.
#
# That shrink introduced a worse bug: PAD_ZONE_Y tops out at 140 and this
# box starts at 128, so they overlap, and the route waypoint (44,132)
# falls inside BOTH. With pad_holder and fan_holder as two separate
# tokens, T1 (outbound, already holding fan_holder from its dump stop)
# and T2 (inbound, already holding pad_holder from (44,108)) each needed
# the OTHER's token to advance past (44,132) -- classic two-resource
# circular-wait deadlock, and both froze there permanently for the rest
# of the run. Fixed by giving pad_holder and fan_holder the SAME
# underlying token (self.zone_holder) instead of two independent ones --
# one resource can't deadlock against itself, only queue.
#
# X upper bound widened 75->90: T3's own original tail endpoint (82,144)
# sits only 0.12m from T2's tail endpoint (82,141) -- a latent risk that
# predates this session's edits (neither point ever changed) and was
# simply never covered, since x=82 falls outside the box that was sized
# around where the collision had actually been observed (x<=75).
FAN_ZONE_X = (42.0, 90.0)
FAN_ZONE_Y = (128.0, 156.0)


class State(Enum):
    QUEUED = auto()
    ANNOUNCED = auto()
    AWAITING_GRANT = auto()
    DRIVING_IN = auto()
    DUMPING = auto()
    DRIVING_OUT = auto()
    DONE = auto()


class TruckState:
    def __init__(self, tid, cls):
        self.id = tid; self.cls = cls
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.target_x = None; self.target_y = None; self.target_r = None
        self.grant_time = None; self.state = State.QUEUED
        self.locked = False; self.locked_since = None; self.dump_start = None
        self.waypoints = []; self.wp_idx = 0
        self.out_waypoints = []; self.out_idx = 0
        self.last_turn_dir = None
        self._dbg_tick = 0


class TruckOrchestrator(Node):
    def __init__(self):
        super().__init__('truck_orchestrator')
        self.trucks = {tid: TruckState(tid, cls) for tid, cls, _ in FLEET}
        self.announce_idx = 0
        self.t0 = time.time()
        self.gate_holder = None
        self.gate_holder_since = None
        # Single token shared by the pad-gate crossing AND the fan-out
        # crossing checks (see FAN_ZONE_X/Y comment) -- their boxes
        # overlap, so two separate tokens could deadlock against each
        # other at the shared waypoint; one token can only ever queue.
        self.zone_holder = None
        self.zone_holder_since = None
        self.zone_holder_last_pos = None
        self.corridor_trucks = {'in': set(), 'out': set()}

        # --- publishers ---
        self.pub_arrival   = self.create_publisher(Float64MultiArray, '/truck_arrivals',  10)
        self.pub_confirm   = self.create_publisher(Point,             '/dump_confirm',    10)
        self.pub_done      = self.create_publisher(Int32,             '/truck_completed', 10)
        self.pub_positions = self.create_publisher(PoseArray,         '/truck_positions', 10)
        self.pub_cmd = {tid: self.create_publisher(Twist, f'/truck_{tid}/cmd_vel', 10)
                        for tid in self.trucks}

        # --- subscribers (from MATLAB) ---
        self.create_subscription(Float64MultiArray, '/dump_plans',          self.on_plan,   10)
        self.create_subscription(Float64MultiArray, '/entry_grants',  self.on_grants, 10)
        self.create_subscription(Int32,             '/milp_lock',           self.on_lock,   10)
        self.create_subscription(Float64MultiArray, '/bay_assignments', self.on_bay, 10)

        # --- subscribers (from Gazebo via bridge) ---
        for tid in self.trucks:
            self.create_subscription(
                Odometry, f'/truck_{tid}/odom',
                lambda msg, t=tid: self.on_odom(msg, t), 10)
            self.create_subscription(
                Pose, f'/truck_{tid}/world_pose',
                lambda msg, t=tid: self.on_world_pose(msg, t), 10)

        # --- timers ---
        self.create_timer(0.1, self.tick)
        self.create_timer(0.1, self.publish_positions)
        self.create_timer(8.0, self.announce_next)

        self.get_logger().info("orchestrator up — 3 trucks, announcing every 3s")

    # ===================== Callbacks =====================
    def on_plan(self, msg):
        d = list(msg.data)
        if len(d) != 5: return
        tid = int(d[0])
        t = self.trucks.get(tid)
        if t is None: return
        t.target_x = float(d[2]); t.target_y = float(d[3]); t.target_r = float(d[4])
        if t.state == State.ANNOUNCED:
            t.state = State.AWAITING_GRANT
        self.get_logger().info(
            f"plan T{tid} → ({t.target_x:.1f}, {t.target_y:.1f}) r={t.target_r}")

    def on_grants(self, msg):
        d = list(msg.data)
        for i in range(0, len(d), 2):
            tid = int(d[i]); gt = float(d[i+1])
            t = self.trucks.get(tid)
            if t is None: continue
            t.grant_time = gt
            # Grant received = cleared to enter. MILP decides who/when via
            # the order grants arrive; we don't sync absolute clocks across nodes.
            if t.state == State.AWAITING_GRANT:
                t.state = State.DRIVING_IN
                self.get_logger().info(f"grant T{tid} — driving in")

    def on_lock(self, msg):
        # Sign convention: positive id = lock, negative id = unlock (the
        # matching release MATLAB's safety checker now sends once
        # clearance is restored -- see milp_traffic_cop.m's checkRelease).
        raw = int(msg.data)
        tid = abs(raw)
        t = self.trucks.get(tid)
        if t is None: return
        if raw < 0:
            t.locked = False
            t.locked_since = None
            self.get_logger().warn(f"!! UNLOCK T{tid} (MILP override cleared)")
        else:
            t.locked = True
            t.locked_since = time.time()
            self.get_logger().warn(f"!! LOCK T{tid} (MILP override)")

    def on_odom(self, msg, tid):
        # Not used for control: wheel odometry integrates commanded velocity
        # kinematically and silently diverges from true motion once turning
        # starts (measured ~3x over-integration of yaw vs. true pose). It made
        # drive_toward's yaw_err confidently wrong. world_pose is the sole
        # source of t.x/t.y/t.yaw below.
        pass

    def on_world_pose(self, msg, tid):
        # Single source of truth for control: already absolute world
        # coordinates (no spawn offset needed, unlike odom), and yaw is
        # recomputed fresh from the quaternion every callback via atan2
        # (always in (-π,π], so it can't accumulate the wrap/drift error
        # odom had).
        t = self.trucks[tid]
        t.x = msg.position.x
        t.y = msg.position.y
        q = msg.orientation
        t.yaw = self.angle_wrap(math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z)))

    # ===================== Main loop =====================
    def announce_next(self):
        if self.announce_idx >= len(FLEET): return
        tid, cls, _ = FLEET[self.announce_idx]
        t = self.trucks[tid]
        if t.state != State.QUEUED: return
        msg = Float64MultiArray(); msg.data = [float(tid), float(cls)]
        self.pub_arrival.publish(msg)
        t.state = State.ANNOUNCED
        self.get_logger().info(f"announced T{tid} (class {cls})")
        self.announce_idx += 1

    def publish_positions(self):
        msg = PoseArray()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        max_id = max(self.trucks.keys()) if self.trucks else 0
        for tid in range(1, max_id + 1):
            p = Pose()
            t = self.trucks.get(tid)
            if t is not None:
                p.position.x = t.x; p.position.y = t.y
            msg.poses.append(p)
        self.pub_positions.publish(msg)

    def tick(self):
        if self.zone_holder is not None:
            holder = self.trucks.get(self.zone_holder)
            moved = (self.zone_holder_last_pos is None or
                      math.hypot(holder.x - self.zone_holder_last_pos[0],
                                 holder.y - self.zone_holder_last_pos[1]) > 1.0)
            if moved:
                self.zone_holder_last_pos = (holder.x, holder.y)
                self.zone_holder_since = time.time()
            elif time.time() - self.zone_holder_since > ZONE_TIMEOUT:
                self.zone_holder = None
                self.zone_holder_since = None
                self.zone_holder_last_pos = None
        for t in self.trucks.values():
            if t.locked:
                if t.locked_since is not None and time.time() - t.locked_since > LOCK_TIMEOUT:
                    t.locked = False
                    t.locked_since = None
                    self.get_logger().warn(
                        f"!! LOCK T{t.id} force-released after {LOCK_TIMEOUT:.0f}s (no unlock received)")
                else:
                    self.stop(t); continue
            if t.state == State.DRIVING_IN:
                if not t.waypoints:
                    last = ROUTES[t.id][-1]
                    dx = t.target_x - last[0]; dy = t.target_y - last[1]
                    d = math.hypot(dx, dy)
                    if d > DUMP_STANDOFF:
                        ux, uy = dx / d, dy / d
                        stop = (t.target_x - ux * DUMP_STANDOFF,
                                t.target_y - uy * DUMP_STANDOFF)
                    else:
                        stop = (t.target_x, t.target_y)
                    t.waypoints = ROUTES[t.id] + [stop]
                    t.wp_idx = 0
                gx, gy = t.waypoints[t.wp_idx]
                is_final = (t.wp_idx == len(t.waypoints) - 1)
                tol = 3.0 if is_final else 6.0
                in_pad_zone = self._in_pad_zone(gx, gy)
                in_fan_zone = self._in_fan_zone(gx, gy)
                in_zone = in_pad_zone or in_fan_zone
                # Release only once the truck's ACTUAL position also clears
                # both zones, not just its next target -- releasing on
                # target alone left a gap where the holder's commanded
                # waypoint had already moved past the boundary while its
                # chassis was still physically sitting in the zone, letting
                # the next truck claim the lock and drive straight into it.
                if (self.zone_holder == t.id and not in_zone
                        and not self._in_pad_zone(t.x, t.y)
                        and not self._in_fan_zone(t.x, t.y)):
                    self.zone_holder = None
                    self.zone_holder_since = None
                    self.zone_holder_last_pos = None
                corridor_blocked = self._corridor_gate(t, 'in', gx, gy)
                gate_blocked = self._gate_lock(t, gy)
                dump_blocked = self._dumping_nearby(t, gx, gy)
                traffic_blocked = self._traffic_too_close(t, gx, gy)
                if ((in_zone and self.zone_holder not in (None, t.id))
                        or corridor_blocked or gate_blocked or dump_blocked or traffic_blocked):
                    self.stop(t)
                else:
                    if in_zone and self.zone_holder != t.id:
                        self.zone_holder = t.id
                        self.zone_holder_since = time.time()
                    # T2 was catching up to T1 at the corridor entrance
                    # chokepoint despite the entry-grant stagger -- T1's
                    # initial turn out of its bay eats into the gap more
                    # than the flat stagger time accounts for. Cap T2's
                    # speed here (not T1/T3, which weren't observed
                    # closing the gap) until it's actually through the
                    # gate, rather than relying on timing alone.
                    speed_cap = 2.0 if (t.id == 2 and t.y < 40.0) else 4.0
                    self.drive_toward(t, gx, gy, speed_cap=speed_cap)
                    if self.reached(t, gx, gy, tol=tol):
                        if is_final:
                            self.stop(t)
                            t.state = State.DUMPING
                            t.dump_start = time.time()
                            self.get_logger().info(f"T{t.id} dumping at ({t.x:.1f},{t.y:.1f})")
                        else:
                            t.wp_idx += 1
                            self.get_logger().info(
                                f"T{t.id} waypoint {t.wp_idx}/{len(t.waypoints)-1}")
            elif t.state == State.DUMPING:
                if time.time() - t.dump_start > 2.0:
                    self.spawn_pile(t)
                    p = Point(); p.x = t.x; p.y = t.y
                    self.pub_confirm.publish(p)
                    t.state = State.DRIVING_OUT
                    self.get_logger().info(f"T{t.id} dump complete — returning")
            elif t.state == State.DRIVING_OUT:
                # Retrace the truck's own inbound lane in reverse, rather
                # than heading straight at a single shared EXIT_AREA point:
                # all 3 trucks converging on one point with no lane
                # separation caused real truck-vs-truck collisions right at
                # the gate, since DRIVING_OUT previously ignored the
                # per-truck lane design that keeps DRIVING_IN collision-free.
                if not t.out_waypoints:
                    # Retrace the inbound lane in reverse, then peel off to
                    # this truck's OWN bay rather than ending at the shared
                    # merge point -- ending all 3 routes at the same point
                    # left whichever truck parked first blocking the lane
                    # for the next one to arrive.
                    t.out_waypoints = list(reversed(ROUTES[t.id])) + [BAY[t.id]]
                    t.out_idx = 0
                gx, gy = t.out_waypoints[t.out_idx]
                is_final = (t.out_idx == len(t.out_waypoints) - 1)
                tol = 5.0 if is_final else 6.0
                in_pad_zone = self._in_pad_zone(gx, gy)
                in_fan_zone = self._in_fan_zone(gx, gy)
                in_zone = in_pad_zone or in_fan_zone
                # See the matching comment in the DRIVING_IN branch -- only
                # release once the truck has actually left both zones, not
                # just its next target.
                if (self.zone_holder == t.id and not in_zone
                        and not self._in_pad_zone(t.x, t.y)
                        and not self._in_fan_zone(t.x, t.y)):
                    self.zone_holder = None
                    self.zone_holder_since = None
                    self.zone_holder_last_pos = None
                # All 3 routes share the same first 3 waypoints (the
                # south-gate/haul-road approach is a single real lane, only
                # ~24m wide -- not enough to also fit separate lanes for
                # three 5-9m-wide trucks). Hold here if another truck holds
                # the token for that shared stretch, like a real single-lane
                # haul road right-of-way rule -- see _gate_lock. Same
                # pattern for the dump-pad gate/fan-out zone (zone_holder).
                corridor_blocked = self._corridor_gate(t, 'out', gx, gy)
                gate_blocked = self._gate_lock(t, gy)
                dump_blocked = self._dumping_nearby(t, gx, gy)
                traffic_blocked = self._traffic_too_close(t, gx, gy)
                blocked = ((in_zone and self.zone_holder not in (None, t.id)) or
                           corridor_blocked or gate_blocked or dump_blocked or traffic_blocked)
                if blocked:
                    self.stop(t)
                else:
                    if in_zone and self.zone_holder != t.id:
                        self.zone_holder = t.id
                        self.zone_holder_since = time.time()
                    self.drive_toward(t, gx, gy)
                    if self.reached(t, gx, gy, tol=tol):
                        if is_final:
                            self.stop(t)
                            t.state = State.DONE
                            if self.gate_holder == t.id:
                                self.gate_holder = None
                                self.gate_holder_since = None
                            if self.zone_holder == t.id:
                                self.zone_holder = None
                                self.zone_holder_since = None
                                self.zone_holder_last_pos = None
                            m = Int32(); m.data = t.id
                            self.pub_done.publish(m)
                            self.get_logger().info(f"T{t.id} DONE")
                        else:
                            t.out_idx += 1

    # ===================== Drive helpers =====================
    def drive_toward(self, t, gx, gy, speed_cap=4.0):
        dx = gx - t.x; dy = gy - t.y
        dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_err = self.angle_wrap(target_yaw - t.yaw)
        tw = Twist()
        if abs(yaw_err) > 0.25:
            # Creep-and-turn, never a zero-vx pivot: verified empirically that
            # this truck's wheels can't break static friction sideways without
            # forward roll, so a standstill turn is a physical no-op even
            # though wheel odometry happily integrates it as if it worked.
            # Keep enough forward creep that the chassis actually rotates.
            tw.linear.x = 1.2
            if abs(abs(yaw_err) - math.pi) < 0.3:
                # Near the +/-pi wrap boundary, sign(yaw_err) can flip from
                # tiny pose noise even though the physical heading barely
                # moved -- reusing the last turn direction instead of
                # sign(yaw_err) avoids a chattering left-right-left limit
                # cycle that pins a truck in place needing a near-180-deg
                # reversal (observed: T3 frozen with yaw alternating
                # between +180 and -180 every sample).
                direction = t.last_turn_dir or (1.0 if yaw_err > 0 else -1.0)
            else:
                direction = 1.0 if yaw_err > 0 else -1.0
            t.last_turn_dir = direction
            tw.angular.z = 0.6 * direction
        else:
            tw.linear.x = min(speed_cap, dist * 0.4)
            tw.angular.z = 1.2 * yaw_err
            t.last_turn_dir = None
        self.pub_cmd[t.id].publish(tw)
        if t.id == 1:
            t._dbg_tick += 1
            if t._dbg_tick % 10 == 1:
                self.get_logger().info(
                    f'[T1] pos=({t.x:.1f},{t.y:.1f}) yaw={t.yaw:.3f}rad '
                    f'wp=({gx:.1f},{gy:.1f}) tgt_yaw={target_yaw:.3f}rad '
                    f'err={yaw_err:.3f}rad vx={tw.linear.x:.2f} wz={tw.angular.z:.2f}')

    def reached(self, t, gx, gy, tol):
        return math.hypot(gx - t.x, gy - t.y) < tol

    @staticmethod
    def _in_pad_zone(gx, gy):
        return PAD_ZONE_X[0] <= gx <= PAD_ZONE_X[1] and PAD_ZONE_Y[0] <= gy <= PAD_ZONE_Y[1]

    @staticmethod
    def _in_corridor(gy):
        return CORRIDOR_Y[0] <= gy <= CORRIDOR_Y[1]

    @staticmethod
    def _in_fan_zone(gx, gy):
        return FAN_ZONE_X[0] <= gx <= FAN_ZONE_X[1] and FAN_ZONE_Y[0] <= gy <= FAN_ZONE_Y[1]

    def _direction_gate(self, t, direction, in_zone_target, in_zone_actual, store):
        """Generic direction-exclusive lock: same-direction trucks may hold
        a zone together (their own fan lanes keep them apart), but a truck
        wanting the opposite direction waits for it to fully clear first.
        Returns True if t should be stopped. `other` excludes t's own id --
        without that, a truck whose stale membership from an earlier pass
        through this same zone (e.g. its DRIVING_IN approach) was never
        cleared (DUMPING doesn't call this) would block itself forever the
        moment it tries to claim the opposite direction here."""
        mine = store[direction]
        other = store['out' if direction == 'in' else 'in']
        if t.id in mine and not in_zone_target and not in_zone_actual:
            mine.discard(t.id)
        if in_zone_target and (other - {t.id}):
            return True
        if in_zone_target:
            mine.add(t.id)
        return False

    def _corridor_gate(self, t, direction, gx, gy):
        return self._direction_gate(
            t, direction, self._in_corridor(gy), self._in_corridor(t.y),
            self.corridor_trucks)

    def _gate_lock(self, t, gy):
        """Single-token mutex for the shared south-gate approach (y<40),
        covering both directions. Releases incrementally once BOTH the
        target and actual position clear the zone -- critical for an
        inbound truck, which still has ~100m of route left and would
        starve every other truck at this chokepoint for its whole transit
        if it held on like an outbound truck (which legitimately holds it
        all the way to BAY, since the zone IS its final stretch -- target
        never clears for that case, so this never fires early for it).
        Also force-releases if the same holder hasn't cleared in
        GATE_TIMEOUT seconds -- protects against the same kind of
        physics freeze seen elsewhere permanently wedging this lock."""
        in_south = gy < 40.0
        if self.gate_holder == t.id and not in_south and t.y >= 40.0:
            self.gate_holder = None
            self.gate_holder_since = None
        if (self.gate_holder not in (None, t.id) and self.gate_holder_since is not None
                and time.time() - self.gate_holder_since > GATE_TIMEOUT):
            self.gate_holder = None
            self.gate_holder_since = None
        if in_south and self.gate_holder not in (None, t.id):
            return True
        if in_south and self.gate_holder != t.id:
            self.gate_holder = t.id
            self.gate_holder_since = time.time()
        return False

    def _dumping_nearby(self, t, gx, gy):
        """True if t's target is within DUMP_SAFETY_RADIUS of some OTHER
        truck currently in DUMPING -- see DUMP_SAFETY_RADIUS comment."""
        for other in self.trucks.values():
            if other.id == t.id or other.state != State.DUMPING:
                continue
            if math.hypot(gx - other.x, gy - other.y) < DUMP_SAFETY_RADIUS:
                return True
        return False

    def _traffic_too_close(self, t, gx, gy):
        """True if t's target is within DUMP_SAFETY_RADIUS of a LOWER-id
        truck's CURRENT position, regardless of that truck's state.
        _dumping_nearby above only catches a collision risk while the
        other truck is actually in DUMPING -- but a real MATLAB-driven run
        put T2's and T3's NFP dump targets only ~12m apart, well outside
        FAN_ZONE/PAD_ZONE coverage (those top out around x=90), and their
        tail lanes crossed with 0.00m clearance while BOTH were still
        actively driving (neither one dumping yet) -- this only-while-
        dumping check can't see that case at all. Checking every other
        truck's live position (not just dumping ones) closes that gap.
        Only yields to a lower id, never the reverse, so this can't
        deadlock two trucks against each other -- exactly one direction of
        any pair ever blocks."""
        for other in self.trucks.values():
            if other.id >= t.id:
                continue
            if math.hypot(gx - other.x, gy - other.y) < DUMP_SAFETY_RADIUS:
                return True
        return False

    def stop(self, t):
        self.pub_cmd[t.id].publish(Twist())

    @staticmethod
    def angle_wrap(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    # ===================== Pile spawning =====================
    def spawn_pile(self, t):
        radius = t.target_r if t.target_r else 3.0
        # Spawn ahead of the truck, not centered on it -- the truck is
        # parked DUMP_STANDOFF short of the literal target, so placing the
        # pile further along its current heading lands it close to the
        # original NFP coordinate while keeping the truck's own chassis
        # clear of the new collision geometry (it previously spawned right
        # on top of the truck, wedging it in place).
        half_len = TRUCK_HALF_LEN.get(t.id, 7.5)
        offset = half_len + radius + 1.0
        px = t.x + offset * math.cos(t.yaw)
        py = t.y + offset * math.sin(t.yaw)
        sdf = f'''<?xml version="1.0"?>
<sdf version="1.10">
  <model name="pile_t{t.id}">
    <static>true</static>
    <pose>{px} {py} 0.5 0 0 0</pose>
    <link name="link">
      <visual name="v">
        <geometry><cylinder><radius>{radius}</radius><length>1.5</length></cylinder></geometry>
        <material><ambient>0.45 0.32 0.20 1</ambient><diffuse>0.65 0.48 0.30 1</diffuse></material>
      </visual>
      <collision name="c">
        <geometry><cylinder><radius>{radius}</radius><length>1.5</length></cylinder></geometry>
      </collision>
    </link>
  </model>
</sdf>'''
        with tempfile.NamedTemporaryFile('w', suffix='.sdf', delete=False) as f:
            f.write(sdf); tmp = f.name
        cmd = ['gz', 'service', '-s', '/world/cat_paddock/create',
               '--reqtype', 'gz.msgs.EntityFactory',
               '--reptype', 'gz.msgs.Boolean',
               '--timeout', '2000',
               '--req', f'sdf_filename: "{tmp}", name: "pile_t{t.id}"']
        try:
            subprocess.run(cmd, check=True, capture_output=True, timeout=3)
        except Exception as e:
            self.get_logger().warn(f"pile spawn failed: {e}")


    def on_bay(self, msg):
        d = list(msg.data)
        if len(d) >= 2:
            self.get_logger().info(f"bay assignment: T{int(d[0])} → bay {int(d[1])}")


def main():
    rclpy.init()
    node = TruckOrchestrator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
