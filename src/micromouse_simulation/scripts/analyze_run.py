#!/usr/bin/env python3
"""Offline analysis of a recorded micromouse run (rosbag2).

Reads the topics captured by sim.launch.py (record:=true) and prints a
control-diagnostics report: real-time factor, whether the robot actually
moved, heading tracking, wheel-command jitter, perception rates, and an
IR-vs-odometry cross-check that flags the "odometry raced ahead of the
body" failure mode without needing ground-truth pose.

Usage:
  source install/setup.bash
  python3 src/micromouse_simulation/scripts/analyze_run.py [BAG_DIR]

If BAG_DIR is omitted, the most recent run_* under <ws>/bags is analyzed.
"""
import glob
import math
import os
import sys
from collections import defaultdict

import rclpy.serialization
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

NS = 1e-9


def find_latest_bag():
    here = os.path.dirname(os.path.abspath(__file__))
    roots = [os.path.abspath(os.path.join(here, "..", "..", "..", "..")), os.getcwd()]
    cands = []
    for root in roots:
        cands += glob.glob(os.path.join(root, "bags", "run_*"))
    cands = [c for c in cands if os.path.isdir(c)]
    return max(cands, key=os.path.getmtime) if cands else None


def detect_storage(path):
    if glob.glob(os.path.join(path, "*.mcap")):
        return "mcap"
    if glob.glob(os.path.join(path, "*.db3")):
        return "sqlite3"
    return "mcap"


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=detect_storage(path)),
        rosbag2_py.ConverterOptions("", ""),
    )
    typemap = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msgs = defaultdict(list)
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic not in typemap:
            continue
        msg = rclpy.serialization.deserialize_message(data, get_message(typemap[topic]))
        msgs[topic].append((t, msg))
    return msgs


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def stats(xs):
    if not xs:
        return (0.0, 0.0, 0.0, 0.0)
    n = len(xs)
    m = sum(xs) / n
    var = sum((x - m) ** 2 for x in xs) / n
    return (min(xs), m, max(xs), math.sqrt(var))


def section(title):
    print("\n" + "=" * 64)
    print(title)
    print("=" * 64)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else find_latest_bag()
    if not path or not os.path.isdir(path):
        print("No bag found. Pass a bag dir, or run with record:=true first.")
        return 1
    print(f"Analyzing: {path}")
    msgs = read_bag(path)

    wall_times = [t for tl in msgs.values() for (t, _) in tl]
    if not wall_times:
        print("Bag is empty.")
        return 1
    wall_span = (max(wall_times) - min(wall_times)) * NS

    section("TIMING / REAL-TIME FACTOR")
    clock = msgs.get("/clock", [])
    if clock:
        def sim_s(m):
            return m.clock.sec + m.clock.nanosec * NS
        sim_span = sim_s(clock[-1][1]) - sim_s(clock[0][1])
        rtf = sim_span / wall_span if wall_span > 0 else 0.0
        print(f"  wall duration : {wall_span:7.2f} s")
        print(f"  sim  duration : {sim_span:7.2f} s")
        print(f"  real-time factor ~= {rtf:.3f}"
              + ("   <-- LOW: perception will lag, dt noisy" if rtf < 0.5 else ""))
    else:
        print("  no /clock recorded")

    section("PERCEPTION RATES (effective, wall-clock)")
    for topic in ["/imu", "/ir_front", "/ir_left", "/ir_right", "/odom",
                  "/joint_states", "/motion_setpoint",
                  "/wheel_velocity_controller/commands"]:
        n = len(msgs.get(topic, []))
        hz = n / wall_span if wall_span > 0 else 0.0
        print(f"  {topic:42s} {n:6d} msgs  {hz:6.1f} Hz")

    section("DID THE ROBOT ACTUALLY MOVE? (odometry)")
    odom = msgs.get("/odom", [])
    if odom:
        xs = [m.pose.pose.position.x for _, m in odom]
        ys = [m.pose.pose.position.y for _, m in odom]
        x0, y0 = xs[0], ys[0]
        disp = max(math.hypot(x - x0, y - y0) for x, y in zip(xs, ys))
        path_len = sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
                       for i in range(1, len(xs)))
        print(f"  start (odom)        : ({x0:.3f}, {y0:.3f})")
        print(f"  end   (odom)        : ({xs[-1]:.3f}, {ys[-1]:.3f})")
        print(f"  max displacement    : {disp:.3f} m  (cells ~ {disp / 0.18:.1f})")
        print(f"  integrated path len : {path_len:.3f} m  (cells ~ {path_len / 0.18:.1f})")
        if path_len > 0.5 and disp < 0.25:
            print("  !! odometry integrated a long path but net displacement is tiny:")
            print("     classic wheel-slip / spinning-in-place signature.")
    else:
        print("  no /odom recorded")

    section("HEADING TRACKING (setpoint vs estimate)")
    sp = msgs.get("/motion_setpoint", [])
    if sp and odom:
        # nearest-earlier setpoint for each odom sample
        sp_sorted = sorted(sp, key=lambda x: x[0])
        errs, speeds = [], []
        j = 0
        for t, om in odom:
            while j + 1 < len(sp_sorted) and sp_sorted[j + 1][0] <= t:
                j += 1
            tgt = sp_sorted[j][1]
            errs.append(abs(wrap(tgt.target_heading - yaw_of(om.pose.pose.orientation))))
            speeds.append(tgt.target_speed)
        lo, mean, hi, sd = stats([e * 180 / math.pi for e in errs])
        print(f"  |heading error| deg : mean {mean:5.1f}  max {hi:5.1f}  sd {sd:5.1f}")
        moving = sum(1 for s in speeds if s > 0.01)
        print(f"  setpoints commanding motion: {moving}/{len(speeds)}")
        if mean > 10:
            print("  !! large mean heading error: heading loop not converging /")
            print("     estimate not matching commanded frame.")
    else:
        print("  need /motion_setpoint and /odom")

    section("WHEEL COMMAND JITTER (actuation smoothness)")
    cmd = msgs.get("/wheel_velocity_controller/commands", [])
    if cmd:
        left = [m.data[0] for _, m in cmd if len(m.data) >= 2]
        right = [m.data[1] for _, m in cmd if len(m.data) >= 2]
        for name, seq in (("left", left), ("right", right)):
            if len(seq) < 2:
                continue
            dribble = [abs(seq[i] - seq[i - 1]) for i in range(1, len(seq))]
            _, dmean, dmax, _ = stats(dribble)
            _, _, _, sd = stats(seq)
            print(f"  {name:5s} cmd rad/s: sd {sd:6.2f}  "
                  f"mean|Δ| {dmean:6.2f}  max|Δ| {dmax:6.2f}")
        big = sum(1 for i in range(1, len(left)) if abs(left[i] - left[i - 1]) > 10)
        if big > 0.05 * max(1, len(left)):
            print("  !! frequent large step-to-step jumps: actuation is chattering.")
    else:
        print("  no /wheel_velocity_controller/commands recorded")

    section("WHEEL SERVO TRACKING (commanded vs measured)")
    js = msgs.get("/joint_states", [])
    if js and cmd:
        # crude time-aligned comparison of commanded vs measured wheel speed
        def measured(seq, name):
            out = []
            for _, m in seq:
                if name in m.name:
                    out.append(m.velocity[m.name.index(name)])
            return out
        ml = measured(js, "left_wheel_joint")
        if ml:
            _, _, _, sdm = stats(ml)
            _, _, _, sdc = stats([m.data[0] for _, m in cmd if len(m.data) >= 2])
            print(f"  left measured sd {sdm:6.2f} vs commanded sd {sdc:6.2f}")
    else:
        print("  need /joint_states and wheel commands")

    section("ODOMETRY vs GROUND TRUTH (does odom lie?)")
    gt = msgs.get("/ground_truth/tf", [])
    gt_xy = []
    for t, m in gt:
        for tr in m.transforms:
            if "micromouse" in tr.child_frame_id or "base_link" in tr.child_frame_id:
                gt_xy.append((t, tr.transform.translation.x, tr.transform.translation.y))
                break
    if gt_xy and odom:
        gx0, gy0 = gt_xy[0][1], gt_xy[0][2]
        gdisp = max(math.hypot(x - gx0, y - gy0) for _, x, y in gt_xy)
        print(f"  true start          : ({gx0:.3f}, {gy0:.3f})")
        print(f"  true end            : ({gt_xy[-1][1]:.3f}, {gt_xy[-1][2]:.3f})")
        print(f"  true max displacement: {gdisp:.3f} m  (cells ~ {gdisp / 0.18:.1f})")
        # final odom-vs-truth error
        ox, oy = odom[-1][1].pose.pose.position.x, odom[-1][1].pose.pose.position.y
        err = math.hypot(ox - gt_xy[-1][1], oy - gt_xy[-1][2])
        print(f"  final odom-vs-truth error: {err:.3f} m")
        if err > 0.18:
            print("  !! odom and ground truth disagree by > 1 cell: the solver is")
            print("     navigating in a frame detached from reality.")
    else:
        print("  no /ground_truth/tf recorded (rebuild + relaunch to capture it)")

    section("IR vs ODOMETRY CROSS-CHECK")
    front = msgs.get("/ir_front", [])
    if front and odom:
        fr = [m.ranges[0] for _, m in front if m.ranges]
        lo, mean, hi, sd = stats([r for r in fr if math.isfinite(r) and r < 5])
        print(f"  front IR range m    : min {lo:.3f}  mean {mean:.3f}  max {hi:.3f}  sd {sd:.3f}")
        print("  (if odom shows many cells crossed but front IR barely varies,")
        print("   the body is not moving through the maze the way odom claims)")
    else:
        print("  need /ir_front and /odom")

    print("\nDone.\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
