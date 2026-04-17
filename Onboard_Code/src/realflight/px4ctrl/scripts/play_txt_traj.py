#!/usr/bin/env python3
import os
import numpy as np
import rospy
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import PoseStamped


def load_traj(path):
    if not os.path.isfile(path):
        raise FileNotFoundError(path)

    rows = []
    with open(path, "r") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            vals = [float(x) for x in s.split()]
            if len(vals) < 8:
                raise ValueError("Each line must have at least 8 columns: x y z vx vy vz yaw yaw_rate")
            rows.append(vals[:8])

    if len(rows) < 2:
        raise ValueError("Trajectory needs at least 2 points")

    return np.array(rows, dtype=np.float64)


def build_msg_from_row(row, acc_row, jerk_row, frame_id, yaw_bias=0.0):
    msg = PositionCommand()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
    msg.trajectory_id = 0

    msg.position.x = float(row[0])
    msg.position.y = float(row[1])
    msg.position.z = float(row[2])

    msg.velocity.x = float(row[3])
    msg.velocity.y = float(row[4])
    msg.velocity.z = float(row[5])

    msg.acceleration.x = float(acc_row[0])
    msg.acceleration.y = float(acc_row[1])
    msg.acceleration.z = float(acc_row[2])

    msg.jerk.x = float(jerk_row[0])
    msg.jerk.y = float(jerk_row[1])
    msg.jerk.z = float(jerk_row[2])

    msg.yaw = float(row[6] + yaw_bias)
    msg.yaw_dot = float(row[7])
    return msg


def build_msg_circle(t, frame_id, r, z, period, clockwise, center_x, center_y, yaw_cmd):
    omega = 2.0 * np.pi / period
    factor = -1.0 if clockwise else 1.0
    ang = omega * t

    x = -r * np.cos(ang) + center_x
    y = factor * -r * np.sin(ang) + center_y

    vx = r * omega * np.sin(ang)
    vy = factor * -r * omega * np.cos(ang)

    ax = r * (omega ** 2) * np.cos(ang)
    ay = factor * r * (omega ** 2) * np.sin(ang)

    jx = -r * (omega ** 3) * np.sin(ang)
    jy = factor * r * (omega ** 3) * np.cos(ang)

    msg = PositionCommand()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
    msg.trajectory_id = 0

    msg.position.x = float(x)
    msg.position.y = float(y)
    msg.position.z = float(z)

    msg.velocity.x = float(vx)
    msg.velocity.y = float(vy)
    msg.velocity.z = 0.0

    msg.acceleration.x = float(ax)
    msg.acceleration.y = float(ay)
    msg.acceleration.z = 0.0

    msg.jerk.x = float(jx)
    msg.jerk.y = float(jy)
    msg.jerk.z = 0.0

    msg.yaw = float(yaw_cmd)
    msg.yaw_dot = 0.0
    return msg


def yaw_from_quat_xyzw(x, y, z, w):
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main():
    rospy.init_node("play_txt_traj")

    traj_mode = rospy.get_param("~traj_mode", "circle").strip().lower()
    traj_file = rospy.get_param("~traj_file", "")
    topic = rospy.get_param("~topic", "/setpoints_cmd")
    dt = float(rospy.get_param("~time_step", 0.01))
    frame_id = rospy.get_param("~frame_id", "world")
    loop = bool(rospy.get_param("~loop", True))
    wait_for_sub = bool(rospy.get_param("~wait_for_sub", True))
    yaw_bias = float(rospy.get_param("~yaw_bias", 0.0))

    # Start gating
    start_mode = rospy.get_param("~start_mode", "on_trigger").strip().lower()  # immediate | on_trigger
    trigger_topic = rospy.get_param("~trigger_topic", "/traj_start_trigger")
    use_trigger_center = bool(rospy.get_param("~circle/use_trigger_center", True))
    use_trigger_altitude = bool(rospy.get_param("~circle/use_trigger_altitude", True))
    use_trigger_yaw = bool(rospy.get_param("~circle/use_trigger_yaw", False))

    # Built-in circle trajectory (no txt dependency)
    circle_radius = float(rospy.get_param("~circle/radius", 1.2))
    circle_altitude = float(rospy.get_param("~circle/altitude", 0.8))
    circle_period = float(rospy.get_param("~circle/period", 8.0))
    circle_center_x = float(rospy.get_param("~circle/center_x", 1.2))
    circle_center_y = float(rospy.get_param("~circle/center_y", 0.0))
    circle_clockwise = bool(rospy.get_param("~circle/clockwise", True))
    circle_duration = float(rospy.get_param("~circle/duration", 0.0))
    circle_yaw = float(rospy.get_param("~circle/yaw", 0.0))

    traj = None
    n = 0
    acc = None
    jerk = None
    if traj_mode == "txt":
        if not traj_file:
            rospy.logfatal("~traj_mode=txt but ~traj_file is empty")
            return
        traj = load_traj(traj_file)
        n = traj.shape[0]

        vel = traj[:, 3:6]
        acc = np.zeros_like(vel)
        acc[:-1, :] = (vel[1:, :] - vel[:-1, :]) / dt
        acc[-1, :] = acc[-2, :]

        jerk = np.zeros_like(vel)
        jerk[:-1, :] = (acc[1:, :] - acc[:-1, :]) / dt
        jerk[-1, :] = jerk[-2, :]
    elif traj_mode != "circle":
        rospy.logfatal("Unsupported ~traj_mode=%s, use 'circle' or 'txt'", traj_mode)
        return

    pub = rospy.Publisher(topic, PositionCommand, queue_size=20)
    rate = rospy.Rate(1.0 / dt)

    runtime = {
        "started": (start_mode == "immediate"),
        "center_x": circle_center_x,
        "center_y": circle_center_y,
        "altitude": circle_altitude,
        "yaw": circle_yaw,
        "t_start": rospy.Time.now().to_sec(),
    }

    if start_mode not in ("immediate", "on_trigger"):
        rospy.logfatal("Unsupported ~start_mode=%s, use 'immediate' or 'on_trigger'", start_mode)
        return

    def on_trigger(msg):
        if use_trigger_center:
            # Keep the first point of circle close to current position:
            # x(0) = center_x - r, y(0) = center_y
            runtime["center_x"] = msg.pose.position.x + circle_radius
            runtime["center_y"] = msg.pose.position.y
        if use_trigger_altitude:
            runtime["altitude"] = msg.pose.position.z
        if use_trigger_yaw:
            q = msg.pose.orientation
            runtime["yaw"] = float(yaw_from_quat_xyzw(q.x, q.y, q.z, q.w))

        runtime["t_start"] = rospy.Time.now().to_sec()
        runtime["started"] = True
        rospy.loginfo("[play_txt_traj] trigger received, start trajectory at center=(%.2f, %.2f), z=%.2f",
                      runtime["center_x"], runtime["center_y"], runtime["altitude"])

    trigger_sub = None
    if start_mode == "on_trigger":
        trigger_sub = rospy.Subscriber(trigger_topic, PoseStamped, on_trigger, queue_size=1)

    if wait_for_sub:
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and pub.get_num_connections() == 0:
            if rospy.Time.now().to_sec() - t0 > 5.0:
                break
            rospy.sleep(0.05)

    idx = 0
    seq = 0
    if traj_mode == "txt":
        rospy.loginfo("[play_txt_traj] mode=txt start, file=%s, points=%d, dt=%.3f", traj_file, n, dt)
    else:
        rospy.loginfo("[play_txt_traj] mode=circle start, r=%.2f, z=%.2f, T=%.2f, center=(%.2f, %.2f), cw=%s, dt=%.3f",
                      circle_radius, circle_altitude, circle_period, circle_center_x, circle_center_y,
                      str(circle_clockwise), dt)
    if start_mode == "on_trigger":
        rospy.loginfo("[play_txt_traj] waiting trigger on %s", trigger_topic)

    while not rospy.is_shutdown():
        if not runtime["started"]:
            rate.sleep()
            continue

        if traj_mode == "txt":
            msg = build_msg_from_row(traj[idx], acc[idx], jerk[idx], frame_id, yaw_bias=yaw_bias)
        else:
            t_now = rospy.Time.now().to_sec()
            t = t_now - runtime["t_start"]
            msg = build_msg_circle(
                t=t,
                frame_id=frame_id,
                r=circle_radius,
                z=runtime["altitude"],
                period=circle_period,
                clockwise=circle_clockwise,
                center_x=runtime["center_x"],
                center_y=runtime["center_y"],
                yaw_cmd=runtime["yaw"],
            )

        pub.publish(msg)

        seq += 1
        if traj_mode == "txt":
            idx += 1
            if idx >= n:
                if loop:
                    idx = 0
                else:
                    rospy.loginfo("[play_txt_traj] done")
                    break
        else:
            if (not loop) and circle_duration > 0.0:
                if (rospy.Time.now().to_sec() - runtime["t_start"]) >= circle_duration:
                    rospy.loginfo("[play_txt_traj] circle done")
                    break

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        rospy.logfatal("[play_txt_traj] %s", str(e))
