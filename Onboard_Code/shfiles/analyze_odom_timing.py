#!/usr/bin/env python3

"""Analyze Odom publish jitter and transport latency from a rosbag."""

import argparse
import csv
import sys

import numpy as np
import rosbag


def stats(values):
    if values.size == 0:
        return None
    return {
        "n": int(values.size),
        "mean": float(np.mean(values)),
        "std": float(np.std(values)),
        "min": float(np.min(values)),
        "p50": float(np.percentile(values, 50)),
        "p90": float(np.percentile(values, 90)),
        "p99": float(np.percentile(values, 99)),
        "max": float(np.max(values)),
    }


def print_stats(title, values, scale=1.0, unit="s"):
    s = stats(values)
    if s is None:
        print(f"{title}: no data")
        return
    print(
        f"{title}: n={s['n']}, mean={s['mean'] * scale:.3f}{unit}, "
        f"std={s['std'] * scale:.3f}{unit}, min={s['min'] * scale:.3f}{unit}, "
        f"p50={s['p50'] * scale:.3f}{unit}, p90={s['p90'] * scale:.3f}{unit}, "
        f"p99={s['p99'] * scale:.3f}{unit}, max={s['max'] * scale:.3f}{unit}"
    )


def top_indices(values, count=8, by_abs=False):
    if values.size == 0:
        return np.array([], dtype=int)
    key = np.abs(values) if by_abs else values
    if count >= key.size:
        return np.argsort(-key)
    idx = np.argpartition(-key, count - 1)[:count]
    return idx[np.argsort(-key[idx])]


def write_csv(path, t_record, t_header, delay, dt_record, dt_header, d_delay):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "t_record_s",
                "t_header_s",
                "delay_s",
                "dt_record_s",
                "dt_header_s",
                "delay_step_s",
            ]
        )
        for i in range(t_record.size):
            writer.writerow(
                [
                    f"{t_record[i]:.9f}",
                    f"{t_header[i]:.9f}",
                    f"{delay[i]:.9f}",
                    f"{dt_record[i - 1]:.9f}" if i > 0 else "",
                    f"{dt_header[i - 1]:.9f}" if i > 0 else "",
                    f"{d_delay[i - 1]:.9f}" if i > 0 else "",
                ]
            )


def main():
    parser = argparse.ArgumentParser(description="Analyze /Odom_high_freq timing quality in rosbag.")
    parser.add_argument("bag", help="Path to rosbag file")
    parser.add_argument("--topic", default="/Odom_high_freq", help="Odometry topic to analyze")
    parser.add_argument("--expected-hz", type=float, default=200.0, help="Expected odometry frequency")
    parser.add_argument("--focus-center", type=float, default=33.0, help="Focus time center in seconds")
    parser.add_argument("--focus-half-window", type=float, default=2.0, help="Focus half-window in seconds")
    parser.add_argument("--jitter-spike-ms", type=float, default=15.0, help="Jitter spike threshold in ms")
    parser.add_argument("--latency-spike-ms", type=float, default=15.0, help="Latency spike threshold from baseline in ms")
    parser.add_argument("--latency-step-ms", type=float, default=8.0, help="Latency step threshold between adjacent messages in ms")
    parser.add_argument("--top-k", type=int, default=8, help="Number of top events to print")
    parser.add_argument("--csv", default="", help="Optional CSV output path")
    args = parser.parse_args()

    t_record = []
    t_header = []
    delay = []

    try:
        with rosbag.Bag(args.bag, "r") as bag:
            bag_start = bag.get_start_time()
            for _, msg, t in bag.read_messages(topics=[args.topic]):
                if not hasattr(msg, "header"):
                    continue
                rec = t.to_sec() - bag_start
                hdr_abs = msg.header.stamp.to_sec()
                hdr = hdr_abs - bag_start
                t_record.append(rec)
                t_header.append(hdr)
                delay.append(t.to_sec() - hdr_abs)
    except Exception as e:
        print(f"Failed to read bag: {e}")
        return 2

    if len(t_record) < 3:
        print("Not enough odometry messages on topic", args.topic)
        return 1

    t_record = np.array(t_record, dtype=float)
    t_header = np.array(t_header, dtype=float)
    delay = np.array(delay, dtype=float)

    dt_record = np.diff(t_record)
    dt_header = np.diff(t_header)
    d_delay = np.diff(delay)

    dt_time = t_record[1:]
    delay_time = t_record
    d_delay_time = t_record[1:]

    focus_l = args.focus_center - args.focus_half_window
    focus_r = args.focus_center + args.focus_half_window

    in_focus_dt = (dt_time >= focus_l) & (dt_time <= focus_r)
    in_focus_delay = (delay_time >= focus_l) & (delay_time <= focus_r)
    in_focus_d_delay = (d_delay_time >= focus_l) & (d_delay_time <= focus_r)

    expected_dt = 1.0 / args.expected_hz
    jitter_spike_th = args.jitter_spike_ms / 1000.0
    latency_spike_th = args.latency_spike_ms / 1000.0
    latency_step_th = args.latency_step_ms / 1000.0

    baseline_mask = delay_time < focus_l
    if np.count_nonzero(baseline_mask) >= 50:
        baseline_delay = float(np.median(delay[baseline_mask]))
    else:
        baseline_delay = float(np.median(delay))

    jitter_spike_mask = dt_record > jitter_spike_th
    jitter_spike_focus = jitter_spike_mask & in_focus_dt

    latency_spike_mask = np.abs(delay - baseline_delay) > latency_spike_th
    latency_spike_focus = latency_spike_mask & in_focus_delay

    latency_step_mask = np.abs(d_delay) > latency_step_th
    latency_step_focus = latency_step_mask & in_focus_d_delay

    print("==== Odom Timing Analysis ====")
    print(f"Bag: {args.bag}")
    print(f"Topic: {args.topic}")
    print(f"Messages: {t_record.size}")
    print(f"Time range: {t_record[0]:.3f}s -> {t_record[-1]:.3f}s")
    print(f"Expected period: {expected_dt * 1000:.3f}ms ({args.expected_hz:.1f}Hz)")
    print(f"Focus window: [{focus_l:.3f}, {focus_r:.3f}]s")
    print()

    print("-- Publish interval jitter (record_time diff) --")
    print_stats("Global dt_record", dt_record, scale=1000.0, unit="ms")
    print_stats("Focus  dt_record", dt_record[in_focus_dt], scale=1000.0, unit="ms")
    print_stats("Global dt_header", dt_header, scale=1000.0, unit="ms")
    print_stats("Focus  dt_header", dt_header[in_focus_dt], scale=1000.0, unit="ms")
    print(
        f"Jitter spikes (> {args.jitter_spike_ms:.1f}ms): "
        f"global={np.count_nonzero(jitter_spike_mask)}, focus={np.count_nonzero(jitter_spike_focus)}"
    )
    print()

    print("-- Transport latency (record_time - header.stamp) --")
    print(f"Baseline delay (median before focus): {baseline_delay * 1000.0:.3f}ms")
    print_stats("Global delay", delay, scale=1000.0, unit="ms")
    print_stats("Focus  delay", delay[in_focus_delay], scale=1000.0, unit="ms")
    print_stats("Focus  delay step", d_delay[in_focus_d_delay], scale=1000.0, unit="ms")
    print(
        f"Latency spikes (|delay-baseline| > {args.latency_spike_ms:.1f}ms): "
        f"global={np.count_nonzero(latency_spike_mask)}, focus={np.count_nonzero(latency_spike_focus)}"
    )
    print(
        f"Latency steps (|delta delay| > {args.latency_step_ms:.1f}ms): "
        f"global={np.count_nonzero(latency_step_mask)}, focus={np.count_nonzero(latency_step_focus)}"
    )
    print()

    print("-- Top jitter events in focus (largest dt_record) --")
    focus_idx_dt = np.where(in_focus_dt)[0]
    top_dt_local = top_indices(dt_record[focus_idx_dt], count=args.top_k)
    if top_dt_local.size == 0:
        print("No dt events in focus window")
    else:
        for rank, local_idx in enumerate(top_dt_local, start=1):
            i = focus_idx_dt[local_idx]
            print(
                f"{rank:2d}. t={dt_time[i]:.3f}s, dt_record={dt_record[i] * 1000.0:.3f}ms, "
                f"dt_header={dt_header[i] * 1000.0:.3f}ms"
            )
    print()

    print("-- Top latency events in focus (largest |delay-baseline|) --")
    focus_idx_delay = np.where(in_focus_delay)[0]
    delay_err = np.abs(delay - baseline_delay)
    top_delay_local = top_indices(delay_err[focus_idx_delay], count=args.top_k)
    if top_delay_local.size == 0:
        print("No delay events in focus window")
    else:
        for rank, local_idx in enumerate(top_delay_local, start=1):
            i = focus_idx_delay[local_idx]
            print(
                f"{rank:2d}. t={delay_time[i]:.3f}s, delay={delay[i] * 1000.0:.3f}ms, "
                f"offset_from_baseline={(delay[i] - baseline_delay) * 1000.0:.3f}ms"
            )
    print()

    print("-- Top latency step events in focus (largest |delta delay|) --")
    focus_idx_d_delay = np.where(in_focus_d_delay)[0]
    top_step_local = top_indices(np.abs(d_delay[focus_idx_d_delay]), count=args.top_k)
    if top_step_local.size == 0:
        print("No delay-step events in focus window")
    else:
        for rank, local_idx in enumerate(top_step_local, start=1):
            i = focus_idx_d_delay[local_idx]
            print(
                f"{rank:2d}. t={d_delay_time[i]:.3f}s, delay_step={d_delay[i] * 1000.0:.3f}ms"
            )
    print()

    if args.csv:
        write_csv(args.csv, t_record, t_header, delay, dt_record, dt_header, d_delay)
        print(f"CSV written: {args.csv}")

    jitter_flag = np.count_nonzero(jitter_spike_focus) > 0
    latency_flag = (
        np.count_nonzero(latency_spike_focus) > 0
        or np.count_nonzero(latency_step_focus) > 0
    )
    print()
    print("==== Quick Verdict ====")
    print(f"Focus jitter anomaly: {'YES' if jitter_flag else 'NO'}")
    print(f"Focus latency anomaly: {'YES' if latency_flag else 'NO'}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
