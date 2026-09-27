"""
Runs the PUSHPAK T-sweep simulation N times (Monte Carlo, one random POI layout
per trial, mirroring reset() -> rng = Math.random() in the original HTML) and
writes:
  1. pushpak_runs_raw.csv       - every metric, one row per trial
  2. pushpak_summary_stats.csv  - min / mean / median / mode / p5 / p95 per metric

Usage:
    python3 monte_carlo.py --n 500 --seed 12345 --out-dir /mnt/user-data/outputs
"""

import argparse
import csv
import importlib.util
from pathlib import Path
import statistics
import time
from collections import Counter
from numbers import Real


def load_pushpak(module_name="puspak.py"):
    """Load the simulation module located beside this script."""
    module_path = Path(__file__).resolve().with_name(module_name)
    spec = importlib.util.spec_from_file_location("pushpak", module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Unable to load simulation module: {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


pushpak = load_pushpak()
run_once = pushpak.run_once
TICK_S = pushpak.TICK_S

# metrics that only make sense when a mission actually finished; NaN otherwise
def safe_mean(values):
    return statistics.fmean(values) if values else float("nan")


def extract_metrics(r):
    delays_min = [d * TICK_S / 60 for d in r["delays"]]
    viol = r["viol"]

    up = r["linkUpTicks"]
    down = r["linkDownTicks"]
    considered = up + down
    connectivity_pct = (up / considered * 100) if considered else float("nan")
    downtime_s = down * TICK_S

    return {
        "ticks_simulated": r["t"],
        "completed_flag": 1 if r["done"] else 0,
        "mission_duration_min": (r["doneTick"] if r["doneTick"] is not None else r["t"]) * TICK_S / 60,
        "recall_time_min": (r["recallTick"] * TICK_S / 60) if r["recallTick"] is not None else float("nan"),
        "pois_appeared": r["appeared"],
        "pois_found": r["found"],
        "pois_reported_on_time": r["reportedOnTime"],
        "pois_late_or_unreported": r["found"] - r["reportedOnTime"],
        "avg_appear_to_report_delay_min": safe_mean(delays_min),
        "max_appear_to_report_delay_min": max(delays_min) if delays_min else float("nan"),
        "drones_used": r["used"],
        "longest_flight_min": r["longest"] * TICK_S / 60,
        "violations_over_20min_flight": viol["over20"],
        "violations_late_landing": viol["late"],
        "violations_late_report": viol["lateReport"],
        "total_rule_breaks": viol["over20"] + viol["late"] + viol["lateReport"],
        "drones_in_air_at_end": r["air"],
        "targets_held": r.get("held", r["found"]),
        # connectivity / separation / relay metrics
        "connectivity_availability_pct": connectivity_pct,
        "communication_downtime_s": downtime_s,
        "relay_reallocations": r["relayReallocations"],
        "min_interuav_separation_m": r["minSeparationM"] if r["minSeparationM"] is not None else float("nan"),
        # priority-weighted mission scores
        "priority_weighted_completion_pct": r["priorityWeightedCompletionPct"],
        "priority_weighted_decay_pct": r["priorityWeightedDecayPct"],
        # packet delivery and latency
        "packets_sent": r["packetsSent"],
        "packets_delivered": r["packetsDelivered"],
        "packet_delivery_ratio_pct": r["packetDeliveryRatioPct"],
        "avg_packet_latency_ticks": safe_mean(r["packetLatencyTicks"]),
        "max_packet_latency_ticks": max(r["packetLatencyTicks"]) if r["packetLatencyTicks"] else float("nan"),
        # failure, recovery, and reconfiguration
        "failure_occurred": 1 if r["failureOccurred"] else 0,
        "failure_tick": r["failureTick"] if r["failureTick"] is not None else float("nan"),
        "failure_role": r["failureRole"] or "none",
        "recovered": 1 if r["recovered"] else 0,
        "recovery_ticks": r["recoveryTicks"] if r["recoveryTicks"] is not None else float("nan"),
        "reconfig_efficiency_pct": r["reconfigEfficiencyPct"] if r["reconfigEfficiencyPct"] is not None else float("nan"),
        # movement model invariant
        "collision_count": r["collisionCount"],
    }


METRIC_ORDER = [
    "ticks_simulated", "completed_flag", "mission_duration_min", "recall_time_min",
    "pois_appeared", "pois_found", "pois_reported_on_time", "pois_late_or_unreported",
    "avg_appear_to_report_delay_min", "max_appear_to_report_delay_min",
    "drones_used", "longest_flight_min",
    "violations_over_20min_flight", "violations_late_landing", "violations_late_report",
    "total_rule_breaks", "drones_in_air_at_end", "targets_held",
    "connectivity_availability_pct", "communication_downtime_s",
    "relay_reallocations", "min_interuav_separation_m",
    "priority_weighted_completion_pct", "priority_weighted_decay_pct",
    "packets_sent", "packets_delivered", "packet_delivery_ratio_pct",
    "avg_packet_latency_ticks", "max_packet_latency_ticks",
    "failure_occurred", "failure_tick", "failure_role", "recovered",
    "recovery_ticks", "reconfig_efficiency_pct", "collision_count",
]


def mode_of(values):
    """Most frequent value; ties broken by the smallest value. NaNs ignored."""
    clean = [v for v in values if v == v]  # drop NaN
    if not clean:
        return float("nan")
    counts = Counter(clean)
    best = max(counts.values())
    candidates = [v for v, c in counts.items() if c == best]
    return min(candidates)


def percentile(values, pct):
    clean = sorted(v for v in values if v == v)
    if not clean:
        return float("nan")
    if len(clean) == 1:
        return clean[0]
    k = (len(clean) - 1) * (pct / 100)
    f = int(k)
    c = min(f + 1, len(clean) - 1)
    if f == c:
        return clean[f]
    d0 = clean[f] * (c - k)
    d1 = clean[c] * (k - f)
    return d0 + d1


def summarize(rows):
    summary = []
    for m in METRIC_ORDER:
        vals = [row[m] for row in rows]
        observed = [v for v in vals if v == v]
        clean = [v for v in observed if isinstance(v, Real)]
        summary.append({
            "metric": m,
            "n": len(observed),
            "min": min(clean) if clean else float("nan"),
            "mean": safe_mean(clean),
            "median": statistics.median(clean) if clean else float("nan"),
            "mode": mode_of(vals),
            "p5": percentile(clean, 5),
            "p95": percentile(clean, 95),
            "max": max(clean) if clean else float("nan"),
        })
    return summary


def main():
    global run_once, TICK_S
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=500, help="number of Monte Carlo trials")
    ap.add_argument("--seed", type=int, default=12345, help="base RNG seed (each trial uses seed+i)")
    ap.add_argument("--out-dir", type=str, default=".", help="output directory for CSVs")
    ap.add_argument("--engine", choices=("puspak", "report_hold"), default="puspak",
                    help="simulation engine: original sweep or report-and-hold")
    ap.add_argument("--appear-window", type=int, default=None,
                    help="POI appearance window in ticks (report_hold only)")
    args = ap.parse_args()
    if args.engine == "report_hold":
        pushpak = load_pushpak("puspak_repot&hold.py")
        run_once = pushpak.run_once
        TICK_S = pushpak.TICK_S
    Path(args.out_dir).mkdir(parents=True, exist_ok=True)

    t0 = time.time()
    rows = []
    for i in range(args.n):
        kwargs = {"seed": args.seed + i}
        if args.engine == "report_hold" and args.appear_window is not None:
            kwargs["appear_window"] = args.appear_window
        r = run_once(**kwargs)
        rows.append(extract_metrics(r))
        if (i + 1) % 50 == 0:
            print(f"  {i + 1}/{args.n} trials done ({time.time() - t0:.1f}s elapsed)")

    raw_path = f"{args.out_dir}/pushpak_runs_raw.csv"
    with open(raw_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["trial"] + METRIC_ORDER)
        w.writeheader()
        for i, row in enumerate(rows):
            out = {"trial": i + 1}
            out.update(row)
            w.writerow(out)

    summary = summarize(rows)
    summary_path = f"{args.out_dir}/pushpak_summary_stats.csv"
    with open(summary_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["metric", "n", "min", "mean", "median", "mode", "p5", "p95", "max"])
        w.writeheader()
        for row in summary:
            w.writerow(row)

    print(f"\nDone: {args.n} trials in {time.time() - t0:.1f}s")
    print(f"Raw per-trial metrics -> {raw_path}")
    print(f"Summary stats         -> {summary_path}")


if __name__ == "__main__":
    main()