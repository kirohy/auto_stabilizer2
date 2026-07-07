#!/usr/bin/env python3

import argparse
import math
import re
from collections import defaultdict


def is_float(token):
    try:
        float(token)
        return True
    except ValueError:
        return False


def split_tokens(line):
    return [token for token in re.split(r"[\s,]+", line.strip()) if token]


def parse_records(tokens):
    if not tokens:
        return []

    candidates = [tokens]
    if len(tokens) > 1 and re.match(r"^\d+$", tokens[0]):
        candidates.append(tokens[1:])

    for body in candidates:
        # link1 p1xyz link2 p2xyz dirxyz distance
        if len(body) % 12 == 0:
            records = []
            ok = True
            for i in range(0, len(body), 12):
                rec = body[i:i + 12]
                numeric = rec[1:4] + rec[5:12]
                if not all(is_float(v) for v in numeric):
                    ok = False
                    break
                records.append((rec[0], rec[4], float(rec[11])))
            if ok:
                return records

        # p1xyz p2xyz dirxyz distance。link名が記録されないlogger向け。
        if len(body) % 10 == 0:
            records = []
            ok = True
            for index, i in enumerate(range(0, len(body), 10)):
                rec = body[i:i + 10]
                if not all(is_float(v) for v in rec):
                    ok = False
                    break
                records.append(("index{}".format(index), "", float(rec[9])))
            if ok:
                return records

    return []


def percentile(values, pct):
    if not values:
        return float("nan")
    sorted_values = sorted(values)
    pos = (len(sorted_values) - 1) * pct / 100.0
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return sorted_values[lo]
    return sorted_values[lo] * (hi - pos) + sorted_values[hi] * (pos - lo)


def summarize(log_path, on_threshold, off_threshold):
    samples = []
    by_key = defaultdict(list)
    unparsable = 0

    with open(log_path) as f:
        for line in f:
            tokens = split_tokens(line)
            if len(tokens) < 2 or not is_float(tokens[0]):
                continue
            t = float(tokens[0])
            records = parse_records(tokens[1:])
            if not records and len(tokens) > 1:
                unparsable += 1
                continue
            samples.append((t, records))
            for index, (link1, link2, distance) in enumerate(records):
                key = "{}:{}:{}".format(index, link1, link2)
                by_key[key].append((t, distance))

    active_counts = []
    hysteresis_counts = []
    hysteresis_state = {}
    active_count_toggles = 0
    hysteresis_count_toggles = 0
    prev_active_count = None
    prev_hysteresis_count = None

    for _, records in samples:
        active_count = sum(1 for _, _, distance in records if distance < on_threshold)
        active_counts.append(active_count)
        if prev_active_count is not None and active_count != prev_active_count:
            active_count_toggles += 1
        prev_active_count = active_count

        for index, (link1, link2, distance) in enumerate(records):
            key = "{}:{}:{}".format(index, link1, link2)
            active = hysteresis_state.get(key, False)
            if active:
                if distance > off_threshold:
                    active = False
            else:
                if distance < on_threshold:
                    active = True
            hysteresis_state[key] = active

        current_keys = set("{}:{}:{}".format(index, link1, link2) for index, (link1, link2, _) in enumerate(records))
        for key in list(hysteresis_state.keys()):
            if key not in current_keys:
                hysteresis_state[key] = False

        hysteresis_count = sum(1 for active in hysteresis_state.values() if active)
        hysteresis_counts.append(hysteresis_count)
        if prev_hysteresis_count is not None and hysteresis_count != prev_hysteresis_count:
            hysteresis_count_toggles += 1
        prev_hysteresis_count = hysteresis_count

    print("file:", log_path)
    print("samples:", len(samples))
    print("unparsable_lines:", unparsable)
    if not samples:
        print("no parsable samples")
        return

    print("candidates:", len(by_key))
    print("active_count min/mean/p95/p99/max: {} {:.3f} {:.3f} {:.3f} {}".format(
        min(active_counts),
        sum(active_counts) / float(len(active_counts)),
        percentile(active_counts, 95),
        percentile(active_counts, 99),
        max(active_counts)))
    print("active_count_toggles:", active_count_toggles)
    print("hysteresis_count min/mean/p95/p99/max: {} {:.3f} {:.3f} {:.3f} {}".format(
        min(hysteresis_counts),
        sum(hysteresis_counts) / float(len(hysteresis_counts)),
        percentile(hysteresis_counts, 95),
        percentile(hysteresis_counts, 99),
        max(hysteresis_counts)))
    print("hysteresis_count_toggles:", hysteresis_count_toggles)

    rows = []
    for key, values in by_key.items():
        distances = [distance for _, distance in values]
        raw_active = [distance < on_threshold for distance in distances]
        raw_toggles = sum(1 for i in range(1, len(raw_active)) if raw_active[i] != raw_active[i - 1])
        near = sum(1 for distance in distances if on_threshold <= distance <= off_threshold)
        rows.append((raw_toggles, near, min(distances), max(distances), key))

    print("top_toggle_candidates:")
    for raw_toggles, near, min_distance, max_distance, key in sorted(rows, reverse=True)[:10]:
        print("  toggles={} near_band={} min={:.6f} max={:.6f} key={}".format(
            raw_toggles, near, min_distance, max_distance, key))


def main():
    parser = argparse.ArgumentParser(description="M5.8 self collision active set解析")
    parser.add_argument("log", help="DataLoggerで取得したCollisionChecker0_collisionOutログ")
    parser.add_argument("--on-threshold", type=float, default=0.05)
    parser.add_argument("--off-threshold", type=float, default=0.07)
    args = parser.parse_args()
    summarize(args.log, args.on_threshold, args.off_threshold)


if __name__ == "__main__":
    main()
