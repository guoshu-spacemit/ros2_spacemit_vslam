#!/usr/bin/env python3
"""Check exact Track selection, then report raw (unaligned) pose differences."""

import argparse
import csv
import json
import math
import re
from pathlib import Path
import statistics


def read_rows(path):
    with path.open() as stream:
        return list(csv.DictReader(line for line in stream if not line.startswith("#")))


def keys(rows):
    return [(int(row["source_frame_index"]), int(row["image_timestamp_ns"])) for row in rows]


def distribution(values):
    if not values:
        return None
    return {
        "count": len(values),
        "median": statistics.median(values),
        "rmse": math.sqrt(sum(v * v for v in values) / len(values)),
        "max": max(values),
    }


def check_sba_execution(output, expected):
    path = output / "launch.log"
    text = path.read_text(errors="replace") if path.is_file() else ""
    summaries = re.findall(
        r"SBA replay summary: frames=(\d+) mismatches=(\d+) resets=(\d+) recorded_schedule=(true|false)", text
    )
    complete = (
        len(summaries) == 1
        and int(summaries[0][0]) == expected["frames"]
        and int(summaries[0][1]) == 0
        and int(summaries[0][2]) == expected.get("resets", 0)
        and summaries[0][3] == "true"
        and "SBA replay mismatch:" not in text
    )
    return {"verified": complete, "summaries": summaries, "log": str(path)}


def check(source, output):
    original = read_rows(source / "vio_frames.csv")
    replay = read_rows(output / "vio_frames.csv")
    summary = json.loads((output / "vio_frames.csv.summary.json").read_text())
    producer = json.loads((output / "replay_summary.json").read_text())
    original_summary = json.loads((source / "vio_frames.csv.summary.json").read_text())
    with (source / "camera0_timestamps.txt").open() as stream:
        camera = list(csv.DictReader((line for line in stream if not line.startswith("#")), delimiter=" "))
    exact = bool(original) and keys(original) == keys(replay)
    source_clean = (
        original_summary["written"] == len(original)
        and original_summary["dropped"] == 0
        and not original_summary["write_failed"]
    )
    writer_clean = summary["written"] == len(replay) and summary["dropped"] == 0 and not summary["write_failed"]
    producer_clean = (
        producer["complete"]
        and producer["targets"] == len(original)
        and producer["acknowledged"] == len(original)
        and producer["camera_frames"] == len(camera)
        and not producer["error"]
    )
    invalid = []
    translation = []
    rotation = []
    status_changes = 0
    fields = ("tx", "ty", "tz", "qx", "qy", "qz", "qw")
    for index, row in enumerate(replay):
        if row["status"] not in ("ok", "invalid_pose", "track_error"):
            invalid.append(index)
        if row["status"] == "ok":
            values = [float(row[field]) for field in fields]
            if (
                not all(math.isfinite(v) for v in values)
                or abs(sum(v * v for v in values[3:]) - 1) > 1e-3
                or int(row["estimate_timestamp_ns"]) != int(row["image_timestamp_ns"])
            ):
                invalid.append(index)
    original_numeric_valid = all(
        row["status"] != "ok" or all(math.isfinite(float(row[field])) for field in fields) for row in original
    )
    if exact and original_numeric_valid:
        for before, after in zip(original, replay):
            status_changes += before["status"] != after["status"]
            if before["status"] == after["status"] == "ok":
                a = [float(before[field]) for field in fields]
                b = [float(after[field]) for field in fields]
                translation.append(math.sqrt(sum((x - y) ** 2 for x, y in zip(a[:3], b[:3]))))
                norm = math.sqrt(sum(x * x for x in a[3:]) * sum(x * x for x in b[3:]))
                cosine = abs(sum(x * y for x, y in zip(a[3:], b[3:]))) / norm if norm else 0
                rotation.append(math.degrees(2 * math.acos(min(1.0, cosine))))
    result = {
        "pass": exact and source_clean and writer_clean and producer_clean and not invalid and original_numeric_valid,
        "exact_track_sequence": exact,
        "original_attempts": len(original),
        "replay_attempts": len(replay),
        "original_ok": sum(row["status"] == "ok" for row in original),
        "replay_ok": sum(row["status"] == "ok" for row in replay),
        "invalid_rows": invalid[:20],
        "status_changes": status_changes if exact else None,
        "raw_translation_difference_m": distribution(translation),
        "raw_rotation_difference_deg": distribution(rotation),
        "pose_comparison_note": (
            "Raw differences are not ground-truth accuracy; "
            "exact_pose_values separately checks serialized pose equality."
        ),
        "writer": summary,
        "producer": producer,
    }
    pose_fields = ("status", "estimate_timestamp_ns") + fields
    exact_poses = exact and all(all(a[k] == b[k] for k in pose_fields) for a, b in zip(original, replay))
    differences = (
        [
            {
                "source_frame_index": int(a["source_frame_index"]),
                "image_timestamp_ns": int(a["image_timestamp_ns"]),
                "fields": {key: {"recorded": a[key], "replay": b[key]} for key in pose_fields if a[key] != b[key]},
            }
            for a, b in zip(original, replay)
            if any(a[key] != b[key] for key in pose_fields)
        ]
        if exact
        else []
    )
    result["pose_difference_frames"] = len(differences) if exact else None
    result["first_pose_difference"] = differences[0] if differences else None
    contract_path = output / "replay_contract.json"
    strict = contract_path.exists() and json.loads(contract_path.read_text()).get("deterministic_sba") is True
    if strict:
        import importlib.util

        spec = importlib.util.spec_from_file_location(
            "sba_replay_contract", Path(__file__).with_name("sba_replay_contract.py")
        )
        validator = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(validator)
        result["sba_journal"] = validator.validate_session(source)
        result["sba_execution"] = check_sba_execution(output, result["sba_journal"])
    result["strict_sba_replay"] = strict
    result["exact_pose_values"] = exact_poses
    result["deterministic_replay_pass"] = bool(
        strict and result["pass"] and exact_poses and result["sba_execution"]["verified"]
    )
    result["pass_note"] = (
        "pass means complete valid frame replay; pose differences are diagnostic, not execution failures"
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--require-exact", action="store_true", help="Fail unless every pose and the original SBA execution match"
    )
    args = parser.parse_args()
    try:
        result = check(args.session, args.output)
    except (OSError, ValueError, KeyError, TypeError, RuntimeError) as error:
        result = {"pass": False, "error": str(error)}
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result.get("deterministic_replay_pass" if args.require_exact else "pass", False) else 1


if __name__ == "__main__":
    raise SystemExit(main())
