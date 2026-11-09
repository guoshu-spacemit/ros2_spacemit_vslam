"""Strict SBA journal validation, shared by launch and post-run checkers."""
import csv
from collections import Counter
import hashlib
import json
from pathlib import Path


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_session(session):
    session = Path(session)
    contract_path = session / "replay_contract.json"
    if not contract_path.is_file():
        raise RuntimeError("Recording has no SBA replay contract; re-record or explicitly use strict_sba_replay:=false")
    contract = json.loads(contract_path.read_text())
    if (contract.get("version") != 1 or contract.get("deterministic_sba") is not True
            or contract.get("sba_journal") != "sba_events.txt"):
        raise RuntimeError("Recording does not contain a supported deterministic SBA contract")
    for name, digest in contract.get("input_sha256", {}).items():
        if name not in ("camera.json", "calibration.yaml", "vio_parameters.yaml") or sha256(session / name) != digest:
            raise RuntimeError(f"SBA recording input fingerprint mismatch: {name}")
    with (session / "vio_frames.csv").open() as stream:
        track_rows = list(csv.DictReader(line for line in stream if not line.startswith("#")))
        targets = [int(row["image_timestamp_ns"]) for row in track_rows]
        required = {int(row["image_timestamp_ns"]) for row in track_rows
                    if row.get("status") == "ok"}
    with (session / "sba_events.txt").open() as stream:
        header = stream.readline().rstrip("\n")
        if header not in ("SPACEMIT_VSLAM_SBA_JOURNAL 1", "SPACEMIT_VSLAM_SBA_JOURNAL 2"):
            raise RuntimeError("Unsupported SBA event journal header")
        version = int(header[-1])
        stamps, tasks, active, commits, resets = [], 0, 0, 0, 0
        reset_boundaries = []
        ended = False
        for line in stream:
            fields = line.split()
            if fields and fields[0] == "END":
                expected = [len(stamps), tasks, active]
                if version == 2:
                    expected.append(resets)
                if len(fields) != len(expected) + 1 or list(map(int, fields[1:])) != expected:
                    raise RuntimeError("Invalid SBA journal footer")
                if stream.read():
                    raise RuntimeError("Trailing data after SBA journal footer")
                ended = True
                break
            if fields and fields[0] == "RESET":
                if (version != 2 or len(fields) != 4
                        or list(map(int, fields[1:])) != [len(stamps), tasks, active]):
                    raise RuntimeError("Invalid SBA journal reset boundary")
                active = 0
                resets += 1
                reset_boundaries.append(len(stamps))
                continue
            if len(fields) != 6:
                raise RuntimeError("Malformed SBA journal row")
            stamp, commit, result_hash, requested, launched, input_hash = map(int, fields)
            if (stamp <= (stamps[-1] if stamps else 0) or requested not in (0, 1)
                    or min(commit, result_hash, launched, input_hash) < 0):
                raise RuntimeError("Invalid SBA journal frame")
            if commit:
                if commit != active:
                    raise RuntimeError("SBA commit without matching active task")
                active = 0
                commits += 1
            elif result_hash:
                raise RuntimeError("SBA result hash without commit")
            if launched:
                if active or not requested or launched != tasks + 1:
                    raise RuntimeError("Invalid SBA task admission sequence")
                tasks, active = launched, launched
            elif input_hash:
                raise RuntimeError("SBA input hash without task")
            stamps.append(stamp)
    # A 2D tracking failure can reset odometry before the solver sees that
    # frame. Only such Track rows may be absent, at the exact RESET boundary.
    missing = []
    solver_index = 0
    for row in track_rows:
        stamp = int(row["image_timestamp_ns"])
        if solver_index < len(stamps) and stamp == stamps[solver_index]:
            solver_index += 1
        else:
            missing.append((solver_index, row.get("status", "")))
    boundary_counts = Counter(reset_boundaries)
    valid_missing = True
    for index, status in missing:
        if status not in ("invalid_pose", "track_error") or boundary_counts[index] == 0:
            valid_missing = False
            break
        boundary_counts[index] -= 1
    if (not ended or not targets or not stamps or solver_index != len(stamps)
            or (stamps != targets if version == 1 else not valid_missing
                or not required.issubset(stamps))):
        raise RuntimeError("Incomplete SBA journal or mismatch with recorded Track sequence")
    return {"frames": len(stamps), "tasks": tasks, "commits": commits,
            "resets": resets, "cancelled_tail_task": active}
