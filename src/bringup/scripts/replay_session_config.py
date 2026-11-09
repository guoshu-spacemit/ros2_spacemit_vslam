#!/usr/bin/env python3
"""Prepare a detached replay session without opening camera/DRM devices."""

import argparse
import json
from pathlib import Path
import subprocess
import signal

import yaml

from vio_session_config import resource
from calibration_bundle import verify_snapshot


def replay_capability(source, mode="auto"):
    """Cheap metadata eligibility, not full journal or numerical validation."""
    source = Path(source)
    try:
        parameters = yaml.safe_load((source / "vio_parameters.yaml").read_text())
    except yaml.YAMLError as error:
        raise ValueError(f"Invalid vio_parameters.yaml: {error}") from error
    if not isinstance(parameters, dict) or type(parameters.get("use_imu", True)) is not bool:
        raise ValueError("vio_parameters.yaml must contain a boolean use_imu")
    recorded_imu = parameters.get("use_imu", True)
    imu = recorded_imu if mode == "auto" else mode == "vio"
    missing = [name for name in ("sba_events.txt", "replay_contract.json",
                                "camera0_writer_summary.json")
               if not (source / name).is_file()]
    reasons = []
    if missing:
        reasons.append("缺少 " + ", ".join(missing))
    if not imu or imu != recorded_imu:
        reasons.append("严格 SBA 回放必须保持录制时的 VIO 模式")
    return {"imu": imu, "unchanged": imu == recorded_imu, "missing": missing,
            "exact_available": not reasons, "reason": "; ".join(reasons)}


def select_policy(source, mode, exact):
    if mode not in ("auto", "vo", "vio") or exact not in ("auto", "on", "off"):
        raise ValueError("Invalid replay mode/policy")
    verify_snapshot(source)
    capability = replay_capability(source, mode)
    journal = all((Path(source) / name).is_file() for name in
                  ("sba_events.txt", "replay_contract.json"))
    strict = exact == "on" or (exact == "auto" and journal
                              and capability["imu"] and capability["unchanged"])
    if strict and not capability["exact_available"]:
        raise ValueError("Exact replay requires the original VIO mode and SBA journal: "
                         + capability["reason"] + "; use --exact off for ordinary comparison")
    return strict, capability["imu"]


def prepare(source, directory, profile, *, mode="auto", mapping=False,
            paused=False, exact="auto", features=True, domain=0):
    source = Path(source).resolve(strict=True)
    directory = Path(directory).resolve()
    if not 0 <= domain <= 232 or mode not in ("auto", "vo", "vio"):
        raise ValueError("Invalid domain or odometry mode")
    if exact not in ("auto", "on", "off"):
        raise ValueError("exact must be auto, on or off")
    strict, imu = select_policy(source, mode, exact)
    calibration_verified = verify_snapshot(source)
    if not calibration_verified:
        print("WARNING: legacy recording has no calibration-manifest.json; "
              "using its recorded snapshots without integrity assurance.")
    model = ""
    if mapping:
        profile = Path(profile).resolve(strict=True)
        config = json.loads(profile.read_text())
        model = str(resource(profile.parent, config.get("model_path"), "model_path"))
    options = {
        "enable_screen": "true", "mapping_enabled": str(mapping).lower(),
        "model_path": model, "record_screen": "false",
        "strict_sba_replay": str(strict).lower(),
        "replay_realtime": "true", "replay_start_paused": str(paused).lower(),
        "feature_panel_enabled": str(features).lower(),
        "publish_frame_diagnostics": "true", "replay_timeout_sec": "120.0",
        "odometry_mode": "recorded" if mode == "auto" else mode,
    }
    directory.mkdir(parents=True, exist_ok=False)
    session = {"source": "step" if paused else "replay", "input": str(source),
               "mode": "vio" if imu else "vo", "domain": domain,
               "mapping": mapping, "record": False, "sba_timeline": False,
               "exact_requested": strict,
               "calibration_verified": calibration_verified,
               "legacy_video_summary": not (source / "camera0_writer_summary.json").is_file(),
               "launch_arguments": options}
    (directory / "session.json").write_text(json.dumps(session, indent=2) + "\n")
    print(f"Replay input: {source}; mode={session['mode']}; exact required={strict}")
    if not strict:
        print("Same recorded frames/IMU; original live poses are NOT guaranteed.")


def run(directory):
    directory = Path(directory)
    session = json.loads((directory / "session.json").read_text())
    # Recheck immediately before launch; source files may change after prepare.
    verified = verify_snapshot(session["input"])
    if session.get("calibration_verified") and not verified:
        raise ValueError("Calibration manifest disappeared after replay preparation")
    command = ["bash", str(Path(__file__).with_name("run_vio_replay.sh")),
               session["input"], str(directory / "result")]
    command += [f"{k}:={v}" for k, v in session["launch_arguments"].items() if v != ""]
    status_path = directory / "pipeline-status.json"
    status_path.write_text(json.dumps({"state": "running", "readiness": "not_observed"}) + "\n")
    process = subprocess.Popen(command)
    interrupted = False

    def stop(*_):
        nonlocal interrupted
        # systemd/terminal sends SIGINT to the whole process group. The child
        # launch/recorder drains itself; keep the supervisor alive to reap it.
        interrupted = True

    previous = signal.signal(signal.SIGINT, stop)
    try:
        returncode = process.wait()
    finally:
        signal.signal(signal.SIGINT, previous)
    status_path.write_text(json.dumps({
        "state": "interrupted" if interrupted else ("completed" if returncode == 0 else "failed"),
        "returncode": returncode,
        "result_directory": str(directory / "result"),
    }, indent=2) + "\n")
    return 0 if interrupted else returncode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path, nargs="?")
    parser.add_argument("--policy-only", action="store_true")
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--source", type=Path)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--mode", choices=("auto", "vo", "vio"), default="auto")
    parser.add_argument("--mapping", choices=("on", "off"), default="off")
    parser.add_argument("--paused", action="store_true")
    parser.add_argument("--exact", choices=("auto", "on", "off"), default="auto")
    parser.add_argument("--features", choices=("on", "off"), default="on")
    parser.add_argument("--domain", type=int, default=0)
    args = parser.parse_args()
    if args.policy_only:
        if args.source is None:
            parser.error("--source is required")
        try:
            print(str(select_policy(args.source, args.mode, args.exact)[0]).lower())
        except (OSError, ValueError, TypeError) as error:
            parser.exit(1, f"ERROR: {error}\n")
        return 0
    if args.directory is None:
        parser.error("directory is required")
    if args.run:
        return run(args.directory)
    if args.source is None:
        parser.error("--source is required")
    prepare(args.source, args.directory, args.profile, mode=args.mode,
            mapping=args.mapping == "on", paused=args.paused, exact=args.exact,
            features=args.features == "on", domain=args.domain)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
