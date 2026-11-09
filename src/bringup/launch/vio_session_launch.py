"""Reuse the reconstruction graph and add optional original-stream recording."""

import hashlib
import json
import os
from pathlib import Path
import runpy

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


def validate_options(options, launch_file):
    """Evaluate the real graph without executing nodes or opening devices."""
    module = runpy.run_path(str(launch_file))
    description = module["generate_launch_description"]()
    declarations = {item.name for item in description.entities
                    if isinstance(item, DeclareLaunchArgument)}
    unknown = set(options) - declarations
    if unknown:
        raise ValueError(f"Unknown reconstruction arguments: {sorted(unknown)}")
    context = LaunchContext()
    context.launch_configurations.update(options)
    for item in description.entities:
        if isinstance(item, DeclareLaunchArgument):
            if item.condition is None or item.condition.evaluate(context):
                item.execute(context)
    # The real graph derives node conditions/prefixes through this pure config
    # function before any Node executes. Reuse it rather than inventing defaults
    # or evaluating derived launch state before it exists.
    for action in module["configure_execution"](context):
        action.execute(context)
    vio = None
    for item in description.entities:
        if isinstance(item, Node):
            if item.condition is not None and not item.condition.evaluate(context):
                continue
            values = evaluate_parameters(context, item._Node__parameters)[0]
            if "imu_frequency_hz" in values:
                vio = values
                if type(values["imu_frequency_hz"]) is not float:
                    raise TypeError("imu_frequency_hz must be decimal (e.g. 0.0)")
            if "render_fps" in values and type(values["render_fps"]) is not float:
                raise TypeError("render_fps must be decimal (e.g. 60.0)")
    return vio


def generate_launch_description():
    directory = Path(os.environ["SPACEMIT_ROBOT_VIO_SESSION_DIR"])
    session = json.loads((directory / "session.json").read_text())
    options = json.loads((directory / "launch-options.json").read_text())
    main_launch = (Path(get_package_share_directory("bringup")) /
                   "launch/vio_las2_esdf_launch.py")
    parameters = validate_options(options, main_launch)
    journal = bool(session.get("sba_journal", False))
    if journal and (not session["record"] or session["slam"]
                    or parameters["use_imu"] is not True
                    or parameters["async_sba"] is not True
                    or parameters["enable_sba"] is not True
                    or parameters["sba_journal_record_file"] != str(directory / "sba_events.txt")):
        raise ValueError("SBA journal requires recorded asynchronous VIO without SLAM")
    (directory / "vio_parameters.yaml").write_text(yaml.safe_dump(parameters))
    if journal:
        inputs = ("camera.json", "calibration.yaml", "vio_parameters.yaml")
        (directory / "replay_contract.json").write_text(json.dumps({
            "version": 1, "sba_journal": "sba_events.txt",
            "deterministic_sba": True,
            "input_sha256": {name: hashlib.sha256((directory / name).read_bytes()).hexdigest()
                             for name in inputs},
        }, indent=2) + "\n")
    actions = []
    if session.get("agc", False):
        actions.append(Node(
            package="camera_io", executable="camera_agc_node", name="camera_agc",
            output="screen", parameters=[{
                **session["agc_parameters"], "log_file": str(directory / "agc.csv"),
            }],
        ))
    if session["record"]:
        actions.append(Node(
            package="camera_io", executable="recorder_node", name="recorder",
            output="screen", parameters=[{
                **session["recorder_parameters"],
                "config_file": str(directory / "recorder.json"),
                "session_directory": str(directory),
            }],
        ))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(main_launch)), launch_arguments=options.items(),
    ))
    return LaunchDescription(actions)
