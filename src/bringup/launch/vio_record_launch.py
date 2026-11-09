"""Camera + LAS2 + VIO + original MJPEG recording in one fresh session."""
import json
import hashlib
from pathlib import Path
import shutil

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    session = Path(LaunchConfiguration("session_directory").perform(context)).resolve()
    session.mkdir(parents=True, exist_ok=False)
    camera_path = Path(LaunchConfiguration("camera_config_file").perform(context))
    camera = json.loads(camera_path.read_text())
    compressed = camera.get("streams", {}).get("compressed", {})
    if not compressed.get("enabled", False):
        raise RuntimeError("Camera config must enable streams.compressed")
    if camera.get("timestamp", {}).get("source") not in (
            "embedded_exposure_start", "embedded_exposure_midpoint"):
        raise RuntimeError("Stereo VIO recording requires embedded exposure timestamps")
    camera_fps = int(LaunchConfiguration("camera_fps", default="0").perform(context))
    if camera_fps < 0:
        raise RuntimeError("camera_fps must be zero (JSON) or positive")
    if camera_fps:
        camera["camera"]["fps"] = camera_fps
    (session / "camera.json").write_text(json.dumps(camera, indent=2) + "\n")
    calibration = Path(LaunchConfiguration("calibration_file").perform(context))
    shutil.copyfile(calibration, session / "calibration.yaml")
    imu = camera.get("imu", {})
    imu_enabled = imu.get("enabled", False)
    if type(imu_enabled) is not bool:
        raise RuntimeError("imu.enabled must be boolean")
    imu_topic = imu.get("raw", {}).get("ros_topic", "")
    if imu_enabled and (not isinstance(imu_topic, str) or not imu_topic.strip()):
        raise RuntimeError("IMU recording requires imu.raw.ros_topic")
    config = {
        "output_dir": str(session.parent),
        "cameras": [{"name": "camera0", "topic": compressed["topic"], "codec": "mjpeg"}],
        # IMU callbacks can extend beyond the video recording boundary.
        # Preserve those real samples independently of embedded video metadata.
        "imus": [{"name": "camera0", "topic": imu_topic, "enabled": True,
                  "queue_depth": 2048}] if imu_enabled else [],
    }
    config_path = session / "recorder.json"
    config_path.write_text(json.dumps(config, indent=2) + "\n")
    enabled = LaunchConfiguration("record_vio").perform(context).lower() == "true"
    bool_keys = ("use_imu", "enable_external_propagation", "enable_sba",
                 "async_sba", "enable_imu_sba_initialization", "slam_enabled",
                 "enable_stationary_initialization")
    defaults = {"use_imu": "true", "enable_external_propagation": "false",
                "enable_sba": "true", "async_sba": "true",
                "enable_imu_sba_initialization": "false", "slam_enabled": "false",
                "enable_stationary_initialization": "false",
                "imu_frequency_hz": "0.0", "pnp_min_depth_m": "0.0",
                "image_width": "640", "image_height": "400",
                "image_queue_depth": "1", "sync_queue_depth": "2",
                "slam_max_map_size": "300", "slam_loop_closure_throttling_ms": "1000"}
    parameters = {}
    for key, default in defaults.items():
        value = LaunchConfiguration(key, default=default).perform(context)
        parameters[key] = (value.lower() == "true" if key in bool_keys else
                           float(value) if key in ("imu_frequency_hz", "pnp_min_depth_m") else int(value))
    screen_enabled = LaunchConfiguration("enable_screen").perform(context).lower() == "true"
    panel_enabled = LaunchConfiguration("feature_panel_enabled").perform(context).lower() == "true"
    parameters["publish_frame_diagnostics"] = screen_enabled and panel_enabled
    (session / "vio_parameters.yaml").write_text(yaml.safe_dump(parameters))
    journal = enabled and LaunchConfiguration("record_sba_events").perform(context).lower() == "true"
    if journal and (not parameters["use_imu"] or not parameters["async_sba"] or parameters["slam_enabled"]):
        raise RuntimeError("SBA event recording requires async VIO without SLAM; disable record_sba_events otherwise")
    (session / "replay_contract.json").write_text(json.dumps({
        "version": 1, "sba_journal": "sba_events.txt" if journal else None,
        "deterministic_sba": journal,
        "input_sha256": {name: hashlib.sha256((session / name).read_bytes()).hexdigest()
                         for name in ("camera.json", "calibration.yaml", "vio_parameters.yaml")},
    }, indent=2) + "\n")
    bringup = get_package_share_directory("bringup")
    return [
        RegisterEventHandler(OnProcessExit(on_exit=[
            EmitEvent(event=Shutdown(reason="Recording pipeline child exited"))])),
        Node(package="camera_io", executable="recorder_node",
             name="recorder", output="screen",
             parameters=[{"config_file": str(config_path), "session_directory": str(session),
                          "camera_write_queue_depth": LaunchConfiguration("camera_write_queue_depth"),
                          "camera_write_buffer_bytes": LaunchConfiguration("camera_write_buffer_bytes")}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(bringup) / "launch/vio_las2_esdf_launch.py")),
            launch_arguments={
                "model_path": LaunchConfiguration("model_path"),
                "vio_frame_record_file": str(session / "vio_frames.csv") if enabled else "",
                "sba_journal_record_file": str(session / "sba_events.txt") if journal else "",
                "enable_mapping": LaunchConfiguration("enable_screen"),
                "enable_gles_drm": LaunchConfiguration("enable_screen"),
                "feature_panel_enabled": LaunchConfiguration("feature_panel_enabled"),
                "publish_frame_diagnostics": "true" if parameters["publish_frame_diagnostics"] else "false",
                "enable_rtsp": "false", "enable_esdf_rtsp": "false",
                "publish_las2_preview": LaunchConfiguration("enable_screen"), "record_enabled": "false",
            }.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("session_directory", description="New, non-existing session directory"),
        DeclareLaunchArgument("camera_config_file", default_value=str(
            Path(get_package_share_directory("camera_io")) / "config/usb_stereo_4000x1200.json")),
        DeclareLaunchArgument("camera_fps", default_value="0"),
        DeclareLaunchArgument("pnp_min_depth_m", default_value="0.0"),
        DeclareLaunchArgument("enable_stationary_initialization", default_value="false",
                              choices=["true", "false"]),
        DeclareLaunchArgument("calibration_file", default_value=str(
            Path(get_package_share_directory("camera_io")) / "config/0.yaml")),
        DeclareLaunchArgument("model_path"),
        DeclareLaunchArgument("camera_write_queue_depth", default_value="32"),
        DeclareLaunchArgument("camera_write_buffer_bytes", default_value="2097152"),
        DeclareLaunchArgument("enable_screen", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("feature_panel_enabled", default_value="true",
                              choices=["true", "false"]),
        DeclareLaunchArgument("record_vio", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("record_sba_events", default_value="true", choices=["true", "false"]),
        OpaqueFunction(function=setup),
    ])
