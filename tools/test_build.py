#!/usr/bin/env python3
"""Exercise the unmodified SDK custom entry with recording CMake/colcon tools."""

import json
import os
import shutil
import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path

SOURCE = Path(__file__).resolve().parents[1]
SDK = SOURCE.parents[3]


class OriginalSdkBuildTest(unittest.TestCase):
    """Check dispatch order, SDK lock ownership, and error propagation."""

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="spacemit-original-sdk-")
        self.addCleanup(self.temp.cleanup)
        self.sdk = Path(self.temp.name)
        shutil.copytree(
            SDK / "build", self.sdk / "build",
            ignore=shutil.ignore_patterns(".git", "__pycache__", ".pytest_cache"),
        )
        self.project = self.sdk / "middleware/ros2/slam/spacemit-vslam"
        (self.project / "tools").mkdir(parents=True)
        (self.project / "dependencies").mkdir()
        for name in ("build.py",):
            shutil.copy2(SOURCE / "tools" / name, self.project / "tools" / name)
        shutil.copy2(SOURCE / "build.sh", self.project / "build.sh")
        shutil.copy2(
            SOURCE / "dependencies/dependencies.json",
            self.project / "dependencies/dependencies.json",
        )
        manifest = json.loads((SOURCE / "dependencies/dependencies.json").read_text())
        for dep in manifest["dependencies"].values():
            (self.sdk / dep["path"]).mkdir(parents=True, exist_ok=True)
        (self.sdk / "components/peripherals/imu").mkdir(parents=True)
        package_groups = (
            ("modules", ("camera", "mapping", "vslam")),
            ("src", ("common", "camera_io", "vio", "las2", "esdf_drm", "bringup")),
        )
        for directory, packages in package_groups:
            for name in packages:
                path = self.project / directory / name
                path.mkdir(parents=True, exist_ok=True)
                (path / "package.xml").write_text(f"<package><name>{name}</name></package>\n")
        self.ros = self.sdk / "ros-setup.bash"
        self.ros.write_text(":\n")
        self.record = self.sdk / "commands.jsonl"
        self.output = self.sdk / "isolated-output"
        tools = self.sdk / "fake-tools"
        tools.mkdir()
        recorder = f"#!{sys.executable}\n" + textwrap.dedent(r'''
            import fcntl
            import json
            import os
            import sys
            from pathlib import Path

            locked = False
            with open(os.environ["TEST_LOCK"], "a") as handle:
                try:
                    fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    locked = True
            record = {
                "tool": Path(sys.argv[0]).name,
                "args": sys.argv[1:],
                "sdk_lock_held": locked,
            }
            with open(os.environ["TEST_RECORD"], "a") as handle:
                handle.write(json.dumps(record) + "\n")
            if (
                "-S" in sys.argv
                and os.environ.get("TEST_FAIL_SOURCE")
                and os.environ["TEST_FAIL_SOURCE"] in sys.argv[sys.argv.index("-S") + 1]
            ):
                sys.exit(17)
        ''')
        for name in ("cmake", "colcon"):
            tool = tools / name
            tool.write_text(recorder)
            tool.chmod(0o755)
        self.env = dict(
            os.environ,
            PATH=str(tools) + os.pathsep + os.environ["PATH"],
            SROBOTIS_SKIP_DEPS_CHECK="1",
            SROBOTIS_USE_DOCKER_BUILD="0",
            TEST_LOCK=str(self.output / ".install.lock"),
            TEST_RECORD=str(self.record),
        )
        for name in (
            "BUILD_TARGET", "BUILD_TARGET_FILE", "BUILD_CONFIG_FILE", "PREFIX",
            "SROBOTIS_CROSS_BUILD", "SROBOTIS_CMAKE_EXTRA_ARGS",
        ):
            self.env.pop(name, None)

    def invoke(self, *extra):
        """Invoke the project CLI against the isolated SDK fixture."""
        return subprocess.run(
            [
                sys.executable, str(self.project / "tools/build.py"),
                "--output", str(self.output), "--jobs", "2",
                "--ros-setup", str(self.ros), *extra,
            ],
            env=self.env, capture_output=True, text=True, timeout=30,
        )

    def commands(self):
        """Return the fixture's ordered CMake and colcon invocation records."""
        return [json.loads(line) for line in self.record.read_text().splitlines()]

    def test_original_sdk_dispatch_orders_native_then_exact_ros_sources(self):
        result = self.invoke(
            "--cmake-arg=-DONNXRUNTIME_ROOT=/fixture/runtime",
            "--cmake-arg=-DSTEREO_MATCHING_EXTERNAL_INPUT_MODEL=/fixture/model.onnx",
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        commands = self.commands()
        self.assertEqual(len(commands), 19)
        self.assertTrue(all(item["sdk_lock_held"] for item in commands))
        configured = [
            item["args"][1] for item in commands
            if item["tool"] == "cmake" and item["args"][0] == "-S"
        ]
        self.assertEqual(
            configured,
            [str(path) for path in (
                self.sdk / "components/multimedia/mpp", self.project / "modules/camera",
                self.project / "modules/mapping",
                self.sdk / "components/perception/stereo_matching",
                self.project / "modules/vslam", self.sdk / "components/peripherals/imu",
            )],
        )
        for item in commands:
            if item["tool"] == "cmake" and item["args"][0] == "-S":
                self.assertIn("-DONNXRUNTIME_ROOT=/fixture/runtime", item["args"])
                self.assertIn(f"-DCMAKE_INSTALL_PREFIX={self.output}/staging", item["args"])
        ros = commands[-1]
        self.assertEqual(ros["tool"], "colcon")
        args = ros["args"]
        start, end = args.index("--base-paths") + 1, args.index("--packages-select")
        self.assertEqual(
            args[start:end],
            [
                str(self.project / "src"),
                str(self.sdk / "middleware/ros2/peripherals/imu"),
                str(self.sdk / "components/multimedia/mpp/test/ros2_mpp_msgs"),
            ],
        )
        self.assertNotIn(str(self.project / "modules"), args)

    def test_native_failure_stops_before_ros(self):
        self.env["TEST_FAIL_SOURCE"] = "modules/camera"
        result = self.invoke()
        self.assertEqual(result.returncode, 17, result.stdout + result.stderr)
        self.assertEqual(len(self.commands()), 4)
        self.assertTrue(all(item["tool"] == "cmake" for item in self.commands()))

    def test_unvalidated_cross_build_is_explicitly_rejected(self):
        result = self.invoke("--cross")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("not validated", result.stderr)
        self.assertFalse(self.record.exists())


if __name__ == "__main__":
    unittest.main()
