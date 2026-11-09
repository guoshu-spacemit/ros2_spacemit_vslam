#!/usr/bin/env python3
"""Check native exposure/IMU clock extension without MPP or camera hardware."""

from pathlib import Path
import subprocess
import tempfile
import unittest


CAMERA = Path(__file__).resolve().parents[1] / "modules/camera"


class CameraTimestampTest(unittest.TestCase):
    """Exercise the production timestamp helper across clock boundaries."""

    def test_exposure_and_imu_share_a_safe_extended_epoch(self):
        source = r'''
#include "timestamp_unwrap.h"
#include "imu_boundary_deduplication.h"
#include "camera_sdk/camera_clock.h"

#include <cassert>
#include <limits>

int main()
{
    using camera_sdk::EmbeddedMetadata;
    using camera_sdk::detail::unwrap_metadata_timestamps;
    using camera_sdk::detail::unwrap_near;
    constexpr std::uint64_t wrap = std::uint64_t{1} << 32;

    EmbeddedMetadata ordinary;
    ordinary.exposure_end_raw_us = 10000;
    ordinary.exposure_start_raw_us = 9000;
    ordinary.imu_count = 1;
    ordinary.imu[0].timestamp_raw_us = 9500;
    const auto ordinary_end = unwrap_metadata_timestamps(ordinary, 0, false);
    assert(ordinary_end == wrap + 10000);
    assert(ordinary.exposure_start_us == wrap + 9000);
    assert(ordinary.imu[0].timestamp_us == wrap + 9500);

    EmbeddedMetadata crossing;
    crossing.exposure_end_raw_us = 0x100;
    crossing.exposure_start_raw_us = 0xffffff00;
    crossing.imu_count = 3;
    crossing.imu[0].timestamp_raw_us = 0xfffffe00;
    crossing.imu[1].timestamp_raw_us = 0xffffff80;
    crossing.imu[2].timestamp_raw_us = 0x80;
    for (auto& sample : crossing.imu)
        sample.valid = true;
    const auto crossing_end = unwrap_metadata_timestamps(crossing, 0, false);
    assert(crossing_end == wrap + 0x100);
    assert(crossing.exposure_start_us == wrap - 0x100);
    assert(crossing.exposure_end_us - crossing.exposure_start_us == 0x200);
    assert(crossing.imu[0].timestamp_us == wrap - 0x200);
    assert(crossing.imu[1].timestamp_us == wrap - 0x80);
    assert(crossing.imu[2].timestamp_us == wrap + 0x80);

    // Raw metadata remains unchanged, so ROS clock mapping keeps its own epoch.
    camera_sdk::CameraClock clock;
    camera_sdk::CameraClock::FrameTimestamps mapped;
    assert(clock.preview(crossing.exposure_start_raw_us,
        crossing.exposure_end_raw_us, 1000000000, &mapped));
    assert(mapped.exposure_end_ns - mapped.exposure_start_ns == 0x200 * 1000);

    camera_sdk::detail::ImuBoundaryDeduplicationState boundary;
    camera_sdk::detail::remove_repeated_boundary_imu(crossing, boundary);
    EmbeddedMetadata next;
    next.exposure_start_raw_us = 0x180;
    next.exposure_end_raw_us = 0x200;
    next.imu_count = 2;
    next.imu[0].timestamp_raw_us = 0x80;
    next.imu[1].timestamp_raw_us = 0x180;
    next.imu[0].valid = next.imu[1].valid = true;
    assert(unwrap_metadata_timestamps(next, crossing_end, true) == wrap + 0x200);
    camera_sdk::detail::remove_repeated_boundary_imu(next, boundary);
    assert(next.imu_count == 1);
    assert(next.imu[0].timestamp_us == wrap + 0x180);

    EmbeddedMetadata before_wrap;
    before_wrap.exposure_start_raw_us = 0xfffffe00;
    before_wrap.exposure_end_raw_us = 0xffffff00;
    const auto before_end = unwrap_metadata_timestamps(before_wrap, 0, false);
    assert(before_end == 2 * wrap - 0x100);
    assert(unwrap_metadata_timestamps(crossing, before_end, true) ==
        2 * wrap + 0x100);
    // A nearby older sample maps to the prior cycle, not a far-future value.
    assert(unwrap_near(2 * wrap + 0x100, 0xfffffe00) == 2 * wrap - 0x200);
    assert(unwrap_near(2 * wrap + 0x100, 0x100) == 2 * wrap + 0x100);

    bool rejected = false;
    try
    {
        unwrap_near(0x100, 0xffffff00);
    }
    catch (const std::overflow_error&)
    {
        rejected = true;
    }
    assert(rejected);
    rejected = false;
    try
    {
        unwrap_near(std::numeric_limits<std::uint64_t>::max() - 0xff, 0x100);
    }
    catch (const std::overflow_error&)
    {
        rejected = true;
    }
    assert(rejected);
}
'''
        with tempfile.TemporaryDirectory(prefix="spacemit-camera-clock-") as temp:
            root = Path(temp)
            cpp = root / "test.cpp"
            binary = root / "test-camera-timestamps"
            cpp.write_text(source)
            result = subprocess.run(
                ["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                 "-fsanitize=undefined", "-fno-sanitize-recover=all",
                 "-I", str(CAMERA / "include"), "-I", str(CAMERA / "src"),
                 str(cpp), str(CAMERA / "src/camera_clock.cpp"),
                 str(CAMERA / "src/imu_boundary_deduplication.cpp"),
                 "-o", str(binary)], capture_output=True, text=True,
                timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([str(binary)], capture_output=True,
                                    text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
