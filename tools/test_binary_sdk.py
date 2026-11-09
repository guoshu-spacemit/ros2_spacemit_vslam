#!/usr/bin/env python3
"""Fault-inject binary SDK metadata checks without loading a target library.

Small ELF header fixtures exercise packaging validation only. These tests do
not validate RISC-V machine instructions, ABI linking or hardware behavior.
"""

import hashlib
import json
from pathlib import Path
import platform
import subprocess
import tempfile
import unittest


HELPER = Path(__file__).resolve().with_name("binary_sdk.cmake")


class BinarySdkMetadataTest(unittest.TestCase):
    """Exercise the actual installer against malformed release artifacts."""

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="spacemit-binary-sdk-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root / "source"
        self.source.mkdir()
        self.payload = self.source / "sdk/riscv64"
        files = {
            "include/spacemit_fixture/api.h": b"struct Fixture { int value; };\n",
            "lib/cmake/spacemit_fixture/spacemit_fixtureConfig.cmake":
                b"set(spacemit_fixture_FOUND TRUE)\n",
            "lib/cmake/spacemit_fixture/spacemit_fixtureTargets.cmake":
                b"# Packaging fixture; no compiled consumer claim.\n",
            "share/licenses/spacemit_fixture/LICENSE": b"Fixture license\n",
        }
        header = bytearray(64)
        header[0:7] = b"\x7fELF\x02\x01\x01"
        header[16:18] = (3).to_bytes(2, "little")
        header[18:20] = (243).to_bytes(2, "little")
        header[20:24] = (1).to_bytes(4, "little")
        header[48:52] = (5).to_bytes(4, "little")
        files["lib/libspacemit_fixture.so.1"] = header
        for name, content in files.items():
            file = self.payload / name
            file.parent.mkdir(parents=True, exist_ok=True)
            file.write_bytes(content)
        (self.payload / "lib/libspacemit_fixture.so").symlink_to(
            "libspacemit_fixture.so.1")
        self.manifest = {
            "schema_version": 1,
            "package": "spacemit_fixture",
            "version": "1.2.3",
            "target": {
                "system": "Linux", "processor": "riscv64", "elf_class": 64,
                "endianness": "little", "isa": "rv64gcv", "abi": "lp64d",
            },
            "libraries": ["lib/libspacemit_fixture.so", "lib/libspacemit_fixture.so.1"],
        }
        self.refresh_manifest()
        (self.source / "CMakeLists.txt").write_text(
            "cmake_minimum_required(VERSION 3.19)\n"
            "project(spacemit_fixture VERSION 1.2.3 LANGUAGES NONE)\n"
            f'include("{HELPER}")\n'
            'spacemit_install_binary_sdk(spacemit_fixture "sdk/riscv64")\n')
        self.toolchain = self.root / "riscv64.cmake"
        self.toolchain.write_text(
            "set(CMAKE_SYSTEM_NAME Linux)\n"
            "set(CMAKE_SYSTEM_PROCESSOR riscv64)\n")

    def refresh_manifest(self):
        """Update hashes after an intentional artifact mutation."""
        self.manifest["files"] = {
            str(file.relative_to(self.payload)): hashlib.sha256(file.read_bytes()).hexdigest()
            for file in self.payload.rglob("*")
            if file.is_file() and file.name != "manifest.json"
        }
        (self.payload / "manifest.json").write_text(
            json.dumps(self.manifest, indent=2) + "\n")

    def configure(self, toolchain=True):
        """Configure the real installer with the selected target metadata."""
        command = ["cmake", "-S", str(self.source), "-B", str(self.root / "build"),
                   f"-DCMAKE_INSTALL_PREFIX={self.root}/prefix"]
        if toolchain:
            command.append(f"-DCMAKE_TOOLCHAIN_FILE={self.toolchain}")
        return subprocess.run(command, text=True, capture_output=True, timeout=30)

    def reject(self, diagnostic):
        result = self.configure()
        output = result.stdout + result.stderr
        self.assertNotEqual(result.returncode, 0, output)
        self.assertIn(diagnostic, " ".join(output.split()))

    def test_target_riscv64_can_install_relocatable_payload(self):
        result = self.configure()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        subprocess.run(["cmake", "--install", str(self.root / "build")],
                       text=True, capture_output=True, check=True, timeout=30)
        prefix = self.root / "prefix"
        self.assertTrue((prefix / "lib/libspacemit_fixture.so").is_symlink())
        self.assertEqual((prefix / "lib/libspacemit_fixture.so").readlink(),
                         Path("libspacemit_fixture.so.1"))
        self.assertTrue((prefix / "share/spacemit_fixture/binary_sdk/manifest.json").is_file())
        self.assertFalse((prefix / "manifest.json").exists())

    @unittest.skipIf(platform.machine().lower().startswith("riscv64"),
                     "native host is already a supported target")
    def test_native_non_riscv_target_is_rejected(self):
        result = self.configure(toolchain=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("contains Linux/riscv64 prebuilt libraries", result.stderr)

    def test_changed_public_header_is_rejected(self):
        (self.payload / "include/spacemit_fixture/api.h").write_text("changed\n")
        self.reject("SHA256 mismatch")

    def test_unlisted_file_is_rejected(self):
        (self.payload / "include/unlisted.cpp").write_text("unexpected implementation\n")
        self.reject("payload and manifest file inventory differ")

    def test_wrong_machine_is_rejected_even_with_valid_checksum(self):
        file = self.payload / "lib/libspacemit_fixture.so.1"
        header = bytearray(file.read_bytes())
        header[18:20] = (62).to_bytes(2, "little")
        file.write_bytes(header)
        self.refresh_manifest()
        self.reject("not a Linux RISC-V ELF64 little-endian shared object")

    def test_wrong_float_abi_is_rejected_even_with_valid_checksum(self):
        file = self.payload / "lib/libspacemit_fixture.so.1"
        header = bytearray(file.read_bytes())
        header[48:52] = (1).to_bytes(4, "little")
        file.write_bytes(header)
        self.refresh_manifest()
        self.reject("does not use the declared LP64D floating-point ABI")

    def test_declared_architecture_mismatch_is_rejected(self):
        self.manifest["target"]["processor"] = "x86_64"
        self.refresh_manifest()
        self.reject("expected a declared Linux/riscv64")

    def test_absolute_symlink_is_rejected(self):
        link = self.payload / "lib/libspacemit_fixture.so"
        link.unlink()
        link.symlink_to(self.payload / "lib/libspacemit_fixture.so.1")
        self.refresh_manifest()
        self.reject("binary SDK symlinks must be relative")

    def test_symlink_escaping_payload_is_rejected(self):
        outside = self.root / "outside.txt"
        outside.write_text("external artifact\n")
        (self.payload / "share/outside.txt").symlink_to("../../../../outside.txt")
        self.refresh_manifest()
        self.reject("binary SDK symlink escapes its payload")

    def test_directory_symlink_is_rejected(self):
        outside = self.root / "outside"
        outside.mkdir()
        (self.payload / "share/linked-directory").symlink_to(outside)
        self.reject("directory symlinks are not supported")


if __name__ == "__main__":
    unittest.main()
