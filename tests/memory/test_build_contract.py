"""Exercise Make against a disposable copy; never touch production timestamps."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import re


class BuildContract(unittest.TestCase):
    def test_default_target_remains_host(self):
        root = Path(__file__).resolve().parents[2]
        query = subprocess.run(["make", "-qp"], cwd=root, text=True,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=30)
        # Inspect Make's executed database, not the source text. Query mode
        # does not invoke compilers, linkers or recursive build recipes.
        match = re.search(r"^\.DEFAULT_GOAL\s*:?=\s*(\S+)$", query.stdout, re.M)
        self.assertIsNotNone(match)
        self.assertEqual(match.group(1), "host")

    def test_header_rebuild_and_invalid_case(self):
        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory(prefix="salut-memory-deps-") as tmp:
            fixture = Path(tmp)
            shutil.copy2(root / "Makefile", fixture / "Makefile")
            for directory in ("src", "tests/memory", "host"):
                (fixture / directory).mkdir(parents=True, exist_ok=True)
            for source in [root / "src/lu_kernel.h", root / "src/black_hole_utils.h",
                           root / "host/salut_layout_validation.h",
                           *root.glob("src/salut_*.h"), *root.glob("src/salut_*.inc"),
                           *root.glob("tests/memory/*.cpp"), *root.glob("tests/memory/*.h")]:
                shutil.copy2(source, fixture / source.relative_to(root))
            def run(*args):
                return subprocess.run(["make", *args], cwd=fixture, text=True,
                                      stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                      timeout=120)
            green = run("memory-test", "MEMORY_CASE=protocol")
            self.assertEqual(green.returncode, 0, green.stdout)
            binary = fixture / "build/memory_banked_lu/tests/memory_test"
            header = fixture / "src/salut_memory_protocol.h"
            later = binary.stat().st_mtime + 2
            os.utime(header, (later, later))
            dry = run("-n", "memory-test", "MEMORY_CASE=protocol")
            self.assertEqual(dry.returncode, 0, dry.stdout)
            self.assertIn("-UNDEBUG", dry.stdout)
            self.assertIn("tests/memory/main.cpp", dry.stdout)
            self.assertNotIn("-DNDEBUG", dry.stdout)
            invalid = run("memory-test", "MEMORY_CASE=does_not_exist")
            self.assertNotEqual(invalid.returncode, 0)
            self.assertIn("missing/unknown scenario", invalid.stdout)


if __name__ == "__main__":
    unittest.main(verbosity=2)
