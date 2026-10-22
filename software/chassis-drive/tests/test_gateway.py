from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class GatewayTests(unittest.TestCase):
    def test_actual_c_gateway(self):
        compiler = shutil.which("gcc")
        if compiler is None:
            raise RuntimeError("gcc required")
        with tempfile.TemporaryDirectory(prefix="chassis-gateway-") as temporary:
            exe = Path(temporary) / "gateway_test.exe"
            subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
                            "-I", str(ROOT / "firmware/common/include"),
                            str(ROOT / "firmware/common/src/diagnostic_gateway.c"),
                            str(ROOT / "tests/gateway_native.c"), "-o", str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True, check=True)
            self.assertIn("tests passed", result.stdout)


if __name__ == "__main__":
    unittest.main()
