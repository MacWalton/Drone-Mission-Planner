"""Run every test script in its own process (QGIS tests need a fresh app each).

    python tests/run_tests.py            all tests
    python tests/run_tests.py core       core + ArcGIS stand-in only (no QGIS needed)

QGIS tests need QGIS's Python (PyQGIS) and run headless (QT_QPA_PLATFORM=offscreen).
Core tests need pyproj; the DEM tests also need GDAL and numpy.
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
QGIS = {"test_qgis.py", "test_qgis_dem.py", "test_qgis_v11.py", "test_brief_ui.py", "test_obstacles_ui.py",
        "test_payload_ui.py", "test_units_ui.py"}


def main():
    subprocess.check_call([sys.executable, os.path.join(ROOT, "tools", "build.py")])
    os.makedirs(os.path.join(HERE, "out"), exist_ok=True)
    only_core = "core" in sys.argv[1:]
    tests = sorted(f for f in os.listdir(HERE) if f.startswith("test_") and f.endswith(".py"))
    if only_core:
        tests = [t for t in tests if t not in QGIS]
    env = dict(os.environ, QT_QPA_PLATFORM="offscreen")
    failed = []
    for t in tests:
        t0 = time.time()
        r = subprocess.run([sys.executable, os.path.join(HERE, t)], cwd=HERE, env=env,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        out = r.stdout
        # PyQGIS can crash while tearing down after a completed script: judge by the output
        ok = r.returncode == 0 or (t in QGIS and "Traceback" not in out and r.returncode in (-11, 139))
        with open(os.path.join(HERE, "out", t + ".log"), "w") as fh:
            fh.write(out)
        print("%-24s %s  %5.1fs" % (t, "ok" if ok else "FAIL (%s)" % r.returncode, time.time() - t0))
        if not ok:
            failed.append(t)
            print("\n".join("    " + l for l in out.strip().splitlines()[-8:]))
    print("\n%d passed, %d failed" % (len(tests) - len(failed), len(failed)))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
