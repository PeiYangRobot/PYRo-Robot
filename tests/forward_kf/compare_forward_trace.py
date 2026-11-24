"""Check migrated forward-filter state, covariance, gain and adaptive R.

Tolerances cover float32 reassociation and masked-row packing; they are not
claims about physical robot accuracy. Both executables run identical inputs.
"""
import json
import math
from pathlib import Path
import struct
import subprocess
import sys

reference, current, directory, scenario = sys.argv[1:]
output = Path(directory)
output.mkdir(parents=True, exist_ok=True)
paths = [output / "reference.bin", output / "current.bin"]
for program, path in zip((reference, current), paths):
    subprocess.run([program, str(path), scenario], check=True)
before, after = [path.read_bytes() for path in paths]
if len(before) != 30000 * 33 * 4 or len(after) != len(before):
    raise SystemExit("FAIL: incomplete forward-filter trace")
absolute_limits = [0.002, 2e-5, 5e-5, 5e-5, 3e-5] + [2e-5]*16 + [3e-5]*8 + [5e-5]*4
maximum = [0.0] * 33
bitwise_equal = [True] * 33
for i, ((a,), (b,)) in enumerate(zip(struct.iter_unpack("<f", before), struct.iter_unpack("<f", after))):
    field = i % 33
    if not math.isfinite(a) or not math.isfinite(b):
        raise SystemExit(f"FAIL: non-finite value at sample {i//33}, field {field}")
    error = abs(a-b)
    maximum[field] = max(maximum[field], error)
    if before[4*i:4*i+4] != after[4*i:4*i+4]:
        bitwise_equal[field] = False
    if error > absolute_limits[field] + 2e-5 * max(abs(a), abs(b)):
        raise SystemExit(f"FAIL: sample {i//33}, field {field}: {a} vs {b}, error {error}")
report = {"reference": "KF_ok / 19ea989", "scenario": scenario, "samples": 30000, "fields_per_sample": 33,
          "max_absolute_errors": {"x": maximum[0], "v": maximum[1], "a": maximum[2],
                                  "bias": maximum[3], "slip_score": maximum[4],
                                  "P": max(maximum[5:21]), "K": max(maximum[21:29]),
                                  "R": max(maximum[29:])},
          "limits_absolute": absolute_limits, "relative_limit": 2e-5}
report["bitwise_equal_fields"] = bitwise_equal
(output / "comparison.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
print("PASS:", json.dumps(report["max_absolute_errors"]), scenario, "30000 samples x 33 float32 vs KF_ok")
