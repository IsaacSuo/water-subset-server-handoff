"""Run six independent native rotational capability episodes, without rendering."""
from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path

from .causal_contract import audit_causal_manifest
from .io import read_json, write_json
from .local import windows_path

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    summaries = {}
    for law in ("effort", "impedance"):
        summaries[law] = {}
        for sid in ("free", "resisted", "overload"):
            output = args.output / f"c1_rotary_{law}_{sid}"
            values = [r"Y:\isaacsim\python.bat", windows_path(ROOT / "world_model_dataset/causal_effort_probe.py"),
                      "--config", windows_path(ROOT / f"configs/dataset/v0_2/c1_rotary_{law}_probe.json"),
                      "--scenario", sid, "--output", windows_path(output)]
            command = "& " + " ".join("'" + v.replace("'", "''") + "'" for v in values) + "; exit $LASTEXITCODE"
            print("START", law, sid, flush=True)
            with (args.output / f"{law}_{sid}.log").open("x", encoding="utf-8") as stream:
                process = subprocess.run(["powershell.exe", "-NoProfile", "-Command", command], cwd=ROOT,
                                         stdout=stream, stderr=subprocess.STDOUT)
            if process.returncode or (output / "failure.json").exists() or not (output / "packaging_review.json").exists():
                raise RuntimeError(f"Native run failed: {output}")
            audit = audit_causal_manifest(read_json(output / "episode.json"))
            write_json(output / "contract_review.json", audit)
            if not audit["accepted"]:
                raise ValueError(audit["errors"])
            summary = read_json(output / "probe_report.json")["summaries"][sid]
            summaries[law][sid] = summary
            print("DONE", law, sid, json.dumps(summary), flush=True)
    write_json(args.output / "rotary_summary.json", {
        "milestone": "C1", "status": "physics_completed_pending_manual_review",
        "c2_prototypes_completed": 0, "summaries": summaries,
        "notes": "Same finite command for all loads within each law. Numerical cache review follows; no video or C2 admission.",
    })


if __name__ == "__main__":
    main()
