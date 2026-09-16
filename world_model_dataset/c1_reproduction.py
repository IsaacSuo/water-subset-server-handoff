"""Serial independent native C1 runs and post-run comparison with the three-lane cache."""
from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path

from .io import file_hash, read_json, write_json
from .local import windows_path
from .causal_contract import audit_causal_manifest

ROOT = Path(__file__).resolve().parents[1]
REFERENCES = {"effort": "c1_effort_probe03", "impedance": "c1_impedance_probe02"}


def compare_traces(reference, independent, sid):
    def rows(path):
        return [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines() if line]
    old = [r for r in rows(reference) if r["scenario_id"] == sid]
    new = rows(independent)
    if not old or len(old) != len(new) or any(r["scenario_id"] != sid for r in new) or \
            any(a["time_s"] != b["time_s"] for a, b in zip(old, new)):
        return {"matched_time_grid": False, "reproduced": False}
    position = max(abs(a["position_m"][i] - b["position_m"][i]) for a, b in zip(old, new) for i in range(3))
    velocity = max(abs(a["linear_velocity_m_s"][i] - b["linear_velocity_m_s"][i]) for a, b in zip(old, new) for i in range(3))
    return {"matched_time_grid": True, "state_count": len(new), "max_position_difference_m": position,
            "max_velocity_difference_m_s": velocity,
            "reproduced": position <= 0.001 and velocity <= 0.01}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    results = []
    for law, reference_id in REFERENCES.items():
        for sid in ("free", "resisted", "overload"):
            output = args.output / f"c1_{law}_{sid}"
            values = [r"Y:\isaacsim\python.bat", windows_path(ROOT / "world_model_dataset/causal_effort_probe.py"),
                      "--config", windows_path(ROOT / f"configs/dataset/v0_2/c1_{law}_probe.json"),
                      "--scenario", sid, "--output", windows_path(output)]
            command = "& " + " ".join("'" + v.replace("'", "''") + "'" for v in values) + "; exit $LASTEXITCODE"
            print("START", law, sid, flush=True)
            with (args.output / f"{law}_{sid}.log").open("x", encoding="utf-8") as stream:
                result = subprocess.run(["powershell.exe", "-NoProfile", "-Command", command], cwd=ROOT,
                                        stdout=stream, stderr=subprocess.STDOUT)
            if result.returncode or (output / "failure.json").exists() or not (output / "packaging_review.json").exists():
                raise RuntimeError(f"Independent run failed: {output}")
            audit = audit_causal_manifest(read_json(output / "episode.json"))
            write_json(output / "contract_review.json", audit)
            if not audit["accepted"]:
                raise ValueError(audit["errors"])
            reference = ROOT / "output/world_model_dataset/v0_2" / reference_id
            comparison = compare_traces(reference / "actuator_state_trace.jsonl", output / "actuator_state_trace.jsonl", sid)
            results.append({"law": law, "scenario": sid, "episode": output.name, **comparison,
                            "reference_report_sha256": file_hash(reference / "probe_report.json"),
                            "independent_summary": read_json(output / "probe_report.json")["summaries"][sid]})
            print("DONE", law, sid, json.dumps(comparison), flush=True)
    write_json(args.output / "reproduction_review.json", {
        "milestone": "C1", "c2_prototypes_completed": 0,
        "comparison_tolerances": {"position_m": 0.001, "velocity_m_s": 0.01},
        "note": "Post-run numerical comparison, not a production admission gate. Lane coordinates retained.",
        "all_reproduced": all(r["reproduced"] for r in results), "episodes": results,
    })


if __name__ == "__main__":
    main()
