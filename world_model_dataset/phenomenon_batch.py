"""Small phenomenon-led recipe batch, reusing the v0.2 solver and existing caches."""
from __future__ import annotations

import argparse
import copy
from pathlib import Path

from .causal_runner import ROOT, load_config, prepare, invoke, package_physics
from .io import read_json, write_json


def recipe(design, variant, asset):
    """One design accepts a list of compatible assets; no per-asset scene scripts."""
    kind = design["kind"]
    controlled = kind == "confinement"
    base = "c2_rigid_push.json" if controlled else "c2_none_collision.json"
    config = load_config(ROOT / "configs/dataset/v0_2" / base)
    manifest = read_json(ROOT / config["manifest_template"])
    for oid, values in config["initial_state_overrides"].items():
        manifest["initial_state"]["body_states"][oid].update(values)
    for body in manifest["system"]["bodies"]:
        body.update(config.get("body_profile_overrides", {}).get(body["instance_id"], {}))
    config["initial_state_overrides"] = {}
    config.pop("body_profile_overrides", None)
    config["prototype_id"] = "phenomenon_" + kind
    manifest["environment"]["environment_id"] = "self_built_" + kind
    oid = "load" if controlled else "left"
    if not controlled:
        manifest["system"]["bodies"] = [b for b in manifest["system"]["bodies"] if b["instance_id"] != "right"]
        manifest["initial_state"]["participant_ids"].remove("right")
        del manifest["initial_state"]["body_states"]["right"]
    geometry = {k: copy.deepcopy(v) for k, v in asset.items() if k != "mass_kg"}
    if geometry["shape"] not in ("box", "sphere"):
        raise ValueError("This first simple-geometry batch takes analytic boxes/spheres; external assets are a separate adapter")
    size = geometry.get("size_m", [2 * geometry.get("radius_m", 0)] * 3)
    if len(size) != 3 or min(size) <= 0:
        raise ValueError("Asset dimensions must be positive")
    if kind == "rolling" and geometry["shape"] != "sphere":
        raise ValueError("Rolling diagnostic requires a sphere with known radius")
    config["geometry_profiles"]["subject"] = geometry
    descriptor = next(b for b in manifest["system"]["bodies"] if b["instance_id"] == oid)
    descriptor["geometry_id"] = "subject"
    profile = config["physics_profiles"][descriptor["physics_profile_id"]]
    profile.update(mass_kg=asset["mass_kg"], restitution=0.0)
    initial = manifest["initial_state"]["body_states"][oid]
    initial.update(position_m=[-0.15 if controlled else -0.6, 0, size[2]/2],
                   linear_velocity_m_s=[0 if controlled else (1.2 if kind == "sliding" else 1.0), 0, 0],
                   angular_velocity_rad_s=[0, 0, 0])
    config["geometry_profiles"]["plane"]["size_m"] = [max(4.0, 4*max(size)), max(2.0, 3*size[1]), 0.05]
    floor_id = next(b["physics_profile_id"] for b in manifest["system"]["bodies"] if b["instance_id"] == "floor")
    floor = config["physics_profiles"][floor_id]
    if kind in ("sliding", "rolling"):
        mu = variant["friction"] if kind == "sliding" else 0.25
        profile.update(static_friction=0.6, dynamic_friction=0.6)
        floor.update(static_friction=0.6, dynamic_friction=mu, restitution=0.0)
        if kind == "rolling":
            initial["angular_velocity_rad_s"][1] = variant["rolling_ratio"] / geometry["radius_m"]
    elif kind == "confinement":
        gap = size[1] * variant["gap_ratio"]
        config["geometry_profiles"]["pusher_pad"]["size_m"] = [0.2, size[1]*0.6, size[2]]
        initial["position_m"][0] = min(-0.15, 0.31-size[0]/2-0.1)
        manifest["initial_state"]["body_states"]["pusher"]["position_m"][0] = initial["position_m"][0]-size[0]/2-0.1-0.4
        manifest["initial_state"]["body_states"]["pusher"]["position_m"][2] = size[2]/2 + 0.03
        config["geometry_profiles"]["wall"] = {"shape": "box", "size_m": [0.18, 0.3, max(0.4, size[2]*1.3)]}
        for name, sign in (("wall_left", -1), ("wall_right", 1)):
            body = copy.deepcopy(next(b for b in manifest["system"]["bodies"] if b["instance_id"] == "floor"))
            body.update(instance_id=name, geometry_id="wall")
            manifest["system"]["bodies"].append(body)
            manifest["environment"]["environment_body_ids"].append(name)
            manifest["initial_state"]["participant_ids"].append(name)
            state = copy.deepcopy(manifest["initial_state"]["body_states"]["floor"])
            state["position_m"] = [0.4, sign*(gap/2+0.15), config["geometry_profiles"]["wall"]["size_m"][2]/2]
            manifest["initial_state"]["body_states"][name] = state
    else:
        raise ValueError(kind)
    return config, manifest


def run_batch(spec_path, destination, prepare_only=False):
    spec = read_json(spec_path)
    destination = destination.resolve()
    destination.mkdir(parents=True, exist_ok=True)
    records = []
    for design in spec["designs"]:
        for asset_id in design.get("assets", [None]):
            for variant in design["variants"]:
                existing = variant.get("existing_episode")
                run_id = "_".join(x for x in (design["id"].lower(), asset_id, variant["id"]) if x)
                output = ROOT / existing if existing else destination / "episodes" / run_id
                if not existing:
                    if not (output / "episode.prepared.json").exists():
                        config, manifest = recipe(design, variant, spec["assets"][asset_id])
                        folder = destination / "recipes" / run_id
                        folder.mkdir(parents=True, exist_ok=False)
                        write_json(folder / "manifest.json", manifest)
                        # Relative to the repo keeps recipes portable across Windows/WSL.
                        config["manifest_template"] = (folder / "manifest.json").relative_to(ROOT).as_posix()
                        write_json(folder / "config.json", config)
                        prepare(folder / "config.json", output)
                    if not prepare_only and not (output / "episode.physics.json").exists():
                        if (output / "simulation.log").exists():
                            raise RuntimeError(f"Inspect interrupted run before retrying: {output}")
                        print("START", run_id, flush=True)
                        invoke("native_causal_rigid.py", output, "simulation.log")
                        package_physics(output)
                        print("DONE", run_id, flush=True)
                if existing and not (output / "episode.physics.json").is_file():
                    raise FileNotFoundError(output)
                records.append({"design_id": design["id"], "title": design["title"], "question": design["question"],
                                "kind": design["kind"], "asset_id": asset_id, "variant": variant,
                                "episode": output.relative_to(ROOT).as_posix(), "reused": bool(existing)})
    index = destination / ("prepared_index.json" if prepare_only else "batch_index.json")
    if not index.exists():
        write_json(index, {"batch_id": spec["batch_id"], "design_spec": str(spec_path), "episodes": records,
                           "scope": "phenomenon-led development examples, not dataset release admission"})
    return index


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, default=ROOT / "configs/dataset/v0_2/phenomena_batch01.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--prepare-only", action="store_true")
    args = parser.parse_args()
    print(run_batch(args.spec, args.output, args.prepare_only), flush=True)


if __name__ == "__main__":
    main()
