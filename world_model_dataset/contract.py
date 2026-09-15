"""M1 contract: schema plus references, units, timing and immutable provenance."""
from __future__ import annotations

import math
import subprocess
import shutil
from functools import lru_cache
from pathlib import Path

from .io import digest, file_hash, finite, inside, read_json, write_json

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/dataset"


@lru_cache(maxsize=16)
def _validator(kind):
    import jsonschema
    schema = read_json(CONFIG / "schema.json")
    extension_path=CONFIG/'m4_extensions.json'
    if extension_path.exists():
        additions=read_json(extension_path).get('events',{})
        allowed=schema['$defs']['spec']['properties']['event_id']['enum']
        allowed.extend(event for event in additions if event not in allowed)
    jsonschema.Draft202012Validator.check_schema(schema)
    return jsonschema.Draft202012Validator(dict(schema, **{"$ref": f"#/$defs/{kind}"}))


def validate_schema(value, kind):
    finite(value)
    _validator(kind).validate(value)


def _lookup(registry, key):
    data = read_json(CONFIG / f"{registry}.json")
    if data["schema_version"] != "0.1.0":raise ValueError(f"Invalid {registry} registry")
    if key in data["entries"]:return data["entries"][key]
    extension_path=CONFIG/'m4_extensions.json'
    extension=read_json(extension_path).get(registry,{}) if extension_path.exists() else {}
    if key in extension:return extension[key]
    raise ValueError(f"Unknown {registry} reference: {key}")


def resolve(spec):
    validate_schema(spec, "spec")
    if len({o["instance_id"] for o in spec["objects"]}) != len(spec["objects"]):
        raise ValueError("Duplicate instance ID")
    builtins={'R01':('rigid',1),'V02':('volumetric',1)}
    if spec['event_id'] in builtins:
        group,object_count=builtins[spec['event_id']]
    else:
        extension=read_json(CONFIG/'m4_extensions.json')['events'].get(spec['event_id'])
        if extension is None:raise ValueError(f"Unknown event: {spec['event_id']}")
        group,object_count=extension['group'],extension['object_count']
    if len(spec['objects'])!=object_count:
        raise ValueError(f"{spec['event_id']} requires exactly {object_count} objects")
    event = read_json(CONFIG / "events" / group / f"{spec['event_id']}.json")
    resolved = {"event": event, "environment": _lookup("environments", spec["environment_id"]),
                "cameras": _lookup("cameras", spec["camera_set_id"]),
                "numerics": _lookup("numerics",spec.get("numerics_profile_id","reference")), "objects": []}
    for o in spec["objects"]:
        geometry = _lookup("objects", o["object_id"])
        physics = _lookup("physics_materials", o["physics_profile_id"])
        appearance = _lookup("appearance_materials", o["appearance_profile_id"])
        if physics["kind"] not in event["allowed_physics"]:
            raise ValueError("Event/physical representation mismatch")
        if abs(sum(v*v for v in o["orientation_xyzw"]) - 1) > 1e-6:
            raise ValueError("Orientation quaternion must be normalized")
        if geometry["characteristic_size_m"] <= 0 or physics["density_kg_m3"] <= 0:
            raise ValueError("Invalid physical scale")
        if physics["kind"] == "rigid" and physics["static_friction"] < physics["dynamic_friction"]:
            raise ValueError("Static friction must not be below dynamic friction")
        resolved["objects"].append(dict(o, geometry=geometry, physics=physics, appearance=appearance))
    fp, ap = spec["fixture_parameters"], spec["action_parameters"]
    if 'fixture_parameters' in event:
        expected_fixture=set(event['fixture_parameters']);expected_action=set(event['action_parameters'])
    else:
        expected_fixture = {"angle_deg", "length_D"} if group == "rigid" else {"compression_fraction"}
        expected_action = set(event["parameters"]) - expected_fixture
    if set(fp) != expected_fixture or set(ap) != expected_action:
        raise ValueError("Unexpected or missing event parameter")
    for name, value in {**fp, **ap}.items():
        low, high = event["parameters"][name]
        if not low <= value <= high:
            raise ValueError(f"Outside event parameter domain: {name}")
    t = spec["timing"]
    if t["physics_hz"] % t["capture_hz"]:
        raise ValueError("Capture rate must divide physics rate")
    for rate in (t["physics_hz"], t["capture_hz"]):
        if abs(t["duration_s"] * rate - round(t["duration_s"] * rate)) > 1e-8:
            raise ValueError("Duration must land on both time grids")
    if spec['event_id']=='R01':ends=ap['release_time_s']
    elif spec['event_id']=='V02':ends=sum(ap.values())
    elif spec['event_id']=='V01':ends=0.
    elif spec['event_id']=='R04':ends=ap['start_time_s']+ap['push_duration_s']
    elif spec['event_id']=='V03':ends=ap['load_start_time_s']+ap['load_duration_s']
    else:ends=ap['start_time_s']
    if ends >= t["duration_s"]:
        raise ValueError("Episode must include post-action observation")
    for name, value in ap.items():
        if (name.endswith('_time_s') or name.endswith('_duration_s')) and abs(value * t["physics_hz"] - round(value * t["physics_hz"])) > 1e-8:
            raise ValueError(f"Action time not on physics grid: {name}")
    cf = spec["counterfactual"]
    if (cf["baseline_episode_id"] is None) != (cf["changed_pointer"] is None):
        raise ValueError("Counterfactual baseline and pointer must both be provided")
    return resolved


def artifact(root, relative):
    path = inside(root, relative)
    return {"path": relative, "sha256": file_hash(path), "bytes": path.stat().st_size}


def provenance():
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    sources = sorted(p for folder in (ROOT / "world_model_dataset", CONFIG) for p in folder.rglob("*")
                     if p.is_file() and p.suffix in (".py", ".json") and "__pycache__" not in p.parts)
    sources += [ROOT / "soft_body/tet_quality.py"]
    return commit, {p.relative_to(ROOT).as_posix(): file_hash(p) for p in sources}


def prepare(spec_path, output):
    from .actions import compile_actions
    spec = read_json(spec_path)
    inputs = resolve(spec)
    output = Path(output)
    if output.exists():
        raise FileExistsError(f"Refusing to replace episode directory: {output}")
    commit, sources = provenance()
    output.mkdir(parents=True)
    for relative,sha in sources.items():
        target=inside(output,'source/'+relative)
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(inside(ROOT,relative),target)
        if file_hash(target)!=sha:
            raise ValueError(f'Source changed during preparation: {relative}')
    from .geometry import save_geometry
    from .fixtures import build_fixture
    import numpy as np
    geometry=save_geometry(output,inputs)
    vertices={}
    for object_spec in spec['objects']:
        instance=object_spec['instance_id']
        with np.load(output/geometry[instance]['path'],allow_pickle=False) as data:
            vertices[instance]=data['vertices'].copy()
    fixture=build_fixture(spec,inputs,vertices if len(vertices)>1 else next(iter(vertices.values())))
    write_json(output/'fixture.json',fixture)
    actions=compile_actions(spec,inputs,fixture)
    validate_schema(actions,'action');write_json(output/'action.json',actions)
    result = dict(schema_version="0.1.0", episode_id=spec["episode_id"], lifecycle="prepared", spec=spec,
                  inputs=inputs, inputs_sha256=digest(inputs), source_commit=commit, source_files=sources,
                  units=dict(length="m", mass="kg", time="s", angle="rad", world_up="Z", quaternion="xyzw"),
                  action=artifact(output, "action.json"), artifacts=[artifact(output,p) for p in
                      ['fixture.json','geometry/index.json',*[g['path'] for g in geometry.values()],
                       *['source/'+p for p in sources]]], capabilities={
                      "rigid_contact_impulse": dict(status="not_probed", source="PhysX contact report", reason="Runtime probe required"),
                      "soft_contact_impulse": dict(status="not_probed", source="PhysX contact report", reason="Runtime probe required")})
    validate_schema(result, "episode")
    write_json(output / "episode.prepared.json", result)
    return result


def validate_episode(path, require_complete=False, check_source=False):
    """Read-only validator. Never creates outputs or launches simulation."""
    path = Path(path)
    episode = read_json(path)
    validate_schema(episode, "episode")
    validate_schema(episode["spec"], "spec")
    if episode["episode_id"] != episode["spec"]["episode_id"]:
        raise ValueError("Mismatched episode identity")
    if digest(episode["inputs"]) != episode["inputs_sha256"]:
        raise ValueError("Resolved input hash mismatch")
    if episode['inputs'].get('event',{}).get('id')!=episode['spec']['event_id']:
        raise ValueError('Resolved event differs from requested event')
    resolved_objects=episode['inputs'].get('objects',[])
    if len(resolved_objects)!=len(episode['spec']['objects']):raise ValueError('Resolved object count mismatch')
    for requested,resolved in zip(episode['spec']['objects'],resolved_objects):
        if any(resolved.get(k)!=v for k,v in requested.items()):raise ValueError('Resolved object/reference mismatch')
    seen = set()
    for item in [episode["action"], *episode["artifacts"]]:
        if item["path"] in seen:
            raise ValueError("Duplicate output path")
        seen.add(item["path"])
        if artifact(path.parent, item["path"]) != item:
            raise ValueError(f"Artifact checksum/size mismatch: {item['path']}")
    actions = read_json(inside(path.parent, episode["action"]["path"]))
    validate_schema(actions, "action")
    from .actions import compile_actions
    if actions != compile_actions(episode["spec"], episode["inputs"],read_json(path.parent/'fixture.json')):
        raise ValueError("Action file does not match declared inputs")
    if check_source:
        for relative, sha in episode["source_files"].items():
            if file_hash(inside(ROOT, relative)) != sha:
                raise ValueError(f"Source changed: {relative}")
    if require_complete:
        if episode["lifecycle"] != "completed":
            raise ValueError("Episode is not completed")
        required={"state/index.json", "metrics.json", "validation.json", "observations/index.json"}
        if episode['capabilities'].get('rigid_contact_impulse',{}).get('status')=='native':required.add('contacts.jsonl')
        if not required <= seen:
            raise ValueError("Missing required output")
        report = read_json(path.parent / "validation.json")
        validate_schema(report, "validation")
        if not report["passed"] or report["missing_required"]:
            raise ValueError("Physical/observation validation has not passed")
    return episode


def differences(a, b, pointer=""):
    if type(a) is not type(b):
        return [pointer]
    if isinstance(a, dict):
        if set(a) != set(b):
            return [pointer]
        return [p for k in a for p in differences(a[k], b[k], pointer + "/" + k)]
    if isinstance(a, list):
        if len(a) != len(b):
            return [pointer]
        return [p for i, (x,y) in enumerate(zip(a,b)) for p in differences(x,y,pointer+f"/{i}")]
    return [] if a == b else [pointer]


def validate_pair(base, variant):
    """Compare resolved physics, not just differing material ID strings."""
    left, right = resolve(base), resolve(variant)
    if variant["counterfactual"]["baseline_episode_id"] != base["episode_id"]:
        raise ValueError("Wrong baseline episode")
    for key in ("family_id",):
        if base["counterfactual"][key] != variant["counterfactual"][key]:
            raise ValueError("Counterfactual family mismatch")
    def semantic(spec, resolved):
        objects = [{k:v for k,v in o.items() if k not in ("physics_profile_id", "appearance_profile_id", "object_id")}
                   for o in resolved["objects"]]
        return {**{k:v for k,v in spec.items() if k not in ("episode_id", "counterfactual", "objects", "environment_id", "camera_set_id", "numerics_profile_id")},
                "objects": objects, "environment": resolved["environment"], "cameras": resolved["cameras"], "numerics": resolved['numerics']}
    changes = differences(semantic(base,left), semantic(variant,right))
    if changes != [variant["counterfactual"]["changed_pointer"]]:
        raise ValueError(f"Not the declared single-variable counterfactual: {changes}")
    return changes
