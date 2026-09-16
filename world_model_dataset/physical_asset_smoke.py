"""Swap exploration assets into the unchanged C2 finite-push development scene."""
import argparse
from pathlib import Path

from .causal_runner import ROOT, load_config, prepare, invoke, package_physics
from .io import file_hash, write_json


def make_config(exploration_root, name, size=.2, mass=None, density=700.):
    exploration_root = Path(exploration_root).resolve()
    if Path(name).name != name or not name.replace('_','').isalnum():
        raise ValueError("Expected a single asset name")
    config = load_config(ROOT/"configs/dataset/v0_2/c2_rigid_push.json")
    loader = exploration_root/"experiments/physical_assets/prepare.py"
    config["prototype_id"] = "development_physical_asset_c2_push"
    config["numerics"]["gpu_dynamics"] = True
    config["geometry_profiles"]["exploration_asset"] = {
        "shape":"mesh", "physical_asset_source":{
            "directory":str(exploration_root/"output/physical_assets/library_v1"/name),
            "loader_path":str(loader), "loader_sha256":file_hash(loader),
            "size":{"max_extent_m":size}}}
    config["physics_profiles"]["asset_development"] = dict(
        mass_kg=mass, density_kg_m3=None if mass is not None else density,
        static_friction=.4,dynamic_friction=.4,restitution=.1,
        linear_damping=0.,angular_damping=0.)
    config["body_profile_overrides"] = {"load":{
        "geometry_id":"exploration_asset", "physics_profile_id":"asset_development"}}
    config["place_on_floor"] = ["load"]
    return config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exploration-root",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--assets",nargs="+",default=["banana","elephant","chair","carrot"])
    parser.add_argument("--size",type=float,default=.2)
    quantities = parser.add_mutually_exclusive_group()
    quantities.add_argument("--mass",type=float)
    quantities.add_argument("--density",type=float,default=700.)
    parser.add_argument("--prepare-only",action="store_true")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True,exist_ok=False)
    records = []
    for name in args.assets:
        recipe = output/"recipes"/(name+".json")
        write_json(recipe,make_config(args.exploration_root,name,args.size,args.mass,args.density))
        episode = output/"episodes"/name
        prepare(recipe,episode)
        if not args.prepare_only:
            print("START",name,flush=True)
            invoke("native_causal_rigid.py",episode,"simulation.log")
            package_physics(episode)
            print("DONE",name,flush=True)
        records.append({"asset":name,"episode":str(episode),"config":str(recipe),
                        "status":"prepared" if args.prepare_only else "physics_completed_not_asset_admitted"})
    write_json(output/"index.json",{"scope":"development asset loading in existing C2 scene; not parameter-range or concavity qualification",
                                   "episodes":records})


if __name__ == "__main__":
    main()
