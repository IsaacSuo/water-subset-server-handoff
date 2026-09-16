"""Read cooked convex geometry from a saved causal initial stage; no simulation."""
import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from world_model_dataset.io import write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    args = parser.parse_args()
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        import numpy as np
        import omni.usd
        from omni.physx import get_physx_cooking_interface
        from pxr import UsdUtils, PhysicsSchemaTools
        omni.usd.get_context().open_stage(str(args.episode / "native_initial.usda"))
        stage = omni.usd.get_context().get_stage()
        stage_id = UsdUtils.StageCache.Get().Insert(stage).ToLongInt()
        result = {}

        def collect(status, convexes):
            result["status"] = str(status)
            result["convexes"] = [np.asarray(c.vertices).tolist() for c in convexes]

        get_physx_cooking_interface().request_convex_collision_representation(
            stage_id=stage_id, collision_prim_id=PhysicsSchemaTools.sdfPathToInt(stage.GetPrimAtPath("/World/load").GetPath()),
            run_asynchronously=False, on_result=collect)
        write_json(args.episode / "cooked_collision_diagnostic.json", result)
        print("COOKED_COLLISION", result["status"], [len(c) for c in result["convexes"]], flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
