"""Isaac/PhysX C1 probe for a force-limited, reaction-sensitive rigid actuator."""
from __future__ import annotations

import argparse
import json
import sys
import time
import traceback
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from world_model_dataset.controllers import bounded_velocity_effort, signed_work_increment
from world_model_dataset.io import read_json, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    config = read_json(args.config)

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "renderer": "RayTracedLighting", "width": 320, "height": 240})
    simulation = None
    attached = False
    streams = []
    try:
        import carb
        import numpy as np
        import omni.usd
        from omni.physx import get_physx_simulation_interface
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, PhysxSchema, PhysicsSchemaTools, UsdUtils

        timing = config["timing"]
        controller = config["controller"]
        actuator = config["actuator"]
        load = config["load"]
        hz = timing["physics_hz"]
        dt = 1.0 / hz
        steps = round(timing["duration_s"] * hz)
        if hz % timing["capture_hz"]:
            raise ValueError("capture_hz must divide physics_hz")
        for value in (controller["start_time_s"], controller["end_time_s"]):
            if abs(value * hz - round(value * hz)) > 1e-8:
                raise ValueError("controller time is off the physics grid")

        settings = carb.settings.get_settings()
        settings.set("/physics/updateToUsd", True)
        settings.set("/physics/updateVelocitiesToUsd", True)
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
        UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr(9.81)
        scene_api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        scene_api.CreateSolverTypeAttr("TGS")
        scene_api.CreateTimeStepsPerSecondAttr(hz)

        material_path = "/World/Material"
        UsdShade.Material.Define(stage, material_path)
        material = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(material_path))
        material.CreateStaticFrictionAttr(0.3)
        material.CreateDynamicFrictionAttr(0.2)
        material.CreateRestitutionAttr(0.0)

        def collision(prim):
            UsdPhysics.CollisionAPI.Apply(prim)
            physics_material = UsdShade.MaterialBindingAPI.Apply(prim)
            physics_material.Bind(UsdShade.Material.Get(stage, material_path),
                                  UsdShade.Tokens.weakerThanDescendants, "physics")
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)

        def cube(path, position, size, color, mass=None, disable_gravity=False, collision_enabled=True):
            shape = UsdGeom.Cube.Define(stage, path)
            shape.CreateSizeAttr(1.0)
            shape.AddTranslateOp().Set(Gf.Vec3d(*position))
            shape.AddScaleOp().Set(Gf.Vec3f(*size))
            shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            if collision_enabled:
                collision(shape.GetPrim())
            body = None
            if mass is not None:
                body = UsdPhysics.RigidBodyAPI.Apply(shape.GetPrim())
                body.CreateVelocityAttr(Gf.Vec3f(0.0))
                body.CreateAngularVelocityAttr(Gf.Vec3f(0.0))
                UsdPhysics.MassAPI.Apply(shape.GetPrim()).CreateMassAttr(float(mass))
                rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(shape.GetPrim())
                rigid.CreateDisableGravityAttr(bool(disable_gravity))
                rigid.CreateLinearDampingAttr(0.0)
                rigid.CreateAngularDampingAttr(0.05)
            return shape, body

        cube("/World/floor", [0.0, 0.0, -0.025], [4.0, 2.4, 0.05], [.25, .27, .30])
        lanes = {}
        for scenario in config["scenarios"]:
            sid = scenario["id"]
            y = scenario["lane_y_m"]
            root = f"/World/{sid}"
            UsdGeom.Xform.Define(stage, root)
            initial = [actuator["initial_x_m"], y, 0.18]
            anchor = UsdGeom.Xform.Define(stage, root + "/anchor")
            anchor.AddTranslateOp().Set(Gf.Vec3d(*initial))
            anchor_body = UsdPhysics.RigidBodyAPI.Apply(anchor.GetPrim())
            anchor_body.CreateKinematicEnabledAttr(True)
            PhysxSchema.PhysxRigidBodyAPI.Apply(anchor.GetPrim()).CreateDisableGravityAttr(True)

            pusher, pusher_body = cube(
                root + "/pusher", initial, actuator["size_m"], [.95, .45, .10],
                actuator["mass_kg"], disable_gravity=True,
            )
            joint = UsdPhysics.PrismaticJoint.Define(stage, root + "/slider")
            joint.CreateBody0Rel().SetTargets([Sdf.Path(root + "/anchor")])
            joint.CreateBody1Rel().SetTargets([Sdf.Path(root + "/pusher")])
            joint.CreateAxisAttr("X")
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            joint.CreateLowerLimitAttr(0.0)
            joint.CreateUpperLimitAttr(actuator["travel_limit_m"])

            force_api = PhysxSchema.PhysxForceAPI.Apply(pusher.GetPrim())
            force_attr = force_api.CreateForceAttr()
            force_attr.Set(Gf.Vec3f(0.0))
            force_api.CreateTorqueAttr(Gf.Vec3f(0.0))
            force_api.CreateModeAttr("force")
            force_api.CreateForceEnabledAttr(True)
            force_api.CreateWorldFrameEnabledAttr(True)

            load_path = None
            if scenario["load"]:
                load_path = root + "/load"
                cube(load_path, [load["initial_x_m"], y, load["size_m"] / 2],
                     [load["size_m"]] * 3, [.20, .55, .90], load["mass_kg"])
            if scenario["blocking_wall"]:
                wall_left = load["initial_x_m"] + load["size_m"] / 2
                cube(root + "/wall", [wall_left + 0.05, y, 0.3], [0.1, 0.6, 0.6], [.45, .47, .50])

            lanes[sid] = {
                "scenario": scenario,
                "pusher": pusher,
                "body": pusher_body,
                "force_attr": force_attr,
                "pusher_path": root + "/pusher",
                "load_path": load_path,
                "initial_x_m": actuator["initial_x_m"],
                "previous_x_m": actuator["initial_x_m"],
                "work_j": 0.0,
                "peak_speed_m_s": 0.0,
                "saturated_steps": 0,
                "command_steps": 0,
                "terminal_saturated_steps": 0,
                "terminal_command_steps": 0,
                "contact_impulse_ns": 0.0,
                "trace": [],
            }

        step_index = 0

        def belongs(path, root):
            return path == root or path.startswith(root + "/")

        def on_contact(headers, data):
            for header in headers:
                paths = [str(PhysicsSchemaTools.intToSdfPath(getattr(header, key)))
                         for key in ("actor0", "actor1", "collider0", "collider1")]
                impulse = 0.0
                for index in range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data):
                    impulse += float(np.linalg.norm(np.asarray(data[index].impulse, dtype=float)))
                for lane in lanes.values():
                    if any(belongs(path, lane["pusher_path"]) for path in paths):
                        lane["contact_impulse_ns"] += impulse

        simulation = get_physx_simulation_interface()
        subscription = simulation.subscribe_contact_report_events(on_contact)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        for _ in range(3):
            app.update()
        simulation.attach_stage(stage_id)
        attached = True

        command_stream = (args.output / "command_trace.jsonl").open("x", encoding="utf-8")
        state_stream = (args.output / "actuator_state_trace.jsonl").open("x", encoding="utf-8")
        effort_stream = (args.output / "actuator_effort_trace.jsonl").open("x", encoding="utf-8")
        streams.extend([command_stream, state_stream, effort_stream])
        for sid in lanes:
            command_stream.write(json.dumps({
                "scenario_id": sid,
                "controller_id": "pusher_velocity_effort",
                "start_time_s": controller["start_time_s"],
                "end_time_s": controller["end_time_s"],
                "target_velocity_m_s": controller["target_velocity_m_s"],
                "velocity_gain_n_s_m": controller["velocity_gain_n_s_m"],
                "max_force_n": controller["max_force_n"],
            }, allow_nan=False) + "\n")

        start = time.monotonic()
        for step in range(steps):
            t = step * dt
            command_active = controller["start_time_s"] <= t < controller["end_time_s"]
            decisions = {}
            for sid, lane in lanes.items():
                measured_vx = float(lane["body"].GetVelocityAttr().Get()[0])
                if command_active:
                    decision = bounded_velocity_effort(
                        controller["target_velocity_m_s"], measured_vx,
                        controller["velocity_gain_n_s_m"], controller["max_force_n"],
                    )
                    lane["command_steps"] += 1
                    lane["saturated_steps"] += int(decision["saturated"])
                    if t >= controller["end_time_s"] - 0.25 * (controller["end_time_s"] - controller["start_time_s"]):
                        lane["terminal_command_steps"] += 1
                        lane["terminal_saturated_steps"] += int(decision["saturated"])
                else:
                    decision = {"requested_force_n": 0.0, "applied_force_n": 0.0, "saturated": False}
                lane["force_attr"].Set(Gf.Vec3f(decision["applied_force_n"], 0.0, 0.0))
                decisions[sid] = decision

            step_index = step + 1
            simulation.simulate(dt, t)
            simulation.fetch_results()
            app.update()
            state_time = step_index * dt
            cache = UsdGeom.XformCache(Usd.TimeCode.Default())
            for sid, lane in lanes.items():
                position = cache.GetLocalToWorldTransform(lane["pusher"].GetPrim()).ExtractTranslation()
                x = float(position[0])
                velocity = lane["body"].GetVelocityAttr().Get()
                vx = float(velocity[0])
                decision = decisions[sid]
                work = signed_work_increment(decision["applied_force_n"], lane["previous_x_m"], x)
                lane["work_j"] += work
                lane["previous_x_m"] = x
                lane["peak_speed_m_s"] = max(lane["peak_speed_m_s"], abs(vx))
                state_stream.write(json.dumps({
                    "scenario_id": sid,
                    "time_s": state_time,
                    "position_m": [float(position[0]), float(position[1]), float(position[2])],
                    "linear_velocity_m_s": list(map(float, velocity)),
                }, allow_nan=False) + "\n")
                effort_stream.write(json.dumps({
                    "scenario_id": sid,
                    "time_s": state_time,
                    **decision,
                    "work_increment_j": work,
                    "cumulative_work_j": lane["work_j"],
                }, allow_nan=False) + "\n")
                lane["trace"].append((state_time, x, vx))

        for stream in streams:
            stream.close()
        streams.clear()
        summaries = {}
        for sid, lane in lanes.items():
            final_t, final_x, final_vx = lane["trace"][-1]
            command_end = min(lane["trace"], key=lambda row: abs(row[0] - controller["end_time_s"]))
            summaries[sid] = {
                "initial_x_m": lane["initial_x_m"],
                "command_end_x_m": command_end[1],
                "command_end_velocity_m_s": command_end[2],
                "command_displacement_m": command_end[1] - lane["initial_x_m"],
                "final_x_m": final_x,
                "final_velocity_m_s": final_vx,
                "peak_speed_m_s": lane["peak_speed_m_s"],
                "controller_work_j": lane["work_j"],
                "saturated_fraction": lane["saturated_steps"] / lane["command_steps"],
                "terminal_saturated_fraction": lane["terminal_saturated_steps"] / lane["terminal_command_steps"],
                "pusher_contact_impulse_ns": lane["contact_impulse_ns"],
            }
        displacement = {sid: row["command_displacement_m"] for sid, row in summaries.items()}
        behavior = {
            "free_exceeds_resisted": displacement["free"] > displacement["resisted"],
            "resisted_exceeds_overload": displacement["resisted"] > displacement["overload"],
            "overload_is_force_limited": (
                summaries["overload"]["terminal_saturated_fraction"] > 0.95 and
                abs(summaries["overload"]["command_end_velocity_m_s"]) < 0.01
            ),
            "same_command_all_scenarios": True,
        }
        write_json(args.output / "probe_report.json", {
            "schema_version": "0.2.0-draft",
            "probe_id": config["probe_id"],
            "status": "completed",
            "runtime": "Isaac Sim 6.0.1 / PhysX 110.1.13",
            "elapsed_seconds": time.monotonic() - start,
            "controller": controller,
            "summaries": summaries,
            "behavior": behavior,
            "behavior_passed": all(behavior.values()),
            "trace_semantics": {
                "command": "one declared command per scenario",
                "state": "native rigid-body state after each explicit physics step",
                "effort": "requested and force-limited PhysxForceAPI input plus signed actuator work",
            },
        })
        stage.GetRootLayer().Export(str(args.output / "probe_final.usda"))
        print("CAUSAL_EFFORT_PROBE_COMPLETE " + str(args.output), flush=True)
    except Exception as exc:
        traceback.print_exc()
        if not (args.output / "failure.json").exists():
            write_json(args.output / "failure.json", {
                "error": f"{type(exc).__name__}: {exc}",
                "traceback": traceback.format_exc(),
            })
        raise
    finally:
        for stream in streams:
            stream.close()
        if attached:
            simulation.detach_stage()
        app.close()


if __name__ == "__main__":
    main()
