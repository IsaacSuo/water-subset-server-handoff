"""Native causal rigid/volume evolution with bounded physical effort/impedance."""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from world_model_dataset.io import file_hash, inside, read_json, write_json
from world_model_dataset.controllers import signed_work_increment
from world_model_dataset.causal_control import evaluate
from world_model_dataset import causal_soft


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    args = parser.parse_args()
    output = args.episode
    manifest = read_json(output / "episode.prepared.json")
    resolved = read_json(output / "resolved_inputs.json")
    program = manifest["control_program"]
    if program["primitive"] not in ("none", "effort_control", "impedance_control"):
        raise ValueError("Backend supports none and finite translational effort/impedance only")
    controllers = program["controllers"]
    if controllers and len(controllers) != 1:
        raise ValueError("Initial causal backend requires one actuator")
    for joint in manifest["system"]["joints"]:
        if joint["kind"] != "prismatic" or joint["body0_id"] is not None or joint["axis"] not in ("X", "Z"):
            raise ValueError("Backend currently supports world-anchored X/Z sliders only")
    snapshot = read_json(output / "source_snapshot.json")
    for name, checksum in snapshot["sha256"].items():
        if file_hash(ROOT / name) != checksum:
            raise ValueError(f"Prepared source changed: {name}")
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "renderer": "RayTracedLighting", "width": 320, "height": 240})
    simulation = None
    attached = False
    streams = []
    try:
        import carb
        import numpy as np
        import omni.usd
        import omni.physics.tensors as tensors
        from omni.physx import get_physx_interface, get_physx_simulation_interface
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, PhysxSchema, PhysicsSchemaTools, UsdUtils

        settings = carb.settings.get_settings()
        settings.set("/physics/updateToUsd", True)
        settings.set("/physics/updateVelocitiesToUsd", True)
        settings.set("/physics/updateParticlesToUsd", True)
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
        UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        gravity = manifest["environment"]["gravity_m_s2"]
        magnitude = math.sqrt(sum(g * g for g in gravity))
        scene.CreateGravityMagnitudeAttr(magnitude)
        scene.CreateGravityDirectionAttr(Gf.Vec3f(*[g / magnitude for g in gravity]) if magnitude else Gf.Vec3f(0, 0, -1))
        numerics = resolved["numerics"]
        timing = manifest["timing"]
        hz = timing["physics_hz"]
        dt = 1.0 / hz
        scene_api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        scene_api.CreateSolverTypeAttr(numerics["solver"])
        scene_api.CreateTimeStepsPerSecondAttr(hz)
        scene_api.CreateEnableGPUDynamicsAttr(numerics["gpu_dynamics"])
        scene_api.CreateBroadphaseTypeAttr("GPU" if numerics["gpu_dynamics"] else "MBP")
        has_soft = any(b["physics_kind"] == "volumetric" for b in manifest["system"]["bodies"])
        if has_soft:
            if not numerics["gpu_dynamics"]:
                raise ValueError("PhysX volume-deformable prototype requires GPU dynamics")
            scene_api.CreateEnableExternalForcesEveryIterationAttr(numerics["external_forces_every_iteration"])
            scene_api.CreateGpuCollisionStackSizeAttr(64*1024*1024)
            scene_api.CreateGpuMaxDeformableSurfaceContactsAttr(1048576)
            scene_api.CreateGpuMaxDeformableVolumeContactsAttr(1048576)
        actors = {}
        for descriptor in manifest["system"]["bodies"]:
            oid = descriptor["instance_id"]
            definition = resolved["bodies"][oid]
            geometry, physics = definition["geometry"], definition["physics"]
            initial = manifest["initial_state"]["body_states"][oid]
            if descriptor["physics_kind"] == "volumetric":
                actors[oid] = causal_soft.author(stage, oid, definition, initial, numerics)
                continue
            path = "/World/" + oid
            if geometry["shape"] == "sphere":
                shape = UsdGeom.Sphere.Define(stage, path)
                shape.CreateRadiusAttr(geometry["radius_m"])
            elif geometry["shape"] == "box":
                shape = UsdGeom.Cube.Define(stage, path)
                shape.CreateSizeAttr(1.0)
            elif geometry["shape"] == "mesh":
                record = geometry["mesh"]
                mesh_path = inside(output,record["path"])
                with np.load(mesh_path,allow_pickle=False) as data:
                    shape = UsdGeom.Mesh.Define(stage,path)
                    shape.CreatePointsAttr([Gf.Vec3f(*map(float,p)) for p in data["vertices"]])
                    shape.CreateFaceVertexCountsAttr([3]*len(data["triangles"]))
                    shape.CreateFaceVertexIndicesAttr(data["triangles"].ravel().tolist())
                    shape.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            else:
                raise ValueError(f"Unsupported prototype shape: {geometry['shape']}")
            shape.AddTranslateOp().Set(Gf.Vec3d(*initial["position_m"]))
            q = initial["orientation_xyzw"]
            shape.AddOrientOp().Set(Gf.Quatf(q[3], Gf.Vec3f(*q[:3])))
            if geometry["shape"] == "box":
                shape.AddScaleOp().Set(Gf.Vec3f(*geometry["size_m"]))
            shape.CreateDisplayColorAttr([Gf.Vec3f(*definition["appearance"]["color"])])
            prim = shape.GetPrim()
            UsdPhysics.CollisionAPI.Apply(prim)
            if geometry["shape"] == "mesh":
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(geometry["collision_approximation"])
                if geometry["collision_approximation"] == "convexHull" and "hull_vertex_limit" in geometry:
                    PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim).CreateHullVertexLimitAttr(geometry["hull_vertex_limit"])
                elif geometry["collision_approximation"] == "sdf":
                    if not numerics["gpu_dynamics"]:
                        raise ValueError("Dynamic SDF mesh requires GPU dynamics; do not silently fall back to convex hull")
                    PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim).CreateSdfResolutionAttr(geometry["sdf_resolution"])
            collision_api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            collision_api.CreateContactOffsetAttr(numerics["contact_offset_m"])
            collision_api.CreateRestOffsetAttr(numerics["rest_offset_m"])
            material_path = "/World/Material_" + oid
            UsdShade.Material.Define(stage, material_path)
            material = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(material_path))
            material.CreateStaticFrictionAttr(physics["static_friction"])
            material.CreateDynamicFrictionAttr(physics["dynamic_friction"])
            material.CreateRestitutionAttr(physics["restitution"])
            combine = PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(material_path))
            combine.CreateFrictionCombineModeAttr("min")
            combine.CreateRestitutionCombineModeAttr("average")
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(UsdShade.Material.Get(stage, material_path),
                                                      UsdShade.Tokens.weakerThanDescendants, "physics")
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
            body = None
            mass = physics["mass_kg"]
            inertia = None
            inertia_matrix = None
            principal_axes = Gf.Quatf(1.0)
            if descriptor["physics_kind"] == "rigid":
                body = UsdPhysics.RigidBodyAPI.Apply(prim)
                # Preparation only: all initial conditions precede solver load and t0 capture.
                body.CreateVelocityAttr(Gf.Vec3f(*initial["linear_velocity_m_s"]))
                body.CreateAngularVelocityAttr(Gf.Vec3f(*[math.degrees(v) for v in initial["angular_velocity_rad_s"]]))
                rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                rigid.CreateSolverPositionIterationCountAttr(numerics["position_iterations"])
                rigid.CreateSolverVelocityIterationCountAttr(numerics["velocity_iterations"])
                rigid.CreateLinearDampingAttr(physics["linear_damping"])
                rigid.CreateAngularDampingAttr(physics["angular_damping"])
                rigid.CreateEnableSpeculativeCCDAttr(numerics["speculative_ccd"])
                if geometry["shape"] == "sphere":
                    inertia = [0.4 * mass * geometry["radius_m"] ** 2] * 3
                elif geometry["shape"] == "mesh":
                    inertia_matrix = np.asarray(geometry["inertia_tensor_kg_m2"])
                    eigenvalues, eigenvectors = np.linalg.eigh(inertia_matrix)
                    if np.linalg.det(eigenvectors) < 0:
                        eigenvectors[:,0] *= -1
                    principal_axes = Gf.Quatf(Gf.Matrix3d(*map(float,eigenvectors.T.ravel())).ExtractRotation().GetQuat())
                    inertia = eigenvalues.tolist()
                else:
                    x, y, z = geometry["size_m"]
                    inertia = [mass * (y*y + z*z) / 12, mass * (x*x + z*z) / 12, mass * (x*x + y*y) / 12]
                mass_api = UsdPhysics.MassAPI.Apply(prim)
                mass_api.CreateMassAttr(mass)
                mass_api.CreateCenterOfMassAttr(Gf.Vec3f(0.0))
                mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
                mass_api.CreatePrincipalAxesAttr(principal_axes)
            elif descriptor["physics_kind"] != "static":
                raise ValueError("Natural rigid backend cannot simulate flexible bodies")
            actors[oid] = {"prim": prim, "body": body, "mass": mass, "inertia": inertia,
                           "inertia_matrix": inertia_matrix}

        for definition in manifest["system"]["joints"]:
            oid = definition["body1_id"]
            initial = manifest["initial_state"]["body_states"][oid]
            if initial["orientation_xyzw"] != [0.0, 0.0, 0.0, 1.0]:
                raise ValueError("World-slider initial frames currently require identity orientation")
            joint = UsdPhysics.PrismaticJoint.Define(stage, "/World/" + definition["joint_id"])
            joint.CreateBody1Rel().SetTargets([Sdf.Path("/World/" + oid)])
            # No body0 means the world frame, not a hidden kinematic actor.
            joint.CreateLocalPos0Attr(Gf.Vec3f(*initial["position_m"]))
            joint.CreateLocalPos1Attr(Gf.Vec3f(0.0))
            joint.CreateLocalRot0Attr(Gf.Quatf(1.0))
            joint.CreateLocalRot1Attr(Gf.Quatf(1.0))
            joint.CreateAxisAttr(definition["axis"])
            joint.CreateLowerLimitAttr(definition["lower_limit"])
            joint.CreateUpperLimitAttr(definition["upper_limit"])

        force_attr = None
        if controllers:
            controller = controllers[0]
            actuator_id = controller["actuator_instance_id"]
            if controller["implementation"] not in ("dynamic_body_effort", "dynamic_body_impedance") or controller["max_torque_nm"] is not None:
                raise ValueError("Push controller must declare finite external linear force")
            if not any(j["body1_id"] == actuator_id for j in manifest["system"]["joints"]):
                raise ValueError("Push actuator requires a declared guide")
            axis = next(j["axis"] for j in manifest["system"]["joints"] if j["body1_id"] == actuator_id)
            axis_index = {"X": 0, "Z": 2}[axis]
            force_api = PhysxSchema.PhysxForceAPI.Apply(actors[actuator_id]["prim"])
            force_attr = force_api.CreateForceAttr(Gf.Vec3f(0.0))
            force_api.CreateTorqueAttr(Gf.Vec3f(0.0))
            force_api.CreateModeAttr("force")
            force_api.CreateForceEnabledAttr(True)
            force_api.CreateWorldFrameEnabledAttr(True)

        step_index = 0
        contact_count = 0
        contacts = (output / "contacts.jsonl").open("x", encoding="utf-8")
        states = (output / "body_state_trace.jsonl").open("x", encoding="utf-8")
        streams.extend([contacts, states])
        if controllers:
            command_stream = (output / "command_trace.jsonl").open("x", encoding="utf-8")
            actuator_stream = (output / "actuator_state_trace.jsonl").open("x", encoding="utf-8")
            effort_stream = (output / "actuator_effort_trace.jsonl").open("x", encoding="utf-8")
            streams.extend([command_stream, actuator_stream, effort_stream])
            for command in program["commands"]:
                command_stream.write(json.dumps(dict(command, time_s=command["start_time_s"]), allow_nan=False) + "\n")

        def on_contact(headers, data):
            nonlocal contact_count
            for header in headers:
                paths = [str(PhysicsSchemaTools.intToSdfPath(getattr(header, k)))
                         for k in ("actor0", "actor1", "collider0", "collider1")]
                ids = [path.split("/")[2] for path in paths[:2]]
                # Flexible impulse is unavailable, never leaked into rigid supervision.
                if any(actors.get(oid, {}).get("soft") for oid in ids):
                    continue
                for index in range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data):
                    point = data[index]
                    contacts.write(json.dumps({
                        "time_s": step_index * dt, "physics_step": step_index, "paths": paths,
                        "actor_ids": ids,
                        "event_type": str(header.type), "position_m": list(map(float, point.position)),
                        "normal": list(map(float, point.normal)), "impulse_ns": list(map(float, point.impulse)),
                        "separation_m": float(point.separation), "source": "PhysX native contact report",
                    }, allow_nan=False) + "\n")
                    contact_count += 1

        simulation = get_physx_simulation_interface()
        subscription = simulation.subscribe_contact_report_events(on_contact)
        for _ in range(3):
            app.update()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        simulation.attach_stage(stage_id)
        attached = True
        get_physx_interface().force_load_physics_from_usd()
        tensor_simulation = tensors.create_simulation_view("numpy", stage_id)
        soft_tensor_simulation = causal_soft.initialize(stage_id, actors, output) if has_soft else None
        readback = {}
        for oid, actor in actors.items():
            if actor["body"] is None:
                continue
            view = tensor_simulation.create_rigid_body_view("/World/" + oid)
            if view.count != 1:
                raise ValueError(f"Cannot resolve native initial body: {oid}")
            initial = manifest["initial_state"]["body_states"][oid]
            # Native initialization ends before capture(0); no later direct state writes.
            view.set_velocities(np.asarray([initial["linear_velocity_m_s"] + initial["angular_velocity_rad_s"]],
                                           dtype=np.float32), np.asarray([0], dtype=np.int32))
            actor["tensor_view"] = view
            readback[oid] = {"velocity_m_s_rad_s": view.get_velocities()[0].tolist(),
                             "mass_kg": float(np.asarray(view.get_masses()).ravel()[0]),
                             "inertia_tensor_kg_m2": view.get_inertias()[0].tolist()}
        write_json(output / "native_initialization_readback.json", {
            "phase": "pre_t0_native_initialization", "source": "PhysX rigid-body tensor API",
            "bodies": readback,
        })

        def capture(step):
            cache = UsdGeom.XformCache(Usd.TimeCode.Default())
            body_states = {}
            for oid, actor in actors.items():
                if actor.get("soft"):
                    continue
                matrix = cache.GetLocalToWorldTransform(actor["prim"])
                transform = Gf.Transform(matrix)
                quat = transform.GetRotation().GetQuat()
                position = list(map(float, transform.GetTranslation()))
                rb = actor["body"]
                if rb:
                    native_pose = actor["tensor_view"].get_transforms()[0]
                    native_velocity = actor["tensor_view"].get_velocities()[0]
                    position = native_pose[:3].tolist()
                    quat = Gf.Quatd(float(native_pose[6]), Gf.Vec3d(*map(float, native_pose[3:6])))
                    linear, angular = native_velocity[:3].tolist(), native_velocity[3:].tolist()
                else:
                    linear, angular = [0.0] * 3, [0.0] * 3
                value = {"position_m": position, "orientation_xyzw": list(map(float, quat.GetImaginary())) + [float(quat.GetReal())],
                         "linear_velocity_m_s": linear, "angular_velocity_rad_s": angular}
                if rb:
                    local_angular = Gf.Rotation(quat).GetInverse().TransformDir(Gf.Vec3d(*angular))
                    matrix_inertia = actor["inertia_matrix"]
                    rotational_energy = (0.5 * float(np.asarray(local_angular) @ matrix_inertia @ np.asarray(local_angular))
                                         if matrix_inertia is not None else
                                         0.5 * sum(actor["inertia"][i] * float(local_angular[i])**2 for i in range(3)))
                    value.update(mass_kg=actor["mass"], inertia_diagonal_kg_m2=actor["inertia"],
                                 kinetic_energy_j=0.5 * actor["mass"] * sum(v*v for v in linear) +
                                     rotational_energy,
                                 potential_energy_j=-actor["mass"] * sum(gravity[i] * position[i] for i in range(3)))
                    if matrix_inertia is not None:
                        value["inertia_tensor_body_kg_m2"] = matrix_inertia.tolist()
                        value["principal_axes_xyzw"] = list(map(float,actor["prim"].GetAttribute("physics:principalAxes").Get().GetImaginary())) + [float(actor["prim"].GetAttribute("physics:principalAxes").Get().GetReal())]
                body_states[oid] = value
            for oid, actor in actors.items():
                if actor.get("soft"):
                    body_states[oid] = causal_soft.capture(stage, actor, cache, output, step, dt, gravity,
                        body_states.get(actuator_id) if controllers else None,
                        resolved["bodies"][actuator_id]["geometry"] if controllers else None,
                        numerics["contact_offset_m"])
            row = {"time_s": step * dt, "physics_step": step, "body_states": body_states}
            states.write(json.dumps(row, allow_nan=False) + "\n")
            if controllers:
                actuator_stream.write(json.dumps({"time_s": step * dt, "physics_step": step,
                    "controller_id": controller["controller_id"], "actuator_instance_id": actuator_id,
                    "state": body_states[actuator_id], "source": "PhysX native rigid-body tensors"}, allow_nan=False) + "\n")
            return row

        first = capture(0)
        write_json(output / "initial_state.json", {"time_s": 0.0, "body_states": {
            oid: {k: value[k] for k in ("position_m", "orientation_xyzw", "linear_velocity_m_s", "angular_velocity_rad_s")}
            for oid, value in first["body_states"].items()}})
        stage.GetRootLayer().Export(str(output / "native_initial.usda"))
        start = time.monotonic()
        previous = first
        cumulative_work = 0.0
        for step in range(round(timing["duration_s"] * hz)):
            step_index = step + 1
            if controllers:
                t = step * dt
                measured = previous["body_states"][actuator_id]
                decision = evaluate(controller, program["commands"], t,
                    measured["position_m"][axis_index], measured["linear_velocity_m_s"][axis_index])
                vector = [0.0]*3
                vector[axis_index] = decision["applied_force_n"]
                force_attr.Set(Gf.Vec3f(*vector))
            # No post-t0 velocity/pose writes, collision toggles, or actor changes.
            simulation.simulate(dt, step * dt)
            simulation.fetch_results()
            app.update()
            current = capture(step_index)
            if controllers:
                work = signed_work_increment(decision["applied_force_n"], measured["position_m"][axis_index],
                                              current["body_states"][actuator_id]["position_m"][axis_index])
                cumulative_work += work
                effort_stream.write(json.dumps(dict(decision, time_s=step_index * dt, physics_step=step_index,
                    interval_start_s=step * dt, controller_id=controller["controller_id"],
                    actuator_instance_id=actuator_id, axis=axis,
                    feedback_position_m=measured["position_m"][axis_index],
                    feedback_velocity_m_s=measured["linear_velocity_m_s"][axis_index],
                    applied_force_vector_n=vector,
                    work_increment_j=work, cumulative_work_j=cumulative_work,
                    source="bounded external force input; F*dx is derived work, not contact-force supervision"), allow_nan=False) + "\n")
            previous = current
        for stream in streams:
            stream.close()
        streams.clear()
        stage.GetRootLayer().Export(str(output / "native_final.usda"))
        if has_soft:
            evidence = {"status": "unavailable", "runtime": "Isaac Sim 6.0.1 / PhysX 110.1.13",
                "probe": "Public PhysxContactReportAPI enabled at rigid, soft root/collision and floor; explicit step callbacks",
                "reason": "No reliable public flexible-side point impulse export; flexible reports are excluded from rigid contact supervision",
                "rigid_point_count": contact_count, "substituted_soft_impulses": False,
                "geometric_contact_source": "sampled collision-node signed distance to guided axis-aligned plate; derived, not solver report"}
            write_json(output / "capability_probes/soft_contact_impulse.json", evidence)
        write_json(output / "native_report.json", {
            "status": "physics_completed", "runtime": "Isaac Sim 6.0.1 / PhysX 110.1.13",
            "elapsed_seconds": time.monotonic() - start, "physics_steps": step_index,
            "dt_s": dt, "numerics": numerics, "contact_points": contact_count,
            "state_update_authority": "solver_only_after_t0", "post_t0_control_writes": step_index if controllers else 0,
            "post_t0_direct_state_writes": 0, "control_write_kind": "bounded actuator force only" if controllers else "none",
            "state_source": "PhysX rigid-body tensors; native USD deformable surface/tet/nodal velocity; static fixtures from immutable USD" if has_soft else "PhysX rigid-body tensors; static fixtures from immutable USD",
            "mass_inertia_source": "explicit MassAPI authoring; state-derived energies are diagnostics, not calibrated physical truth",
        })
        print("CAUSAL_RIGID_NATIVE_COMPLETE", output, flush=True)
    except Exception as exc:
        traceback.print_exc()
        write_json(output / "native_failure.json", {"error": f"{type(exc).__name__}: {exc}", "traceback": traceback.format_exc()})
        raise
    finally:
        for stream in streams:
            stream.close()
        if attached:
            simulation.detach_stage()
        app.close()


if __name__ == "__main__":
    main()
