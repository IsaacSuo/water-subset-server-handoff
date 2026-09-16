"""Native volume-deformable authoring and full-step caches for causal prototypes.

Uses the corrected root-only material binding from native_mixed; no soft state edits.
Imports Kit dependencies only inside native execution.
"""
from __future__ import annotations

from .io import write_json
from .probe_episode import artifact


def author(stage, oid, definition, initial, numerics):
    import numpy as np
    from pxr import Gf, Sdf, UsdGeom, UsdShade, PhysxSchema
    from omni.physx.scripts import deformableUtils, physicsUtils
    geometry, profile = definition["geometry"], definition["physics"]
    if geometry["shape"] != "soft_box" or initial["orientation_xyzw"] != [0, 0, 0, 1]:
        raise ValueError("First soft backend supports axis-aligned soft boxes only")
    if any(initial["linear_velocity_m_s"] + initial["angular_velocity_rad_s"]):
        raise ValueError("Soft initial nodal velocity initialization is not implemented; nonzero input cannot be ignored")
    root = "/World/" + oid
    xf = UsdGeom.Xform.Define(stage, root)
    xf.AddTranslateOp().Set(Gf.Vec3d(*initial["position_m"]))
    vertices = np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],
                         [-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]], dtype=float) * np.array(geometry["size_m"]) / 2
    triangles = np.array([[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],
                          [1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]], dtype=np.int32)
    visual = UsdGeom.Mesh.Define(stage, root + "/Visual")
    visual.CreatePointsAttr([Gf.Vec3f(*v) for v in vertices])
    visual.CreateFaceVertexCountsAttr([3] * len(triangles))
    visual.CreateFaceVertexIndicesAttr(triangles.ravel().tolist())
    visual.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    visual.CreateDisplayColorAttr([Gf.Vec3f(*definition["appearance"]["color"])])
    material_path = "/World/Material_" + oid
    if not deformableUtils.add_deformable_material(stage, material_path, density=profile["density_kg_m3"],
        static_friction=profile["static_friction"], dynamic_friction=profile["dynamic_friction"],
        youngs_modulus=profile["youngs_modulus_pa"], poissons_ratio=profile["poissons_ratio"]):
        raise RuntimeError("Deformable material creation failed")
    material_prim = stage.GetPrimAtPath(material_path)
    material_prim.ApplyAPI("PhysxDeformableMaterialAPI")
    damping = material_prim.GetAttribute("physxDeformableMaterial:elasticityDamping")
    if not damping or not damping.Set(profile["elasticity_damping"]):
        raise RuntimeError("Elasticity damping unavailable")
    simulation_path, collision_path = root + "/Simulation", root + "/Collision"
    if not deformableUtils.create_auto_volume_deformable_hierarchy(stage, Sdf.Path(root),
        Sdf.Path(simulation_path), Sdf.Path(collision_path), visual.GetPath(), True, True, True):
        raise RuntimeError("Auto volume hierarchy failed")
    prim = xf.GetPrim()
    mass = float(np.prod(geometry["size_m"]) * profile["density_kg_m3"])
    for name, value in {"physxDeformableBody:resolution": geometry["deformable_resolution"],
        "omniphysics:mass": mass, "physxDeformableBody:remeshingEnabled": True,
        "physxDeformableBody:forceConforming": True, "physxDeformableBody:targetTriangleCount": 0}.items():
        attr = prim.GetAttribute(name)
        if not attr or not attr.Set(value):
            raise RuntimeError("Unavailable deformable attribute: " + name)
    prim.ApplyAPI("PhysxBaseDeformableBodyAPI")
    for name, value in {"linearDamping": profile["linear_damping"], "settlingDamping": 0.0,
        "solverPositionIterationCount": numerics["deformable_position_iterations"],
        "selfCollision": False, "enableSpeculativeCCD": True}.items():
        attr = prim.GetAttribute("physxDeformableBody:" + name)
        if not attr or not attr.Set(value):
            raise RuntimeError("Unavailable deformable attribute: " + name)
    # Binding only at the root is essential for native material registration.
    physicsUtils.add_physics_material_to_prim(stage, prim, Sdf.Path(material_path))
    collision = stage.GetPrimAtPath(collision_path)
    from pxr import UsdPhysics
    UsdPhysics.CollisionAPI.Apply(collision)
    collision_api = PhysxSchema.PhysxCollisionAPI.Apply(collision)
    collision_api.CreateContactOffsetAttr(numerics["contact_offset_m"])
    collision_api.CreateRestOffsetAttr(numerics["rest_offset_m"])
    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
    PhysxSchema.PhysxContactReportAPI.Apply(collision).CreateThresholdAttr(0.0)
    return dict(prim=prim, body=None, mass=mass, inertia=None, soft=True, oid=oid,
        simulation_path=simulation_path, collision_path=collision_path, visual=visual,
        triangles=triangles, material_path=material_path, profile=profile, rest=None)


def initialize(stage_id, actors, output):
    import omni.physics.tensors as tensors
    view = tensors.create_simulation_view("warp", stage_id)
    results = {}
    for oid, actor in actors.items():
        if not actor.get("soft"):
            continue
        actor["tensor_body"] = view.create_volume_deformable_body_view("/World/" + oid)
        material = view.create_deformable_material_view(actor["material_path"] + "*")
        readback = dict(status="native", count=material.count,
            youngs_modulus_pa=float(material.get_youngs_modulus().numpy()[0,0]),
            poissons_ratio=float(material.get_poissons_ratio().numpy()[0,0]),
            dynamic_friction=float(material.get_dynamic_friction().numpy()[0,0]))
        for key in ("youngs_modulus_pa", "poissons_ratio", "dynamic_friction"):
            if abs(readback[key] - actor["profile"][key]) > max(1e-6, abs(actor["profile"][key])*1e-6):
                raise RuntimeError("Native soft material readback mismatch: " + key)
        results[oid] = readback
    write_json(output / "native_soft_material_readback.json", {"runtime": "Isaac Sim 6.0.1 / PhysX 110.1.13", "bodies": results})
    return view


def sampled_rigid_contacts(points_world, rigid_shapes, contact_offset):
    """Node/analytic-shape distances only; not solver contacts, forces or exact penetration."""
    import numpy as np
    result = {}
    for oid, (state, geometry) in rigid_shapes.items():
        delta = points_world - np.asarray(state["position_m"])
        if geometry["shape"] == "sphere":
            signed = np.linalg.norm(delta, axis=1) - geometry["radius_m"]
        elif geometry["shape"] == "box":
            x,y,z,w = np.asarray(state["orientation_xyzw"])/np.linalg.norm(state["orientation_xyzw"])
            rotation = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                                 [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                                 [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
            q = np.abs(delta@rotation) - np.asarray(geometry["size_m"])/2
            signed = np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(q.max(axis=1),0)
        else:
            result[oid] = {"status": "unavailable", "reason": "No geometric distance adapter for this collision representation"}
            continue
        gap = float(signed.min())
        result[oid] = {"status": "derived", "minimum_node_gap_m": gap,
                       "sampled_penetration_m": max(0.0,-gap), "near_contact": gap<=contact_offset,
                       "source": "collision nodes versus analytic rigid shape; sampled lower bound, not solver report"}
    return result


def capture(stage, actor, cache, output, step, dt, gravity, actuator_state, actuator_geometry, contact_offset, rigid_shapes=None):
    import numpy as np
    from pxr import UsdGeom
    from soft_body.tet_quality import compute_tet_deformation, signed_tetrahedron_volumes
    mesh = UsdGeom.TetMesh.Get(stage, actor["simulation_path"])
    points = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float64)
    tets = np.asarray(mesh.GetTetVertexIndicesAttr().Get(), dtype=np.int32).reshape(-1,4)
    if actor["rest"] is None:
        bind = mesh.GetPrim().GetAttribute("deformablePose:default:omniphysics:points").Get()
        if bind is None:
            raise RuntimeError("No native bind pose")
        actor["rest"] = np.asarray(bind, dtype=np.float64)
        actor["tets"] = tets.copy()
        weights = np.zeros(len(points))
        volumes = np.abs(signed_tetrahedron_volumes(actor["rest"], tets))
        np.add.at(weights, tets.ravel(), np.repeat(volumes/4,4))
        actor["weights"] = weights/weights.sum()
        actor["rest_volume"] = float(volumes.sum())
        actor["rest_height"] = float(np.ptp(actor["rest"][:,2]))
        write_json(output / (actor["oid"] + ".topology.json"), {"simulation_tets": tets.tolist(),
            "surface_triangles": actor["triangles"].tolist(), "simulation_bind_points_m": actor["rest"].tolist()})
        actor["topology_record"] = artifact(output, actor["oid"] + ".topology.json", "immutable native bind and simulation/surface topology")
    if not np.array_equal(tets, actor["tets"]):
        raise RuntimeError("Simulation topology changed")
    nodal = None
    for name in ("omniphysics:velocities", "velocities"):
        attr = mesh.GetPrim().GetAttribute(name)
        values = attr.Get() if attr else None
        if values is not None and len(values) == len(points):
            nodal = np.asarray(values, dtype=np.float64)
            break
    if nodal is None:
        if step == 0:
            nodal = np.zeros_like(points)  # Explicitly supported zero-velocity initial condition only.
        else:
            nodal = actor["tensor_body"].get_simulation_nodal_velocities().numpy()[0,:len(points)].astype(np.float64)
    def world(mesh, values):
        matrix = np.asarray(cache.GetLocalToWorldTransform(mesh.GetPrim()))
        return values @ matrix[:3,:3] + matrix[3,:3]
    simulation_world = world(mesh, points)
    surface = world(actor["visual"], np.asarray(actor["visual"].GetPointsAttr().Get(), dtype=np.float64))
    collision_mesh = UsdGeom.TetMesh.Get(stage, actor["collision_path"])
    collision = world(collision_mesh, np.asarray(collision_mesh.GetPointsAttr().Get(), dtype=np.float64))
    metrics = compute_tet_deformation(points, tets, actor["rest"])
    metrics["volume_ratio"] = float(np.abs(signed_tetrahedron_volumes(points,tets)).sum())/actor["rest_volume"]
    rest_centered = actor["rest"] - actor["weights"] @ actor["rest"]
    current_centered = points - actor["weights"] @ points
    u, _, vh = np.linalg.svd(rest_centered.T @ (actor["weights"][:,None] * current_centered))
    sign = np.eye(3)
    sign[2,2] = np.linalg.det(u@vh)
    residual = current_centered - rest_centered @ (u@sign@vh)
    metrics["nonrigid_rms_m"] = float(np.sqrt(actor["weights"] @ (residual*residual).sum(axis=1)))
    metrics["maximum_local_displacement_m"] = float(np.linalg.norm(residual,axis=1).max())
    metrics["axis_z_compression_fraction"] = 1 - float(np.ptp(simulation_world[:,2]))/actor["rest_height"]
    metrics["height_m"] = float(np.ptp(simulation_world[:,2]))
    metrics["bounds_world_m"] = [simulation_world.min(axis=0).tolist(), simulation_world.max(axis=0).tolist()]
    metrics["axis_extents_m"] = np.ptp(simulation_world,axis=0).tolist()
    if rigid_shapes is not None:
        metrics["geometric_contacts"] = sampled_rigid_contacts(collision, rigid_shapes, contact_offset)
    metrics["maximum_nodal_speed_m_s"] = float(np.linalg.norm(nodal,axis=1).max())
    if actuator_state is not None:
        # Diagnostic sampling at native collision nodes; not exact mesh intersection.
        centre = np.array(actuator_state["position_m"])
        half = np.array(actuator_geometry["size_m"])/2
        q = np.abs(collision-centre)-half
        signed = np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(np.max(q,axis=1),0)
        metrics["sampled_actuator_gap_m"] = float(signed.min())
        metrics["sampled_actuator_penetration_m"] = max(0.0,-float(signed.min()))
        metrics["geometric_contact"] = bool(signed.min() <= contact_offset)
    linear = actor["weights"] @ nodal
    centre = actor["weights"] @ simulation_world
    path = f"soft_state/{actor['oid']}_{step:05d}.npz"
    (output / "soft_state").mkdir(exist_ok=True)
    with (output / path).open("xb") as stream:
        np.savez_compressed(stream, surface_world_m=surface.astype(np.float32), surface_triangles=actor["triangles"],
            simulation_world_m=simulation_world.astype(np.float32), simulation_points_m=points.astype(np.float32),
            simulation_tets=tets, simulation_nodal_velocities_m_s=nodal.astype(np.float32),
            collision_world_m=collision.astype(np.float32), time_s=step*dt, physics_step=step)
    root_position = np.asarray(cache.GetLocalToWorldTransform(actor["prim"]))[3,:3].tolist()
    return {"position_m": root_position, "orientation_xyzw": [0,0,0,1],
        "linear_velocity_m_s": linear.tolist(), "angular_velocity_rad_s": [0,0,0] if step==0 else None,
        "pose_semantics": "immutable USD root frame, not rigid soft-body pose; COM/deformation recorded separately",
        "centre_of_mass_m": centre.tolist(), "mass_kg": actor["mass"],
        "kinetic_energy_j": .5*actor["mass"]*float(actor["weights"] @ (nodal*nodal).sum(axis=1)),
        "potential_energy_j": -actor["mass"]*float(np.array(gravity)@centre),
        "geometry": artifact(output,path,"native soft surface, simulation tet, collision nodes and native nodal velocities"),
        "topology": actor["topology_record"],
        "metrics": metrics}
