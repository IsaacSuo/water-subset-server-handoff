"""Simple phenomenon-led vertical, rotational, multibody and soft examples."""
import copy

from .causal_runner import ROOT, load_config
from .io import read_json


KINDS = {"falling", "balance", "support_edge", "collision_chain", "soft_confinement", "soft_impact"}


def add_body(config, manifest, oid, geometry, profile_id, position, velocity=None, kind="rigid", appearance="matte_blue", role="subject"):
    config["geometry_profiles"][oid] = copy.deepcopy(geometry)
    body = dict(instance_id=oid, role=role, physics_kind=kind, geometry_id=oid,
                physics_profile_id=profile_id, appearance_profile_id=appearance,
                physics_presence="continuous", collision_participation="continuous", render_presence="continuous")
    manifest["system"]["bodies"].append(body)
    manifest["initial_state"]["participant_ids"].append(oid)
    manifest["initial_state"]["body_states"][oid] = dict(position_m=position, orientation_xyzw=[0,0,0,1],
                linear_velocity_m_s=velocity or [0,0,0], angular_velocity_rad_s=[0,0,0])
    if role=="environment":
        manifest["environment"]["environment_body_ids"].append(oid)


def build(design, variant, asset):
    kind = design["kind"]
    active = kind in {"balance", "soft_confinement"}
    soft = kind in {"soft_confinement", "soft_impact"}
    config = load_config(ROOT / "configs/dataset/v0_2" / ("c2_rigid_push.json" if active else "c2_none_collision.json"))
    manifest = read_json(ROOT / config["manifest_template"])
    config["initial_state_overrides"] = {}
    config.pop("body_profile_overrides", None)
    manifest["system"]["bodies"] = []
    manifest["initial_state"]["participant_ids"] = []
    manifest["initial_state"]["body_states"] = {}
    manifest["environment"]["environment_body_ids"] = []
    config["prototype_id"] = "phenomenon_"+kind
    manifest["environment"]["environment_id"] = "self_built_"+kind
    config["timing"]["duration_s"] = 5.0 if kind=="soft_confinement" else 4.0 if kind=="balance" else 3.0
    manifest["timing"] = copy.deepcopy(config["timing"])
    reference = copy.deepcopy(next(p for p in config["physics_profiles"].values() if p.get("mass_kg") is not None))
    config["physics_profiles"]["subject"] = dict(reference, mass_kg=asset["mass_kg"], restitution=0.0, static_friction=.3, dynamic_friction=.2)
    config["physics_profiles"]["ground"] = dict(reference, mass_kg=None, restitution=0.0, static_friction=.3, dynamic_friction=.2)
    ground = config["physics_profiles"]["ground"]
    material = config["physics_profiles"]["subject"]
    shape = {k:copy.deepcopy(v) for k,v in asset.items() if k!="mass_kg"}
    dimensions = shape.get("size_m", [2*shape.get("radius_m",0)]*3)
    add_body(config,manifest,"floor",{"shape":"box","size_m":[8.0,2.0,.05]},"ground",[0,0,-.025],kind="static",appearance="matte_gray",role="environment")
    if kind=="falling":
        material["restitution"] = ground["restitution"] = .6
        add_body(config,manifest,"left",shape,"subject",[0,0,variant["height_m"]+dimensions[2]/2],appearance="matte_orange")
    elif kind=="support_edge":
        material.update(static_friction=.2,dynamic_friction=.1)
        ground.update(static_friction=.2,dynamic_friction=.1)
        add_body(config,manifest,"platform",{"shape":"box","size_m":[1.2,1.0,.12]},"ground",[-.6,0,.39],kind="static",appearance="matte_gray",role="environment")
        add_body(config,manifest,"left",shape,"subject",[-.45,0,.45+dimensions[2]/2],[variant["speed_m_s"],0,0],appearance="matte_orange")
    elif kind=="collision_chain":
        material.update(restitution=.8,static_friction=0,dynamic_friction=0)
        ground.update(static_friction=0,dynamic_friction=0)
        for i,color in enumerate(([.95,.4,.08],[.1,.45,.85],[.2,.65,.45],[.65,.35,.75])):
            config["appearance_profiles"]["chain_"+str(i)] = dict(color=color,roughness=.5,metallic=0)
            add_body(config,manifest,"ball"+str(i),shape,"subject",[-.65+i*(dimensions[0]+variant["gap_m"]),0,dimensions[2]/2],
                     [1.2 if i==0 else 0,0,0],appearance="chain_"+str(i))
    elif kind=="balance":
        material.update(static_friction=.6,dynamic_friction=.5)
        ground.update(static_friction=.6,dynamic_friction=.5)
        add_body(config,manifest,"load",shape,"subject",[0,0,dimensions[2]/2])
    if soft:
        soft_config = load_config(ROOT / "configs/dataset/v0_2/c2_soft_compression.json")
        soft_manifest = read_json(ROOT / soft_config["manifest_template"])
        manifest["capabilities"]["soft_contact_impulse"] = copy.deepcopy(soft_manifest["capabilities"]["soft_contact_impulse"])
        config["numerics"] = soft_config["numerics"]
        config["physics_profiles"]["elastic"] = soft_config["physics_profiles"]["elastic_soft"]
        config["physics_profiles"]["elastic"]["density_kg_m3"] = asset["mass_kg"]/(dimensions[0]*dimensions[1]*dimensions[2])
        add_body(config,manifest,"soft",shape,"elastic",[0,0,dimensions[2]/2],kind="volumetric")
        if kind=="soft_impact":
            config["physics_profiles"]["projectile"] = dict(reference,mass_kg=1.0,static_friction=0,dynamic_friction=0,restitution=0)
            add_body(config,manifest,"projectile",{"shape":"sphere","radius_m":.1},"projectile",[-.7,0,.1],[variant["speed_m_s"],0,0],appearance="matte_orange")
        else:
            config["physics_profiles"]["wall"] = dict(ground,static_friction=.1,dynamic_friction=.1)
            for oid,sign in (("wall_left",-1),("wall_right",1)):
                add_body(config,manifest,oid,{"shape":"box","size_m":[.12,.25,.4]},"wall",[.3,sign*(variant["gap_m"]/2+.125),.2],kind="static",appearance="matte_gray",role="environment")
    if active:
        if kind=="balance":
            pad = [.08,.24,.10]; position=[-.34,0,variant["push_height_m"]]
            force,speed,gain,start,end = 15.0,.25,80.0,.2,2.2
        else:
            pad = [.08,.12,.24]; position=[-.35,0,.14]
            force,speed,gain,start,end = 80.0,.3,1000.0,.5,3.5
        add_body(config,manifest,"pusher",{"shape":"box","size_m":pad},"actuator_1kg",position,appearance="matte_orange",role="actuator")
        controller = manifest["control_program"]["controllers"][0]
        controller["max_force_n"] = force
        command = manifest["control_program"]["commands"][0]
        command.update(start_time_s=start,end_time_s=end,target={"velocity_m_s":speed,"velocity_gain_n_s_m":gain},limits={"max_force_n":force})
        manifest["implementation"]["post_t0_operations"][0]["time_s"] = start
        if kind=="soft_confinement":
            # Reuse the tested C2 impedance rather than an excessively stiff
            # discrete velocity loop on a one-kilogram free actuator.
            controller.update(primitive="impedance_control",implementation="dynamic_body_impedance",
                              stiffness_n_m=6000.0,damping_n_s_m=120.0)
            manifest["control_program"]["primitive"] = "impedance_control"
            manifest["control_program"]["commands"] = [
                dict(command_id="initial_hold",controller_id=controller["controller_id"],start_time_s=0.0,end_time_s=.5,
                     target={"position_m":position[0],"velocity_m_s":0.0},limits={"max_force_n":force}),
                dict(command_id="advance",controller_id=controller["controller_id"],start_time_s=.5,end_time_s=3.5,
                     target={"trajectory":"smoothstep","position_start_m":position[0],"position_end_m":.6},limits={"max_force_n":force})]
            manifest["implementation"]["post_t0_operations"][0]["time_s"] = 0.0
    return config,manifest
