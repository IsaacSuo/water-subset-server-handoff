"""Development-only bridge to exploration's parameterized geometry packages.

Preparation calls the supplied, hash-pinned loader; native simulation consumes
only the episode-local resolved mesh. No scene or physics authoring is imported.
"""
from pathlib import Path
import hashlib
import types

import numpy as np
import trimesh

from .io import file_hash, read_json


def resolve_asset(geometry, physics, root):
    source = geometry["physical_asset_source"]
    package = (Path(root) / source["directory"]).resolve()
    loader = (Path(root) / source["loader_path"]).resolve()
    loader_bytes = loader.read_bytes()
    digest = hashlib.sha256(loader_bytes).hexdigest()
    if digest != source["loader_sha256"]:
        raise ValueError("Exploration loader changed; review and pin the new revision explicitly")
    metadata = read_json(package / "asset.json")
    if metadata["format"] != "physical-asset-source/1":
        raise ValueError("Unsupported physical asset source format")
    module = types.ModuleType("physical_asset_source_loader")
    module.__file__ = str(loader)
    exec(compile(loader_bytes, str(loader), "exec"), module.__dict__)
    parameters = dict(source["size"])
    if not parameters or set(parameters) - {"max_extent_m", "extents_m"}:
        raise ValueError("Asset size must specify max_extent_m or extents_m")
    mass, density = physics.get("mass_kg"), physics.get("density_kg_m3")
    if (mass is None) == (density is None):
        raise ValueError("Physical asset recipe requires exactly one explicit mass or density")
    parameters.update(mass_kg=mass, density_kg_m3=density,
                      static_friction=physics["static_friction"],
                      dynamic_friction=physics["dynamic_friction"],
                      restitution=physics["restitution"], sdf_resolution=256)
    variant = module.load_variant(package, **parameters)
    if not np.allclose(variant["center_of_mass_m"], 0, atol=1e-12):
        raise ValueError("Current mainline mesh path requires COM-centered vertices")
    collision = variant["collision"]
    if collision["method"] != "sdf":
        raise ValueError("This source bridge does not replace SDF with a convex approximation")
    mesh = trimesh.Trimesh(variant["vertices"], variant["triangles"], process=False)
    physics.update(mass_kg=variant["mass_kg"], density_kg_m3=variant["density_kg_m3"])
    geometry.update(asset_id=variant["asset_id"], collision_approximation="sdf",
                    sdf_resolution=collision["resolution"], sdf_subgrid_resolution=collision["subgrid"],
                    sdf_bits_per_subgrid_pixel="BitsPerPixel"+str(collision["bits"]),
                    sdf_triangle_count_reduction_factor=collision["triangle_reduction"],
                    center_of_mass_m=variant["center_of_mass_m"].tolist(),
                    inertia_tensor_kg_m2=variant["inertia_kg_m2"].tolist(),
                    bounds_m=mesh.bounds.tolist(),
                    rest_volume_m3=variant["mass_kg"]/variant["density_kg_m3"],
                    source_sha256=variant["source_sha256"],
                    physical_asset_provenance={
                        "status": "development_integration_not_asset_admission",
                        "source_package": str(package), "asset_manifest": metadata,
                        "asset_manifest_sha256": file_hash(package/"asset.json"),
                        "loader_path": str(loader), "loader_sha256": digest,
                        "loader_source": loader_bytes.decode("utf-8"), "parameters": parameters,
                        "collision": collision, "portable_native_cooked_data": False,
                        "parameter_range_validation": "not_established",
                        "runtime_dependency": "episode-local mesh and resolved inputs only"})
    return mesh
