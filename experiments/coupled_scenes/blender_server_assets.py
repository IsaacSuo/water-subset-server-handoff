"""Helpers used inside Blender; path remapping is intentionally in-memory only."""
from pathlib import Path

import bpy


def reload_hdri(path: Path) -> list[str]:
    path = Path(path).resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    remapped = []
    for image in bpy.data.images:
        source = image.filepath.replace("\\", "/")
        if Path(source).name == path.name:
            image.filepath = str(path)
            image.reload()
            remapped.append(image.name)
    if not remapped:
        raise RuntimeError(f"No image datablock references handed-off HDRI {path.name}")
    print(f"[server-assets] remapped HDRI in memory: {remapped} -> {path}", flush=True)
    return remapped
