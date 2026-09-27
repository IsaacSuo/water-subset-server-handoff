"""Resolve the handed-off Windows/WSL asset paths without rewriting source data."""
from __future__ import annotations

import hashlib
from pathlib import Path


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_handoff_path(value: str | Path, root: Path) -> Path:
    """Map Y:/isaacsim_work and /mnt/y/isaacsim_work into this checkout."""
    root = Path(root).resolve()
    raw = str(value)
    normalized = raw.replace("\\", "/")
    direct = Path(normalized)
    if direct.is_file() or direct.is_dir():
        return direct.resolve()

    lowered = normalized.lower()
    marker = "isaacsim_work/"
    if marker in lowered:
        offset = lowered.index(marker) + len(marker)
        return (root / normalized[offset:]).resolve()

    if "/scenes/hdri/" in lowered or lowered.startswith("y:/scenes/hdri/"):
        return resolve_hdri(root, Path(normalized).name)
    return direct


def resolve_hdri(root: Path, name: str = "bryanston_park_sunrise_8k.exr") -> Path:
    matches = sorted((Path(root) / "external_assets").glob(f"**/{name}"))
    if len(matches) != 1:
        raise FileNotFoundError(f"Expected exactly one handed-off HDRI named {name}, found {len(matches)}")
    return matches[0].resolve()


def mapping_entry(label: str, source: str | Path, resolved: Path) -> dict:
    resolved = Path(resolved).resolve()
    entry = {"label": label, "source": str(source), "resolved": str(resolved), "exists": resolved.exists()}
    if resolved.is_file():
        entry.update(bytes=resolved.stat().st_size, sha256=sha256_file(resolved))
    return entry
