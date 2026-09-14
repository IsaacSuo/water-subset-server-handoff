"""Strict portable JSON, deterministic hashes and confined artifact paths."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path, PurePosixPath


def _pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON key: {key}")
        result[key] = value
    return result


def finite(value):
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("Non-finite JSON value")
    if isinstance(value, dict):
        for item in value.values():
            finite(item)
    elif isinstance(value, (list, tuple)):
        for item in value:
            finite(item)


def read_json(path):
    value = json.loads(Path(path).read_text(encoding="utf-8"), object_pairs_hook=_pairs)
    finite(value)
    return value


def canonical(value):
    finite(value)
    return json.dumps(value, sort_keys=True, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")


def digest(value):
    return hashlib.sha256(canonical(value)).hexdigest()


def file_hash(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def inside(root, relative):
    p = PurePosixPath(relative)
    if not relative or "\\" in relative or ":" in relative or p.is_absolute() or ".." in p.parts:
        raise ValueError(f"Non-portable or escaping path: {relative}")
    if str(p) != relative or relative == ".":
        raise ValueError(f"Non-canonical path: {relative}")
    root = Path(root).resolve()
    result = (root / relative).resolve()
    if not result.is_relative_to(root):
        raise ValueError(f"Path escapes root through symlink: {relative}")
    return result


def write_json(path, value):
    """Runtime output only; never replace a published manifest implicitly."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    finite(value)
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
