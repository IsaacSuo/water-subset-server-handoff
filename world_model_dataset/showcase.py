"""Render and encode M5A presentation slices from accepted native caches."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
from pathlib import Path

from .contract import ROOT
from .io import read_json


DEFAULT_CONFIG = ROOT / "configs/dataset/m5a_observation_slices.json"
DEFAULT_OUTPUT = ROOT / "output/world_model_dataset/v0_1/m5a_showcase_v01"


def windows_path(path: Path) -> str:
    return subprocess.check_output(["wslpath", "-w", str(path.resolve())], text=True).strip()


def sha256(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def select_slices(config, only, priority):
    result = config["slices"]
    if only:
        wanted = set(only)
        result = [row for row in result if row["id"] in wanted]
        missing = wanted - {row["id"] for row in result}
        if missing:
            raise ValueError(f"Unknown M5A slice IDs: {sorted(missing)}")
    if priority:
        result = [row for row in result if row["priority"] == priority]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--only", nargs="*")
    parser.add_argument("--priority", choices=["real_asset_delivery", "event_representative"])
    parser.add_argument("--view", default="hero")
    parser.add_argument("--pilot-frames", type=int, default=0)
    parser.add_argument("--force-render", action="store_true")
    args = parser.parse_args()
    config = read_json(args.config)
    slices = select_slices(config, args.only, args.priority)
    if not slices:
        raise ValueError("No M5A slices selected")
    blender = Path(os.environ.get("BLENDER_BIN", "/mnt/d/Program Files (x86)/Blender/blender.exe"))
    ffmpeg = shutil.which("ffmpeg")
    ffprobe = shutil.which("ffprobe")
    if not blender.is_file():
        raise FileNotFoundError(blender)
    if not ffmpeg or not ffprobe:
        raise RuntimeError("ffmpeg and ffprobe are required")
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for selection in slices:
        episode = ROOT / selection["episode"]
        manifest = read_json(episode / "episode.prepared.json")
        validation = read_json(episode / "physics_validation.json")
        if not validation["passed"]:
            raise ValueError(f"Rejected physics cache: {selection['id']}")
        if manifest["spec"]["event_id"] != selection["event_id"]:
            raise ValueError(f"Event mismatch: {selection['id']}")
        destination = args.output / selection["id"]
        report_path = destination / "render_report.json"
        expected = int(round((selection["end_s"] - selection["start_s"]) * config["output"]["fps"])) + 1
        if args.pilot_frames:
            expected = min(expected, args.pilot_frames)
        if args.force_render or not report_path.is_file() or len(list((destination / "frames").glob(f"{args.view}_*.png"))) != expected:
            prior_report_mtime = report_path.stat().st_mtime_ns if report_path.is_file() else None
            command = [
                str(blender), "--background", "--python", windows_path(ROOT / "world_model_dataset/showcase_render_blender.py"), "--",
                "--config", windows_path(args.config), "--slice-id", selection["id"], "--episode", windows_path(episode),
                "--output", windows_path(destination), "--view", args.view
            ]
            if args.pilot_frames:
                command += ["--frame-count", str(args.pilot_frames)]
            print("M5A_BLENDER_START " + selection["id"], flush=True)
            subprocess.run(command, cwd=ROOT, check=True)
            if not report_path.is_file():
                raise RuntimeError(f"Blender exited without a render report for {selection['id']}")
            if prior_report_mtime is not None and report_path.stat().st_mtime_ns <= prior_report_mtime:
                raise RuntimeError(f"Blender exited without updating the render report for {selection['id']}")
        report = read_json(report_path)
        frame_count = len(report["renders"])
        if frame_count != expected:
            raise RuntimeError(f"Unexpected rendered frame count for {selection['id']}: {frame_count}/{expected}")
        video = destination / (f"{selection['id']}_{args.view}_pilot.mp4" if args.pilot_frames else f"{selection['id']}_{args.view}.mp4")
        temporary = video.with_name("." + video.name + ".part.mp4")
        subprocess.run([
            ffmpeg, "-hide_banner", "-loglevel", "error", "-xerror", "-y",
            "-framerate", str(config["output"]["fps"]), "-start_number", "0",
            "-i", str(destination / "frames" / f"{args.view}_%04d.png"), "-frames:v", str(frame_count),
            "-c:v", "libx264", "-preset", "medium", "-crf", str(config["output"]["crf"]),
            "-pix_fmt", config["output"]["pixel_format"], "-movflags", "+faststart", str(temporary)
        ], check=True)
        os.replace(temporary, video)
        probe = json.loads(subprocess.check_output([
            ffprobe, "-v", "error", "-count_frames", "-show_entries",
            "stream=codec_name,width,height,pix_fmt,r_frame_rate,nb_read_frames:format=duration", "-of", "json", str(video)
        ], text=True))
        stream = probe["streams"][0]
        width, height = config["output"]["resolution"]
        checks = {
            "codec": stream["codec_name"] == "h264",
            "resolution": [int(stream["width"]), int(stream["height"])] == [width, height],
            "pixel_format": stream["pix_fmt"] == config["output"]["pixel_format"],
            "fps": stream["r_frame_rate"] == f"{config['output']['fps']}/1",
            "frames": int(stream["nb_read_frames"]) == frame_count
        }
        if not all(checks.values()):
            raise RuntimeError(f"Video validation failed for {selection['id']}: {checks}")
        result = {
            "slice_id": selection["id"], "event_id": selection["event_id"], "priority": selection["priority"],
            "source_episode_id": manifest["spec"]["episode_id"], "view": args.view, "frame_count": frame_count,
            "duration_s": float(probe["format"]["duration"]), "video": str(video.relative_to(ROOT)),
            "video_bytes": video.stat().st_size, "video_sha256": sha256(video), "checks": checks
        }
        (destination / "video_report.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        results.append(result)
        print("M5A_VIDEO_COMPLETE " + selection["id"] + " " + str(video), flush=True)
    # The launcher is routinely called in small resumable subsets.  Keep the
    # root summary cumulative so a final no-render pass produces one index for
    # the complete ten-event slice instead of describing only the last call.
    cumulative = {}
    for path in sorted(args.output.glob("*/video_report.json")):
        row = read_json(path)
        if Path(ROOT / row["video"]).is_file() and all(row["checks"].values()):
            cumulative[row["slice_id"]] = row
    expected_ids = {row["id"] for row in config["slices"]}
    summary = {
        "schema_version": "0.1.0", "valid": set(cumulative) == expected_ids,
        "complete_event_coverage": set(cumulative) == expected_ids,
        "event_ids": sorted({row["event_id"] for row in cumulative.values()}),
        "config": str(args.config.relative_to(ROOT)),
        "config_sha256": sha256(args.config), "physics_rerun": False,
        "results": [cumulative[key] for key in sorted(cumulative)]
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
