"""Capture cached v0.2 observations; no automatic physical-behavior admission."""
import argparse
from pathlib import Path

from .causal_runner import invoke
from .io import read_json, write_json
from .probe_episode import artifact


def package_observations(output):
    output = Path(output)
    index = read_json(output / "observations/index.json")
    manifest = read_json(output / "episode.physics.json")
    manifest["trajectory"]["observations"] = artifact(output, "observations/index.json", "synchronized two-camera cache-only RTX RGB-D and segmentation")
    manifest["capabilities"]["rgb_depth_segmentation"] = {
        "status": "native", "source": index["render_backend"] + " sensor annotators on cached native states", "reason": None}
    # Observation production alone is not physical or dataset acceptance.
    write_json(output / "episode.observed.json", manifest)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    args = parser.parse_args()
    invoke("causal_render.py", args.episode, "observation.log")
    if (args.episode / "observations/failure.json").exists():
        raise RuntimeError("Observation backend reported failure")
    package_observations(args.episode)
    print("CAUSAL_OBSERVATIONS_PACKAGED", args.episode, flush=True)


if __name__ == "__main__":
    main()
