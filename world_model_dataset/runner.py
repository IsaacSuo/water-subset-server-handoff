"""CPU entrypoint. Validate and prepare first; native execution is explicit."""
from __future__ import annotations

import argparse
import json

from .contract import prepare, validate_episode, validate_pair
from .io import read_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("prepare")
    p.add_argument("spec")
    p.add_argument("output")
    p = sub.add_parser("validate")
    p.add_argument("manifest")
    p.add_argument("--require-complete", action="store_true")
    p.add_argument("--check-source", action="store_true")
    p = sub.add_parser("validate-pair")
    p.add_argument("baseline")
    p.add_argument("variant")
    p = sub.add_parser('legacy-inventory')
    p.add_argument('--config')
    p.add_argument('--runs')
    p.add_argument('--output')
    args = parser.parse_args()
    if args.command == "prepare":
        ep = prepare(args.spec,args.output)
        print(json.dumps({"episode_id":ep["episode_id"],"lifecycle":ep["lifecycle"],"output":args.output}))
    elif args.command == "validate":
        ep = validate_episode(args.manifest,args.require_complete,args.check_source)
        print(json.dumps({"valid":True,"episode_id":ep["episode_id"],"lifecycle":ep["lifecycle"]}))
    elif args.command == 'validate-pair':
        print(json.dumps({"valid":True,"changed":validate_pair(read_json(args.baseline),read_json(args.variant))}))
    else:
        from pathlib import Path
        from .legacy import inventory,DEFAULT_CONFIG,DEFAULT_RUNS
        value=inventory(Path(args.config) if args.config else DEFAULT_CONFIG,Path(args.runs) if args.runs else DEFAULT_RUNS)
        if args.output:
            from .io import write_json
            write_json(args.output,value)
        print(json.dumps({'valid':value['valid'],'episodes':value['episode_count'],'bodies':value['body_count'],'errors':value['errors']},ensure_ascii=False))
        if not value['valid']:raise SystemExit(1)


if __name__ == "__main__":
    main()
