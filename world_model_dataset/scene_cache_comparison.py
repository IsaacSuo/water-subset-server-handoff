"""Stack synchronized original-scene replays without changing physical frames."""
import argparse
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from .io import file_hash, read_json, write_json


def compose(plan, output):
    panels = plan['panels']
    if len(panels) < 2:
        raise ValueError('A comparison needs at least two panels')
    sources = []
    for panel in panels:
        folder = Path(panel['render']).resolve()
        replay = read_json(folder/'replay.json')
        state = Path(replay['state_path'])
        if file_hash(state) != replay['state_sha256']:
            raise ValueError('Physical cache changed: '+str(state))
        sources.append(dict(label=panel['label'], render=str(folder),
                            replay_path=str(folder/'replay.json'),
                            replay_sha256=file_hash(folder/'replay.json'), replay=replay))
    first = sources[0]['replay']
    times = [f['time_s'] for f in first['frames']]
    for source in sources[1:]:
        replay = source['replay']
        if [f['time_s'] for f in replay['frames']] != times:
            raise ValueError('Comparison panels must use identical physical timestamps')
        for key in ('camera', 'resolution', 'fps', 'source_sha256', 'keyframes_only'):
            if replay[key] != first[key]:
                raise ValueError('Comparison panels differ in '+key)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    bundle = output/'comparison_sources.json'
    write_json(bundle, dict(kind='independent_episode_comparison', sources=sources,
                           comparison=plan.get('comparison', {})))
    width, height = first['resolution']
    font = ImageFont.truetype(plan.get('font', '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'), 16)
    records = []
    for frame, time in enumerate(times):
        canvas = Image.new('RGB', (width, height*len(panels)), '#142b36')
        for i, source in enumerate(sources):
            record = source['replay']['frames'][frame]
            with Image.open(Path(source['render'])/record['path']) as image:
                if image.size != (width, height):
                    raise ValueError('Frame dimensions differ from replay metadata')
                canvas.paste(image.convert('RGB'), (0, i*height))
            draw = ImageDraw.Draw(canvas)
            text = source['label']
            right = 20 + draw.textlength(text, font=font)
            draw.rectangle((8, i*height+6, right, i*height+33), fill='#142b36')
            draw.text((14, i*height+7), text, font=font, fill='#ffffff')
            if i:
                draw.line((0, i*height, width, i*height), fill='#9bd3cb', width=2)
        path = f'frame_{frame:04d}.png'
        canvas.save(output/path)
        records.append(dict(frame=frame, time_s=time, path=path,
                            panel_state_indices=[s['replay']['frames'][frame]['state_index'] for s in sources]))
    write_json(output/'replay.json', dict(id=plan['id'], kind='synchronized_comparison',
        state_path=str(bundle), state_sha256=file_hash(bundle),
        state_path_semantics='Bundle of independent physical state traces and source replay records; not a new simulated episode.',
        cache_sources=[dict(state_path=s['replay']['state_path'], state_sha256=s['replay']['state_sha256']) for s in sources],
        source_blend=first['source_blend'], source_sha256=first['source_sha256'],
        source_lighting_preserved=all(s['replay']['source_lighting_preserved'] for s in sources),
        camera=first['camera'], resolution=[width, height*len(panels)], fps=first['fps'],
        keyframes_only=first['keyframes_only'], frames=records, physics_rerun=False,
        display_operation='Native rendered pixels stacked vertically with labels; no resampling, interpolation or motion editing.',
        comparison=plan.get('comparison', {})))
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print('COMPOSED', compose(read_json(args.plan), args.output), flush=True)


if __name__ == '__main__':
    main()
