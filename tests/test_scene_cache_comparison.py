import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from PIL import Image, ImageFont

from world_model_dataset.io import file_hash, read_json, write_json
from world_model_dataset.scene_cache_comparison import compose


class SceneCacheComparisonTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.plan = dict(id='pair', panels=[])
        self.font = ImageFont.load_default()
        for name, color in [('flat', 'red'), ('upright', 'blue')]:
            folder = self.root/name
            folder.mkdir()
            state = folder/'states.jsonl'
            state.write_text('{"time_s": 0}\n')
            Image.new('RGB', (64, 48), color).save(folder/'frame_0000.png')
            write_json(folder/'replay.json', dict(state_path=str(state), state_sha256=file_hash(state),
                camera={'position_m': [0, -1, 1]}, resolution=[64, 48], fps=30,
                source_sha256='scene-hash', source_blend='original.blend',
                source_lighting_preserved=True, keyframes_only=False,
                frames=[dict(frame=0, state_index=0, time_s=0., path='frame_0000.png')]))
            self.plan['panels'].append(dict(label=name, render=str(folder)))

    def test_preserves_pixels_and_both_source_hashes(self):
        with patch('world_model_dataset.scene_cache_comparison.ImageFont.truetype', return_value=self.font):
            out = compose(self.plan, self.root/'composed')
        with Image.open(out/'frame_0000.png') as image:
            self.assertEqual(image.size, (64, 96))
            self.assertEqual(image.getpixel((60, 44)), (255, 0, 0))
            self.assertEqual(image.getpixel((60, 92)), (0, 0, 255))
        replay = read_json(out/'replay.json')
        self.assertEqual(len(replay['cache_sources']), 2)
        self.assertEqual(replay['state_sha256'], file_hash(Path(replay['state_path'])))
        self.assertEqual(replay['frames'][0]['panel_state_indices'], [0, 0])

    def test_rejects_mismatched_physical_time_and_camera(self):
        path = self.root/'upright/replay.json'
        original = read_json(path)
        changed = read_json(path)
        changed['frames'][0]['time_s'] = .1
        path.write_text(json.dumps(changed))
        with self.assertRaisesRegex(ValueError, 'timestamps'):
            compose(self.plan, self.root/'bad-time')
        original['camera'] = {'position_m': [1, -1, 1]}
        path.write_text(json.dumps(original))
        with self.assertRaisesRegex(ValueError, 'camera'):
            compose(self.plan, self.root/'bad-camera')
        self.assertFalse((self.root/'bad-time').exists())
        self.assertFalse((self.root/'bad-camera').exists())

    def test_rejects_mutated_physical_cache(self):
        (self.root/'flat/states.jsonl').write_text('modified')
        with self.assertRaisesRegex(ValueError, 'cache changed'):
            compose(self.plan, self.root/'bad-cache')


if __name__ == '__main__':
    unittest.main()
