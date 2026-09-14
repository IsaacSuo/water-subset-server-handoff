import tempfile
import unittest
from pathlib import Path

from world_model_dataset.prototypes import generate,write_specs,ensure_specs
from world_model_dataset.io import read_json
from world_model_dataset.contract import CONFIG


class PrototypeTests(unittest.TestCase):
    def test_matrix_and_pairs(self):
        rows,pairs=generate()
        self.assertEqual(len(rows),35);self.assertEqual(len({r['episode_id'] for r in rows}),35)
        self.assertEqual(sum(r['event_id']=='R01' for r in rows),20)
        self.assertEqual(sum(r['event_id']=='V02' for r in rows),15)
        # R01: 4 geometries x (2 angle pairs + 1 friction pair) = 12.
        # V02: 3 compression levels x (2 modulus pairs + 1 speed pair) = 9.
        self.assertEqual(len(pairs),21)
        self.assertTrue(all(r['numerics_profile_id']=='contact_convergence_128' for r in rows if r['event_id']=='V02'))
        selected=read_json(CONFIG/'prototypes/M3.json')['cycles_selected_episode_ids']
        self.assertEqual(len(selected),3);self.assertTrue(set(selected)<={r['episode_id'] for r in rows})

    def test_write_is_create_only(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'matrix';write_specs(path)
            self.assertEqual(len(list((path/'specs').glob('*.json'))),35)
            with self.assertRaises(FileExistsError):write_specs(path)

    def test_resume_requires_identical_matrix(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'matrix'
            self.assertEqual(ensure_specs(path),ensure_specs(path))
            spec=path/'specs/r01_sphere_angle20.json'
            spec.write_text(spec.read_text().replace('20.0','21.0',1))
            with self.assertRaises(ValueError):ensure_specs(path)


if __name__=='__main__':unittest.main()
