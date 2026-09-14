import tempfile
import unittest
from pathlib import Path

from world_model_dataset.m4 import ensure_r03_specs,generate_r03


class M4Tests(unittest.TestCase):
    def test_r03_matrix(self):
        rows,pairs=generate_r03()
        self.assertEqual(len(rows),11);self.assertEqual(len(pairs),8)
        self.assertEqual({row['objects'][0]['object_id'] for row in rows},{'sphere','rounded_cube','cylinder'})
        self.assertEqual(sum(row['episode_id'].endswith('lateral_miss') for row in rows),1)

    def test_r03_specs_are_immutable(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)/'r03';self.assertEqual(ensure_r03_specs(root),ensure_r03_specs(root))


if __name__=='__main__':unittest.main()
