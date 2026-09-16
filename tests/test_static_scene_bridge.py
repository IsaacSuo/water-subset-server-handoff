import tempfile
from pathlib import Path
import unittest

import numpy as np

from world_model_dataset.io import file_hash
from world_model_dataset.static_scene_bridge import resolve_static_scene


class StaticSceneBridgeTests(unittest.TestCase):
    def test_keeps_open_world_mesh_and_winding(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'mesh.npz'
            vertices=np.array([[7,2,1],[8,2,1],[8,3,1]],dtype=np.float32)
            faces=np.array([[0,1,2]],dtype=np.int32)
            np.savez(path,vertices=vertices,triangles=faces)
            g=dict(static_scene_source=dict(path=str(path),sha256=file_hash(path)))
            initial=dict(position_m=[0,0,0],orientation_xyzw=[0,0,0,1])
            mesh=resolve_static_scene(g,dict(physics_kind='static'),initial,tmp)
            np.testing.assert_array_equal(mesh.vertices,vertices)
            np.testing.assert_array_equal(mesh.faces,faces)
            self.assertEqual(g['collision_approximation'],'none')
            with self.assertRaisesRegex(ValueError,'must be static'):
                resolve_static_scene(g,dict(physics_kind='rigid'),initial,tmp)
            with self.assertRaisesRegex(ValueError,'original world'):
                resolve_static_scene(g,dict(physics_kind='static'),dict(initial,position_m=[1,0,0]),tmp)
            g['static_scene_source']['sha256']='0'*64
            with self.assertRaisesRegex(ValueError,'hash mismatch'):
                resolve_static_scene(g,dict(physics_kind='static'),initial,tmp)


if __name__=='__main__':unittest.main()
