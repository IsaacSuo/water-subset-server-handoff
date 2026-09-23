"""CPU-only rejection checks for external collision provenance."""
import tempfile,unittest
from pathlib import Path
from world_model_dataset.experiment_adapters import cache_binding
from world_model_dataset.io import write_json,file_hash

class WrapCacheTests(unittest.TestCase):
    def test_same_mesh_cannot_hide_changed_object_identity(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);a=root/'current';b=root/'external'
            for path in (a,b):
                (path/'prepared').mkdir(parents=True)
                write_json(path/'prepared/config.json',{'kind':'rope'})
            write_json(b/'episode.json',{})
            write_json(a/'prepared/rod_collision_policy.json',{'objects':[{'name':'rod'}]})
            write_json(b/'prepared/rod_collision_policy.json',{'objects':[{'name':'other'}]})
            cache=dict(run=str(b),manifest='episode.json',manifest_sha256=file_hash(b/'episode.json'))
            with self.assertRaisesRegex(ValueError,'policy/source identity'):
                cache_binding({'backend':{'kind':'rope'}},a,cache)

    def test_identical_policy_admitted_without_starting_solver(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);a=root/'current';b=root/'external'
            for path in (a,b):
                (path/'prepared').mkdir(parents=True)
                write_json(path/'prepared/config.json',{'kind':'rope'})
                write_json(path/'prepared/rod_collision_policy.json',{'objects':[{'name':'rod'}]})
            (b/'episode').mkdir();write_json(b/'episode/episode.json',{})
            cache=dict(run=str(b),manifest='episode/episode.json',manifest_sha256=file_hash(b/'episode/episode.json'))
            result=cache_binding({'backend':{'kind':'rope'}},a,cache)
            self.assertFalse(result['physics_rerun'])
            self.assertEqual(result['episode'],str(b/'episode'))

if __name__=='__main__':unittest.main()
