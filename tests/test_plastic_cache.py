"""Validate diagnostic intervals and preserve external execution provenance; no physics."""
import tempfile,unittest
from pathlib import Path
from unittest.mock import Mock
import numpy as np
from world_model_dataset.causal_loader import CausalEpisode
from world_model_dataset.experiment_adapters import cache_package_context
from world_model_dataset.io import write_json,read_json

class PlasticCacheTests(unittest.TestCase):
    def reader(self,path,offsets=(0,0,1),ids=(0,)):
        np.savez(path,time=np.array([0,.1,.2]),contact_step_offsets=np.array(offsets),contact_impulse=np.array([[0.,0.,-1.]]),contact_position=np.array([[0.,0.,0.]]),contact_collider_id=np.array(ids))
        ep=object.__new__(CausalEpisode);ep.resolved_inputs=Mock(return_value=dict(config=dict(kind='plastic',state_hz=10,physics_hz=10,environment=[{'id':'original_table'}]),raw_native={}))
        ep.record_path=Mock(return_value=path);return ep
    def test_impulse_intervals_are_not_force_labels(self):
        with tempfile.TemporaryDirectory() as td:
            rows=list(self.reader(Path(td)/'raw.npz').material_contact_diagnostics())
            self.assertEqual([r['physics_step'] for r in rows],[1,2]);self.assertEqual(len(rows[0]['collider_ids']),0)
            self.assertFalse(rows[1]['force_truth']);self.assertFalse(rows[1]['supervision_admitted'])
            self.assertEqual(rows[1]['start_time_s'],.1)
            np.testing.assert_array_equal(rows[1]['impulse_on_colliders_Ns'],[[0,0,-1]])
    def test_bad_offsets_and_unknown_colliders_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            for offsets,ids in [((0,2,1),(0,)),((0,0,1),(1,))]:
                with self.assertRaises(ValueError):list(self.reader(Path(td)/'raw.npz',offsets,ids).material_contact_diagnostics())
    def test_packaging_preserves_external_overlay_binding(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);source=root/'source';current=root/'current'
            (source/'prepared').mkdir(parents=True);(current/'native').mkdir(parents=True)
            for name in ('source_binding.json','import_origin.json'):
                write_json(source/name,dict(external_run='material',physics_rerun=False))
            result=cache_package_context(current,{'source_run':str(source)},root/'package')
            for name in ('source_binding.json','import_origin.json'):self.assertEqual(read_json(result/name),read_json(source/name))

if __name__=='__main__':unittest.main()
