"""No rendering/physics/preparation: native existing streams and isolated records."""
import copy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from world_model_dataset.causal_loader import open_episode
from world_model_dataset.experiment_contract import load_experiment, validate_common, native_config
from world_model_dataset.experiment_material_bridge import map_existing
from world_model_dataset.experiment_review import verify_alignment
from world_model_dataset.io import read_json, write_json, file_hash
from world_model_dataset.phenomenon_catalog import build
from world_model_dataset.phenomenon_experiment import run

ROOT=Path(__file__).resolve().parents[1]
CONFIG=ROOT/'configs/dataset/v0_2/construction'


class BoundaryTests(unittest.TestCase):
    def test_register_requires_current_catalog_before_execution(self):
        with tempfile.TemporaryDirectory() as temp, patch('world_model_dataset.experiment_adapters.prepare') as prep:
            with self.assertRaisesRegex(ValueError,'current-catalog'):
                run(ROOT/'configs/dataset/v0_2/experiment_api_v1/cloth.json',Path(temp)/'out',stage='register')
            prep.assert_not_called()

    def test_catalog_preserves_entries_and_supersession(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            def entry(name):
                ep=root/name; ep.mkdir(); write_json(ep/'episode.json',{})
                return dict(id=name,episode=str(ep),manifest_sha256=file_hash(ep/'episode.json'),
                            source_states_sha256='hash',states=2,observations=1,group=1)
            current=entry('current'); old=entry('old'); new=entry('new')
            history=[dict(old_episode=old,replacement_id='current',reason='prior')]
            write_json(root/'base.json',dict(episodes=[current],superseded=history))
            write_json(root/'new.json',dict(episodes=[new]))
            class Episode:
                manifest={'trajectory':{'states':{'sha256':'hash'}}}
            with patch('world_model_dataset.phenomenon_catalog.open_episode',return_value=Episode()):
                result=build([root/'base.json',root/'new.json'],[],root/'combined.json')
            self.assertEqual(result['episodes'],[current,new]); self.assertEqual(result['superseded'],history)


@unittest.skipUnless(Path('/mnt/y/isaacsim_work').exists(),'read-only local B/C sources require WSL')
class ExistingMaterialTests(unittest.TestCase):
    def test_B_public_mapping_preserves_200kPa_and_three_conditions(self):
        docs={}
        for mode in ('free','blocked','low'):
            doc,report=map_existing(**read_json(CONFIG/('bridge_b_'+mode+'.json')))
            self.assertEqual(doc['input']['material']['youngs_modulus_Pa'],200000)
            self.assertTrue(report['physical_input_preserved'])
            self.assertEqual(report['construction_owner'],'experiment entry line')
            self.assertEqual(report['construction'],'forward_only')
            docs[mode]=doc
        self.assertEqual(docs['free']['input']['gripper']['max_force_n'],4)
        self.assertEqual(docs['low']['input']['gripper']['max_force_n'],.2)
        self.assertIn('opposing_fixture',docs['blocked']['input'])

    def test_C_disabled_semantics_and_exact_revision(self):
        doc,report=map_existing(**read_json(CONFIG/'bridge_c_disabled.json'))
        self.assertEqual(doc['control'],dict(mode='impedance_control',enabled=False))
        self.assertEqual(doc['actuation'],dict(kind='finite_load',instance_ids=['front_load']))
        self.assertIs(doc['input']['load_control']['enabled'],False)
        self.assertEqual(doc['input']['load_control']['displacement_m'],-.24)
        self.assertEqual(report['source_commit'],'05aabd376dc32916bb5541406ca67378da9fbc72')
        self.assertEqual(report['backend'],'Newton SolverVBD')
        self.assertEqual(report['construction_owner'],'experiment entry line')
        self.assertEqual(report['construction'],'forward_only')
        self.assertEqual(report['executed_stages'],[])
        wrong=copy.deepcopy(doc); wrong['control']='finite_load'
        with self.assertRaisesRegex(ValueError,'Unsupported actuator'): validate_common(wrong)

    def test_existing_B_C_multirate_reading(self):
        pairs=[('/mnt/y/isaacsim_work_cloth_gripper/output/cloth_gripper_substeps/depen005_hold5s/episode',
                dict(duration_s=5.,physics_hz=1200,state_hz=120),601,601,6000),
               ('/mnt/y/isaacsim_work_rope_load/output/rope_load_c/c_travel24_v1/episode',
                dict(duration_s=3.,physics_hz=960,state_hz=60),181,2880,2880)]
        for folder,timing,states,actual,commands in pairs:
            ep=open_episode(folder,require_complete=False)
            result=verify_alignment(ep,timing)
            self.assertEqual(result['counts']['states'],states)
            self.assertEqual(result['counts']['actuator_states'],actual)
            self.assertEqual(result['counts']['commands'],commands)
            self.assertFalse(result['interpolation'])


if __name__=='__main__': unittest.main()
