import copy
import json
import tempfile
import unittest
from pathlib import Path

from world_model_dataset.contract import CONFIG, prepare, resolve, validate_episode, validate_pair, validate_schema
from world_model_dataset.io import file_hash, inside, read_json
from world_model_dataset.actions import due, sample_trajectory, compile_actions
from world_model_dataset.finalize import checks_accepted
from world_model_dataset.loader import Episode
from world_model_dataset.legacy import inventory


class ContractTests(unittest.TestCase):
    def setUp(self):
        self.spec = read_json(CONFIG/"examples/r01_sphere.json")

    def test_prepare_and_read_only_validate(self):
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp)/"episode"
            prepare(CONFIG/"examples/r01_sphere.json",out)
            before = {p.name:p.stat().st_mtime_ns for p in out.iterdir()}
            validate_episode(out/"episode.prepared.json",check_source=True)
            self.assertEqual(before,{p.name:p.stat().st_mtime_ns for p in out.iterdir()})
            with self.assertRaises(ValueError):
                validate_episode(out/"episode.prepared.json",require_complete=True)
            with self.assertRaises(FileExistsError):
                prepare(CONFIG/"examples/r01_sphere.json",out)

    def test_reject_bad_inputs(self):
        mutations = [lambda x:x.update(extra=1), lambda x:x.update(seed=float('nan')),
                     lambda x:x["objects"][0].update(physics_profile_id="missing"),
                     lambda x:x["objects"][0].update(physics_profile_id="elastic_soft"),
                     lambda x:x["objects"][0].update(orientation_xyzw=[0,0,0,2]),
                     lambda x:x["timing"].update(physics_hz=241),
                     lambda x:x["action_parameters"].update(release_time_s=0.5001),
                     lambda x:x["fixture_parameters"].update(angle_deg=89)]
        for mutate in mutations:
            value=copy.deepcopy(self.spec);mutate(value)
            with self.subTest(value=value), self.assertRaises(Exception):resolve(value)

    def test_path_confinement(self):
        with tempfile.TemporaryDirectory() as temp:
            for path in ('../x','/x','C:/x','a\\b','a/../b','./b','a//b'):
                with self.subTest(path=path), self.assertRaises(ValueError):inside(temp,path)
            (Path(temp)/"link").symlink_to('/tmp',target_is_directory=True)
            with self.assertRaises(ValueError):inside(temp,'link/outside')

    def test_duplicates_and_nonfinite_json(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'x.json'
            for content in ('{"a":1,"a":2}','{"a":NaN}','{"a":1e999}'):
                p.write_text(content)
                with self.assertRaises(ValueError):read_json(p)

    def test_checksum_tampering(self):
        with tempfile.TemporaryDirectory() as temp:
            out=Path(temp)/'episode';prepare(CONFIG/'examples/r01_sphere.json',out)
            with (out/'action.json').open('a') as stream:stream.write(' ')
            with self.assertRaises(ValueError):validate_episode(out/'episode.prepared.json')

    def test_effective_sources_are_archived(self):
        with tempfile.TemporaryDirectory() as temp:
            out=Path(temp)/'episode';ep=prepare(CONFIG/'examples/r01_sphere.json',out)
            from world_model_dataset.io import file_hash
            for relative,sha in ep['source_files'].items():
                self.assertEqual(file_hash(out/'source'/relative),sha)

    def test_frozen_v0_1_contract_hashes(self):
        release=read_json(CONFIG/'contract_v0_1_release.json')
        self.assertEqual(release['status'],'frozen')
        self.assertEqual(release['contract_version'],'0.1.0')
        for relative,expected in release['files'].items():
            self.assertEqual(file_hash(CONFIG/relative),expected,relative)

    def test_forged_resolved_reference_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            out=Path(temp)/'episode';ep=prepare(CONFIG/'examples/r01_sphere.json',out)
            from world_model_dataset.io import digest
            ep['inputs']['objects'][0]['object_id']='capsule'
            ep['inputs_sha256']=digest(ep['inputs'])
            (out/'episode.prepared.json').write_text(json.dumps(ep))
            with self.assertRaises(ValueError):validate_episode(out/'episode.prepared.json')

    def test_strict_counterfactual(self):
        v=copy.deepcopy(self.spec)
        v['episode_id']='r01_sphere_low_friction'
        v['objects'][0]['physics_profile_id']='rigid_low_friction'
        v['counterfactual'].update(baseline_episode_id=self.spec['episode_id'],changed_pointer='/objects/0/physics/dynamic_friction')
        self.assertEqual(len(validate_pair(self.spec,v)),1)
        v['seed']+=1
        with self.assertRaises(ValueError):validate_pair(self.spec,v)

    def test_action_timeline(self):
        spec=read_json(CONFIG/'examples/v02_cube.json');resolved=resolve(spec)
        from world_model_dataset.geometry import make_geometry
        from world_model_dataset.fixtures import build_fixture
        fixture=build_fixture(spec,resolved,make_geometry(resolved['objects'][0]['geometry']).vertices)
        actions=compile_actions(spec,resolved,fixture)
        validate_schema(actions,'action')
        self.assertTrue(all(set(c['parameters'])=={'interpolation','from_m','to_m'} for c in actions['commands']))
        self.assertAlmostEqual(sample_trajectory(actions['commands'][0],2.)[2],.17)
        self.assertAlmostEqual(sample_trajectory(actions['commands'][-1],3.)[2],.30)
        command=compile_actions(self.spec,resolve(self.spec),{})['commands'][0]
        self.assertEqual([s for s in range(241) if due(command,s,240)],[120])

    def test_r03_two_body_extension(self):
        spec=read_json(CONFIG/'examples/r03_equal_spheres.json')
        resolved=resolve(spec)
        self.assertEqual(resolved['event']['id'],'R03')
        self.assertEqual(len(resolved['objects']),2)
        from world_model_dataset.geometry import make_geometry
        from world_model_dataset.fixtures import build_fixture
        vertices={o['instance_id']:make_geometry(o['geometry']).vertices for o in resolved['objects']}
        fixture=build_fixture(spec,resolved,vertices)
        self.assertEqual(fixture['subject_positions_m']['left'][0],-.4)
        self.assertEqual(fixture['subject_positions_m']['right'][0],.4)
        actions=compile_actions(spec,resolved,fixture)
        validate_schema(actions,'action')
        self.assertEqual([c['target'] for c in actions['commands']],['left','right'])
        self.assertEqual(actions['commands'][0]['parameters']['linear_m_s'],[1.5,0.,0.])
        self.assertEqual(actions['commands'][1]['parameters']['linear_m_s'],[-1.5,0.,0.])
        invalid=copy.deepcopy(spec);invalid['objects']=invalid['objects'][:1]
        with self.assertRaisesRegex(ValueError,'requires exactly 2'):resolve(invalid)

    def test_numerics_separate_from_material(self):
        a=read_json(CONFIG/'examples/v02_cube.json');b=copy.deepcopy(a)
        b['numerics_profile_id']='contact_convergence_128'
        left,right=resolve(a),resolve(b)
        self.assertEqual(left['objects'],right['objects'])
        from world_model_dataset.contract import differences
        self.assertEqual(differences(left['numerics'],right['numerics']),['/deformable_position_iterations'])

    def test_unavailable_capability_is_not_a_failed_episode(self):
        self.assertTrue(checks_accepted([
            {'name':'shape','status':'pass','detail':'ok'},
            {'name':'soft_contact_impulse','status':'unavailable','detail':'public backend has no output'},
        ],[]))
        self.assertFalse(checks_accepted([{'name':'shape','status':'fail','detail':'bad'}],[]))
        self.assertFalse(checks_accepted([{'name':'shape','status':'pass','detail':'ok'}],['observations']))

    def test_loader_rejects_unavailable_soft_impulse(self):
        episode=Episode.__new__(Episode)
        episode.manifest={'capabilities':{
            'rigid_contact_impulse':{'status':'not_applicable','source':'callback','reason':None},
            'soft_contact_impulse':{'status':'unavailable','source':'public capability probe','reason':'no reliable output'}}}
        with self.assertRaisesRegex(RuntimeError,'unavailable'):
            next(episode.contacts())
        with self.assertRaisesRegex(RuntimeError,'undeclared'):
            episode.capability('unknown_capability')

    def test_legacy_bridge_quarantines_missing_contact_truth(self):
        bridge=inventory()
        self.assertTrue(bridge['valid'],bridge['errors'])
        self.assertEqual(bridge['episode_count'],14)
        self.assertEqual(bridge['body_count'],42)
        self.assertEqual(bridge['capabilities']['point_contact_impulse']['status'],'unavailable')
        self.assertEqual({r['environment_id'] for r in bridge['rows']},
                         {'alley','apartment','bedroom','city','classroom','elevator','factory','garage',
                          'graffiti_warehouse','hospital','mountain','subway2','swamp','warehouse'})


if __name__=='__main__':unittest.main()
