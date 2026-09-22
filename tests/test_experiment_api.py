"""Failure-oriented tests; integration tests read real, immutable native caches."""
import copy
from pathlib import Path
import tempfile
import unittest

from world_model_dataset.causal_loader import open_episode
from world_model_dataset.experiment_contract import (
    load_experiment, validate_common, native_config, rigid_spec, intervention)
from world_model_dataset.experiment_adapters import cache_binding
from world_model_dataset.experiment_adapters import material_call
from world_model_dataset.experiment_review import verify_alignment, beam_outcomes
from world_model_dataset.io import read_json, write_json, file_hash
from world_model_dataset.phenomenon_experiment import verify
from world_model_dataset.phenomenon_experiment import adopt_native_cache

ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = ROOT / 'configs/dataset/v0_2/experiment_api_v1'


class ContractTests(unittest.TestCase):
    def doc(self, name='roll'):
        return load_experiment(EXAMPLES / (name + '.json'))

    def test_wrong_backend_and_unimplemented_control_rejected(self):
        doc = self.doc(); doc['backend']['kind'] = 'rope'
        with self.assertRaisesRegex(ValueError, 'wrong backend'):
            validate_common(doc)
        for name in ('cloth', 'rope'):
            doc = self.doc(name); doc['control'] = 'dynamic_attachment'
            with self.assertRaisesRegex(ValueError, 'Unsupported actuator'):
                validate_common(doc)

    def test_unknown_input_not_silently_ignored(self):
        doc = self.doc(); doc['input']['participants'][0]['linear_velocity'] = [1, 0, 0]
        with self.assertRaisesRegex(ValueError, 'unknown'):
            rigid_spec(doc, 'test')
        doc = self.doc('cloth'); doc['input']['attachment'] = {'nodes': [0]}
        with self.assertRaisesRegex(ValueError, 'unknown'):
            native_config(doc)

    def test_observation_camera_and_time_fields_consumed_or_rejected(self):
        doc = self.doc(); doc['observations']['camera']['focal_length_mm'] = 99
        with self.assertRaisesRegex(ValueError, 'unknown'):
            validate_common(doc)
        doc = self.doc('cloth'); doc['observations']['hz'] = 24
        with self.assertRaisesRegex(ValueError, 'saved-state'):
            validate_common(doc)
        doc = self.doc(); doc['timing']['state_hz'] = 60
        with self.assertRaisesRegex(ValueError, '240 Hz'):
            validate_common(doc)

    def test_environment_movement_and_unit_conversion_rejected(self):
        doc = self.doc('cloth'); doc['scene']['collision']['meshes'][0]['scale'] = 1.1
        with self.assertRaisesRegex(ValueError, 'Environment transforms'):
            validate_common(doc)
        doc = self.doc(); doc['scene']['units'] = 'cm'
        with self.assertRaisesRegex(ValueError, 'metres'):
            validate_common(doc)

    def test_condition_cannot_change_scene_or_ignore_typo(self):
        base = self.doc('multibody'); base.pop('conditions')
        for path in ('/scene/units', '/input/participants/0/mass_typo'):
            with self.assertRaises(ValueError):
                intervention(base, dict(id='bad', changes={path: 1}, derived_impacts=['test']))
        with self.assertRaisesRegex(ValueError, 'derived_impacts'):
            intervention(base, dict(id='bad', changes={'/input/participants/1/clearance_m': .12}, derived_impacts=[]))
        changed, report = intervention(base, dict(id='lower', changes={'/input/participants/1/clearance_m': .12}, derived_impacts=['drop height']))
        self.assertEqual(changed['scene'], base['scene'])
        self.assertEqual(changed['input']['participants'][0], base['input']['participants'][0])
        self.assertEqual(report['changed_inputs'], ['/input/participants/1/clearance_m'])

    def test_duplicate_ids_and_mass_ambiguity_rejected(self):
        doc = self.doc('multibody'); doc['input']['participants'][1]['id'] = doc['input']['participants'][0]['id']
        with self.assertRaisesRegex(ValueError, 'Duplicate participant'):
            rigid_spec(doc, 'test')
        doc = self.doc(); doc['input']['participants'][0]['mass_kg'] = .1
        with self.assertRaisesRegex(ValueError, 'mass or density'):
            rigid_spec(doc, 'test')

    def test_resume_refuses_changed_dependency(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'prepared_input.json'; path.write_text('{}')
            pins = {str(path): file_hash(path)}
            path.write_text('{"velocity": 1}')
            with self.assertRaisesRegex(ValueError, 'changed'):
                verify(pins)

    def test_native_copy_rejects_change_after_cache_binding(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); source=root/'source'; native=source/'native'; native.mkdir(parents=True)
            record=native/'runtime.json'; record.write_text('{"version":1}')
            pinned={str(record):file_hash(record)}
            record.write_text('{"version":2}')
            destination=root/'destination'; destination.mkdir()
            with self.assertRaisesRegex(ValueError, 'changed during adoption'):
                adopt_native_cache(dict(cache={'source_run':str(source)}, cache_pins=pinned),destination)
            self.assertTrue((destination/'native/runtime.json').exists())
            self.assertFalse((destination/'cache_origin.json').exists())


@unittest.skipUnless(Path('/mnt/y/isaacsim_work').exists(), 'Local real-cache integration requires WSL data')
class NativeCacheTests(unittest.TestCase):
    def test_cached_packaging_preserves_original_native_input_context(self):
        from world_model_dataset.experiment_adapters import cache_package_context
        doc = load_experiment(EXAMPLES / 'beam.json'); cache = doc['conditions'][0]['cache']
        source = Path(cache['run'])
        original_hash = file_hash(source/'prepared/config.json')
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); current = root/'current'; (current/'native').mkdir(parents=True)
            # This regression only needs the real native config: the complete
            # beam arrays are separately exercised by the actual API workflow.
            import shutil
            shutil.copyfile(source/'native/config.json', current/'native/config.json')
            ctx = cache_package_context(current, dict(source_run=str(source)), root/'context')
            self.assertEqual(read_json(ctx/'prepared/config.json'), read_json(ctx/'native/config.json'))
            self.assertEqual(file_hash(source/'prepared/config.json'), original_hash)

    def test_material_preparation_retains_identical_particle_realization(self):
        import numpy as np
        doc = load_experiment(EXAMPLES / 'plastic.json')
        source = Path(doc['conditions'][0]['cache']['run'])
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); config = root / 'config.json'
            write_json(config, native_config(doc))
            arrays = []
            for i in range(2):
                out = root / ('prepared_' + str(i))
                material_call(doc, out, 'prepare', root / ('prepare_'+str(i)+'.log'), config)
                with np.load(out/'prepared/initial.npz') as z:
                    arrays.append(z['points'].copy())
            np.testing.assert_array_equal(arrays[0], arrays[1])
            with np.load(source/'prepared/initial.npz') as z:
                np.testing.assert_array_equal(arrays[0], z['points'])

    def test_real_beam_actual_loading_and_withdrawal(self):
        doc = load_experiment(EXAMPLES / 'beam.json'); cache = doc['conditions'][0]['cache']
        ep = open_episode(Path(cache['run']) / Path(cache['manifest']).parent, require_complete=False)
        result = verify_alignment(ep, doc['timing'])
        self.assertEqual(result['counts']['commands'], 960)
        actual = beam_outcomes(ep)
        self.assertTrue(actual['hold_every_sample_bottom_proximity'])
        self.assertTrue(actual['physical_withdrawal_observed'])
        self.assertGreater(actual['actual_plate_downward_travel_m'], .05)
        self.assertEqual(actual['contact_force'], 'unavailable')

    def test_real_cached_states_reject_wrong_duration_and_frequency(self):
        doc = load_experiment(EXAMPLES / 'rope.json'); cache = doc['conditions'][0]['cache']
        ep = open_episode(Path(cache['run']) / 'episode', require_complete=False)
        verify_alignment(ep, doc['timing'])
        wrong = dict(doc['timing'], physics_hz=240)
        with self.assertRaisesRegex(ValueError, 'physics_step'):
            verify_alignment(ep, wrong)
        wrong = dict(doc['timing'], duration_s=1.)
        with self.assertRaisesRegex(ValueError, 'duration'):
            verify_alignment(ep, wrong)

    def test_real_cache_binding_refuses_new_end_constraint(self):
        doc = load_experiment(EXAMPLES / 'rope.json'); cache = doc['conditions'][0]['cache']
        source = Path(cache['run'])
        # Only the modified prepared request is local; source cache remains read-only.
        with tempfile.TemporaryDirectory() as tmp:
            local = Path(tmp); (local/'prepared').mkdir()
            cfg = read_json(source/'prepared/config.json')
            cfg['rope']['end_condition'] = 'both_clamped'
            write_json(local/'prepared/config.json', cfg)
            with self.assertRaisesRegex(ValueError, 'different physical inputs'):
                cache_binding(doc, local, cache)


if __name__ == '__main__':
    unittest.main()
