"""Missing observations are explicit; truncated/failed physics cannot sneak in."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from world_model_dataset.phenomenon_catalog import open_catalog_episode

class PhysicsOnlyTests(unittest.TestCase):
    def fixture(self):
        available=dict(status='available')
        manifest=dict(lifecycle='draft',initial_state=dict(state=available),
            trajectory=dict(states=available,interaction_annotations=available,outcomes=available,
                            observations=dict(status='unavailable')),
            control_program=dict(primitive='none',controllers=[]),timing=dict(duration_s=2.))
        ep=SimpleNamespace(manifest=manifest,record_path=lambda r: None,
                           states=lambda:iter([dict(time_s=0.),dict(time_s=2.)]))
        return dict(episode='test',record_scope='physics_only',states=2,observations=0),ep

    def test_explicit_absent_observations_are_readable(self):
        item,ep=self.fixture()
        with patch('world_model_dataset.phenomenon_catalog.open_episode',return_value=ep):
            self.assertIs(open_catalog_episode(item),ep)

    def test_failed_prefix_rejected(self):
        item,ep=self.fixture();ep.states=lambda:iter([dict(time_s=0.),dict(time_s=1.)])
        with patch('world_model_dataset.phenomenon_catalog.open_episode',return_value=ep):
            with self.assertRaisesRegex(ValueError,'incomplete in time'):open_catalog_episode(item)

    def test_rejected_manifest_stays_rejected(self):
        item,ep=self.fixture();ep.manifest['lifecycle']='rejected'
        with patch('world_model_dataset.phenomenon_catalog.open_episode',return_value=ep):
            with self.assertRaisesRegex(ValueError,'lifecycle'):open_catalog_episode(item)

    def test_default_keeps_completed_reader(self):
        with patch('world_model_dataset.phenomenon_catalog.open_episode') as reader:
            open_catalog_episode(dict(episode='test'))
            reader.assert_called_once_with('test')
