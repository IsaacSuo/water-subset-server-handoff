from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

from world_model_dataset.causal_window import read_window


class WindowTests(unittest.TestCase):
    def episode(self):
        rows=[dict(time_s=k*.5,physics_step=k,body_states={}) for k in range(3)]
        manifest=dict(timing=dict(physics_hz=2),control_program=dict(primitive='none',controllers=[]),
                      trajectory=dict(observations=dict(status='unavailable')))
        return SimpleNamespace(manifest=manifest,states=lambda:iter(rows),controls=lambda:iter([]))

    def test_closed_window_and_none_control_do_not_invent_records(self):
        window=read_window(self.episode(),.25,1.)
        self.assertEqual(window['sample_interval_s'],[.5,1.])
        self.assertEqual(window['controls'],[]);self.assertEqual(window['observations'],[])
        self.assertEqual(window['geometries'],{});self.assertEqual(window['actuator_states'],{})
        with self.assertRaisesRegex(ValueError,'outside'):read_window(self.episode(),0,2)

    def test_terminal_state_does_not_fabricate_next_command(self):
        ep=self.episode();ep.manifest['control_program']=dict(primitive='impedance_control',controllers=[dict(controller_id='c')])
        command=dict(time_s=.5,physics_step=1,target_position_m=[9,0,0])
        actual=dict(time_s=1.,physics_step=2,position_m=[1,0,0])
        ep.controls=lambda:iter([command]);ep.actuator_states=lambda oid:iter([actual]);ep.actuator_efforts=lambda oid:iter([])
        window=read_window(ep,.5,1.)
        self.assertEqual(window['controls'],[command]);self.assertEqual(window['actuator_states']['c'],[actual])
        self.assertEqual(window['actuator_efforts']['c'],[])
        command['time_s']=.75
        with self.assertRaisesRegex(ValueError,'align'):read_window(ep,.5,1.)

    def test_native_geometry_identity_and_time_are_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'geometry.npz';nodes=np.array([[1.,2.,3.],[2.,3.,4.]])
            np.savez(path,time_s=.5,physics_step=1,surface_world_m=nodes)
            ep=self.episode();rows=list(ep.states());rows[1]['body_states']={'cloth':dict(geometry='geometry.npz',topology='topology.json')}
            ep.states=lambda:iter(rows);ep.record_path=lambda record:path
            window=read_window(ep,.5,.5)
            np.testing.assert_array_equal(window['geometries']['cloth'][0][1]['surface_world_m'],nodes)
            np.savez(path,time_s=.6,physics_step=1,surface_world_m=nodes)
            with self.assertRaisesRegex(ValueError,'metadata mismatch'):read_window(ep,.5,.5)

    def test_static_scene_geometry_is_not_treated_as_time_varying_nodes(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'scene.npz';np.savez(path,vertices=[[0,0,0]],triangles=[])
            ep=self.episode();rows=list(ep.states())
            for row in rows:row['body_states']={'table':dict(geometry='scene.npz')}
            ep.states=lambda:iter(rows);ep.record_path=lambda record:path
            window=read_window(ep,0,1.)
            self.assertEqual(window['geometries'],{})
            self.assertIn('table',window['geometry_assets'])

    def test_substep_commands_are_preserved_without_fabricating_saved_states(self):
        ep=self.episode();rows=list(ep.states())
        for row in rows:row['physics_step']*=10
        ep.states=lambda:iter(rows);ep.manifest['timing']['physics_hz']=20
        ep.manifest['control_program']=dict(primitive='impedance_control',controllers=[dict(controller_id='c')])
        commands=[dict(physics_step=k,time_s=k/20) for k in range(20)]
        ep.controls=lambda:iter(commands);ep.actuator_efforts=lambda oid:iter(commands)
        ep.actuator_states=lambda oid:iter(rows)
        window=read_window(ep,.5,1.)
        self.assertEqual(len(window['states']),2)
        self.assertEqual(len(window['controls']),10)
        self.assertEqual(len(window['actuator_efforts']['c']),10)
        self.assertEqual(window['controls'][1]['time_s'],.55)
        ep.actuator_states=lambda oid:iter(commands)
        window=read_window(ep,.5,1.)
        self.assertEqual(len(window['actuator_states']['c']),10)
        self.assertEqual(window['actuator_states']['c'][1]['time_s'],.55)
        self.assertEqual(len(window['states']),2)
        commands[11]['time_s']=.551
        with self.assertRaisesRegex(ValueError,'physics clock'):read_window(ep,.5,1.)


if __name__=='__main__':unittest.main()
