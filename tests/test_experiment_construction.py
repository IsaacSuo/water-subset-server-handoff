"""No solver imports: geometry rejection, reuse and derived-condition checks."""
import copy
import tempfile
import unittest
from pathlib import Path

import numpy as np
import trimesh

from world_model_dataset.experiment_construct import construct, compare_requests, construct_pair
from world_model_dataset.experiment_contract import intervention
from world_model_dataset.io import read_json, write_json, file_hash

ROOT = Path(__file__).resolve().parents[1]
PROFILES = ROOT/'configs/dataset/v0_2/construction/profiles'


class ConstructionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def mesh(self, name, size, center):
        mesh=trimesh.creation.box(size); mesh.apply_translation(center)
        path=self.root/(name+'.npz')
        np.savez(path, vertices=mesh.vertices, triangles=mesh.faces)
        return dict(id=name, path=str(path))

    def request(self, kind):
        support=self.mesh('support',[1,1,.1],[0,0,.45])
        scene=dict(id='original', units='m', up_axis='Z', frame='original_world',
                   region_bounds_m=[[-.7,-.45,0],[.95,.45,1.5]],
                   collision=dict(meshes=[support]), source_records={})
        return dict(format='phenomenon-construction/1', id='trial', phenomenon=kind,
                    profile='profile.json', scene=scene, support_group='support',
                    object=dict(size_m=[.6,.3]), conditions=dict(overhang_fraction=.3))

    def test_cloth_resize_and_original_region_reselection(self):
        r=self.request('cloth_drape'); p=read_json(PROFILES/'cloth.json')
        a,_,report=construct(r,p)
        self.assertAlmostEqual(report['calculations']['supported_area_fraction'], .7)
        q=copy.deepcopy(r); q['object']['size_m']=[.4,.2]
        b,_,_=construct(q,p)
        self.assertNotEqual(a['input']['cloth']['cells'], b['input']['cloth']['cells'])
        q['scene']['region_bounds_m'][0][1]=-.2
        c,_,_=construct(q,p)
        self.assertNotEqual(b['input']['cloth']['world_from_mesh'][1][3], c['input']['cloth']['world_from_mesh'][1][3])
        self.assertEqual(a['scene']['collision'], c['scene']['collision'])

    def test_cloth_clearance_and_support_rejections(self):
        r=self.request('cloth_drape'); p=read_json(PROFILES/'cloth.json')
        r['scene']['collision']['meshes'].append(self.mesh('obstacle',[.3,.4,.4],[.7,0,.3]))
        with self.assertRaisesRegex(ValueError,'hanging_clearance|edge_transition'):
            construct(r,p)
        r=self.request('cloth_drape'); r['object']['size_m']=[.6,1.2]
        with self.assertRaisesRegex(ValueError,'support_shape'):
            construct(r,p)

    def test_condition_derived_impacts_and_invariants(self):
        r=self.request('cloth_drape'); q=copy.deepcopy(r)
        q['conditions']['overhang_fraction']=.4
        result=compare_requests(r,q,read_json(PROFILES/'cloth.json'))
        self.assertIn('/cloth/world_from_mesh/0/3', [p.removeprefix('/input') for p in result['derived_changes']])
        self.assertNotEqual(result['before']['reserved_drop_m'],result['after']['reserved_drop_m'])
        profile=read_json(PROFILES/'cloth.json')
        doc,report=construct_pair(r,q,profile)
        resolved,_=intervention(doc,doc['conditions'][1])
        expected,_,_=construct(q,profile)
        self.assertEqual(resolved['input'],expected['input'])
        self.assertEqual(resolved['scene'],doc['scene'])

    def rigid_request(self):
        r=self.request('geometry_constrained_motion')
        meshes=[self.mesh('support',[3,1,.1],[0,0,.45]),
                self.mesh('wall_a',[.2,.3,.5],[0,-.35,.75]),
                self.mesh('wall_b',[.2,.3,.5],[0,.35,.75])]
        manifest=self.root/'scene.json'
        write_json(manifest,dict(metres_per_unit=1,coordinate_frame='original Z up',
            groups={m['id']:dict(path=m['path'],sha256=file_hash(m['path'])) for m in meshes}))
        r['scene']['collision']=dict(scene_export=str(manifest))
        r['scene']['region_bounds_m']=[[-1.4,-.5,0],[1.4,.5,1.5]]
        folder=self.root/'library'/'sample'; folder.mkdir(parents=True)
        m=trimesh.creation.box([1,.8,.6]); np.savez(folder/'geometry.npz',vertices=m.vertices,triangles=m.faces)
        write_json(folder/'asset.json',dict(format='physical-asset-source/1',geometry='geometry.npz',geometry_sha256=file_hash(folder/'geometry.npz')))
        p=read_json(PROFILES/'passage.json'); p['asset_library']=dict(exploration_root=str(self.root),library='library')
        r['object']=dict(id='subject',asset='sample',size_m=.2,mass_kg=.2,rotation_deg=[0,0,0])
        r['conditions']=dict(speed_m_s=.4,clearance_regime='passable')
        return r,p

    def test_passage_resize_recompute_and_regime_reject(self):
        r,p=self.rigid_request(); a,_,ar=construct(r,p)
        q=copy.deepcopy(r); q['object']['size_m']=.3
        b,_,br=construct(q,p)
        self.assertNotEqual(a['input']['participants'][0]['xy_m'],b['input']['participants'][0]['xy_m'])
        self.assertLess(br['calculations']['signed_clearance_m'],ar['calculations']['signed_clearance_m'])
        q['object']['size_m']=.7
        with self.assertRaisesRegex(ValueError,'clearance_regime'): construct(q,p)
        q['conditions']['clearance_regime']='obstructed'
        _,_,br=construct(q,p)
        self.assertLess(br['calculations']['signed_clearance_m'],0)

    def test_units_refused(self):
        r=self.request('cloth_drape'); r['scene']['units']='cm'
        with self.assertRaisesRegex(ValueError,'coordinates'): construct(r,read_json(PROFILES/'cloth.json'))

    def test_other_material_rules_resize_and_condition_invariants(self):
        examples={
            'plastic':('plastic_impact',dict(kind='box',size_m=[.12,.08,.06],density_kg_m3=900),dict(drop_height_m=.1)),
            'beam':('beam_load_hold_withdraw',dict(size_m=[.22,.04,.03]),dict(clamp_fraction=.2,deflection_fraction=.1,max_force_n=10)),
            'rope':('rope_passive',dict(length_m=.3,radius_m=.003,density_kg_m3=800),dict(clamp_fraction=.2))}
        for name,(phenomenon,obj,condition) in examples.items():
            with self.subTest(name=name):
                r=self.request(phenomenon); r['object']=obj; r['conditions']=condition
                if name=='plastic': r['scene']['region_bounds_m'][1][0]=.5
                p=read_json(PROFILES/(name+'.json')); a,_,_=construct(r,p)
                q=copy.deepcopy(r)
                if name=='rope': q['object']['length_m']=.25
                else: q['object']['size_m']=[v*.8 for v in obj['size_m']]
                b,_,_=construct(q,p)
                self.assertNotEqual(a['input'],b['input'])
                report=compare_requests(r,q,p)
                self.assertTrue(report['profile_material_numerics_unchanged'])

    def test_roll_and_chain_recompute(self):
        r,p=self.rigid_request()
        for kind in ('rigid_roll_slide','multibody_collision_propagation'):
            q=copy.deepcopy(r); q['phenomenon']=kind; profile=copy.deepcopy(p); profile['phenomenon']=kind
            # Use an upstream subregion of exactly the same original mesh.
            q['scene']['region_bounds_m'][1][0]=-.2
            if kind=='rigid_roll_slide': q['conditions']=dict(speed_m_s=.5,spin_ratio=1)
            else:
                q['object']=[dict(q['object'],id='a'),dict(q['object'],id='b')]
                q['conditions']=dict(speed_m_s=.5,gap_ratio=.2)
            a,_,_=construct(q,profile)
            self.assertEqual(len(a['input']['participants']),1 if kind=='rigid_roll_slide' else 2)

    def test_unusable_shapes_and_unknown_conditions_fail(self):
        r=self.request('cloth_drape'); r['conditions']['world_x']=3
        with self.assertRaisesRegex(ValueError,'unknown'): construct(r,read_json(PROFILES/'cloth.json'))
        r=self.request('beam_load_hold_withdraw'); r['object']=dict(size_m=[.1,.1,.1])
        r['conditions']=dict(clamp_fraction=.2,deflection_fraction=.1,max_force_n=10)
        with self.assertRaisesRegex(ValueError,'beam_shape'): construct(r,read_json(PROFILES/'beam.json'))


if __name__=='__main__': unittest.main()
