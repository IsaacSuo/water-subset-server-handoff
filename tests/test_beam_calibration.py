import copy
import numpy as np
from tests.test_experiment_construction import ConstructionTests,PROFILES
from world_model_dataset.experiment_construct import construct
from world_model_dataset.io import read_json,write_json,file_hash,digest


class BeamCalibrationTests(ConstructionTests):
    # These tests exercise cache compatibility/rejection, not a fake physics claim.
    def setup_probe(self):
        r=self.request('beam_load_hold_withdraw');r['object']=dict(size_m=[.22,.05,.03])
        r['conditions']=dict(clamp_fraction=.13,deflection_fraction=.2,max_force_n=15.)
        p=read_json(PROFILES/'beam_adaptive.json');entry=self.root/'backend/entry.py';entry.parent.mkdir();entry.write_text('# fake public entry\n')
        runtime=entry.parent/'runtime.json';write_json(runtime,{})
        p['backend'].update(entry=str(entry),runtime=str(runtime))
        doc,_,_=construct(r,p);folder=self.root/'probe';folder.mkdir();report=folder/'construction.json'
        doc['scene']['source_records']['construction']=str(report)
        write_json(folder/'experiment.json',doc)
        write_json(report,dict(experiment_sha256=digest(doc),source_pins={str(entry):file_hash(entry),str(runtime):file_hash(runtime)}))
        cfg=copy.deepcopy(doc['input']);cfg['physics_hz']=p['timing']['physics_hz'];cfg['environment']=[dict(id='support',geometry='environment.npz')]
        cfg['source_records']=doc['scene']['source_records']
        center=np.array(cfg['beam']['center_m']);dims=np.array(cfg['beam']['size_m'])
        points=np.stack(np.meshgrid(np.linspace(-dims[0]/2,dims[0]/2,8),np.linspace(-dims[1]/2,dims[1]/2,5),[-dims[2]/2,dims[2]/2],indexing='ij'),-1).reshape(-1,3)+center
        t=np.arange(481)/60;surface=np.repeat(points[None],len(t),axis=0);surface[:,:,2]-=.04*(1-np.exp(-t[:,None]))
        pose=np.tile([*cfg['plate']['center_m'],0,0,0,1],(len(t),1));raw=folder/'frames.npz'
        np.savez(raw,time=t,collision_world=surface,plate_pose=pose)
        resolved=dict(config=cfg,provenance={'support':dict(sha256=file_hash(r['scene']['collision']['meshes'][0]['path']))},raw_native=dict(path='frames.npz',sha256=file_hash(raw)))
        write_json(folder/'resolved_inputs.json',resolved);write_json(folder/'episode.physics.json',{})
        return r,p,folder,resolved

    def test_calibration_derives_path_and_retains_physics(self):
        r,p,ep,_=self.setup_probe();probe,_,_=construct(r,p);doc,_,report=construct(r,p,ep)
        self.assertEqual(doc['input']['beam'],probe['input']['beam'])
        self.assertEqual(doc['input']['fixture'],probe['input']['fixture'])
        self.assertEqual(doc['scene']['collision'],probe['scene']['collision'])
        self.assertLess(min(s[1] for s in doc['input']['plate']['schedule']),doc['input']['plate']['center_m'][2]-.04)
        self.assertEqual(report['calculations']['calibration']['phase'],'calibrated_load')
        changed=copy.deepcopy(r);changed['object']['size_m'][0]=.21
        with self.assertRaisesRegex(ValueError,'calibration_mismatch'):construct(changed,p,ep)
        changed=copy.deepcopy(r);changed['scene']['region_bounds_m'][0][1]+=.1
        with self.assertRaisesRegex(ValueError,'calibration_mismatch'):construct(changed,p,ep)

    def test_tampered_native_cache_is_rejected(self):
        r,p,ep,_=self.setup_probe()
        with (ep/'frames.npz').open('ab') as f:f.write(b'tamper')
        with self.assertRaisesRegex(ValueError,'calibration_source'):construct(r,p,ep)

    def test_nonconstant_probe_is_rejected(self):
        r,p,ep,resolved=self.setup_probe();resolved['config']['plate']['schedule'][-1][1]-=.01
        import json
        (ep/'resolved_inputs.json').write_text(json.dumps(resolved))
        with self.assertRaisesRegex(ValueError,'calibration_loaded'):construct(r,p,ep)
