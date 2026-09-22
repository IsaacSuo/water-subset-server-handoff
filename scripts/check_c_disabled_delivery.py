"""Read-only verification of the d8a4c4b delivery; no source upgrade or physics."""
import argparse
from pathlib import Path
import numpy as np
from world_model_dataset.io import read_json,write_json,file_hash
from world_model_dataset.causal_loader import open_episode
from world_model_dataset.experiment_review import verify_alignment


def check(root,output):
    evidence=root/'evidence/rope_load_c/disabled_v1'
    audit=read_json(evidence/'delivery_audit.json');run=Path(audit['run_path'])
    for rel,h in audit['file_sha256'].items():
        assert file_hash(run/rel)==h,rel
    for rel,h in audit['review_sha256'].items():
        assert file_hash(evidence/rel)==h,rel
    ep=open_episode(run/'episode',require_complete=False)
    alignment=verify_alignment(ep,dict(duration_s=3.,physics_hz=960,state_hz=60))
    efforts=list(ep.actuator_efforts())
    assert len(efforts)==2880
    for row in efforts:
        assert row['target_active'] is False and row['phase']=='disabled'
        assert np.array_equal(row['applied_force_world_n'],np.zeros(3))
        assert np.array_equal(row['applied_torque_world_nm'],np.zeros(3))
    pair=read_json(evidence/'paired_cache_review.json')
    write_json(output,dict(delivery_commit='d8a4c4b415ae283f2ed8d8903ed2274bb1ebd40c',
        control_extension_commit=audit['control_extension_commit'],
        delivery_hashes_verified=True,checked_hashes={**audit['file_sha256'],**audit['review_sha256']},
        source_audit_sha256=file_hash(evidence/'delivery_audit.json'),alignment=alignment,
        actual_wrench_all_zero=True,annotations=ep.annotations(),
        common_stable_reference_window=pair['reference_rule']['window_s'],
        interface_difference_from_05aabd3='package writes final disabled annotation once; fixes duplicate exclusive write; solver/control input contract unchanged',
        pinned_entry_versions_unchanged=True,physics_runs=0,source_workspace_modified=False,
        qualification='human_use_review_pending; training_admission=false'))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,default=Path('/mnt/y/isaacsim_work_rope_load'))
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();check(a.source,a.output)
