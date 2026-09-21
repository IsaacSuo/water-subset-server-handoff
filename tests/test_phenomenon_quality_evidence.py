import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from world_model_dataset.io import file_hash
from world_model_dataset.phenomenon_collect import apply_quality_evidence
from world_model_dataset.phenomenon_pilot_batch import run


class EvidenceTests(unittest.TestCase):
    def test_matching_review_clears_only_stability_warning(self):
        ep=SimpleNamespace(manifest={'trajectory':{'states':{'sha256':'current'}}})
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'review.json'
            p.write_text(json.dumps(dict(format='cloth-stability-review/1',source_states_sha256='current',
                                        status='local_stability_checked',checks={'finite':True})))
            record=dict(path=str(p),sha256=file_hash(p))
            review=apply_quality_evidence(ep,{'warnings':['Cloth contact stability pending','Other issue']},record)
            self.assertIn('Other issue',review['warnings'])
            self.assertFalse(any(w.startswith('Cloth contact stability') for w in review['warnings']))
            p.write_text('{}')
            with self.assertRaisesRegex(ValueError,'hash mismatch'):
                apply_quality_evidence(ep,{'warnings':[]},record)

    def test_changed_backend_refuses_before_creating_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);source=folder/'entry.py';source.write_text('changed')
            plan=folder/'plan.json'
            plan.write_text(json.dumps(dict(material_entry=str(source),material_sources={'entry.py':'old_hash'},
                                            jobs=[dict(id='cloth',backend='material')])))
            with self.assertRaisesRegex(ValueError,'Pinned material source changed'):
                run(plan,folder/'output')
            self.assertFalse((folder/'output').exists())

    def test_evidence_cannot_clear_another_cache(self):
        ep=SimpleNamespace(manifest={'trajectory':{'states':{'sha256':'current'}}})
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'review.json'
            p.write_text(json.dumps(dict(format='cloth-stability-review/1',source_states_sha256='other',
                                        status='local_stability_checked',checks={'finite':True})))
            with self.assertRaisesRegex(ValueError,'different physical cache'):
                apply_quality_evidence(ep,{'warnings':['Cloth contact stability pending']},
                                       dict(path=str(p),sha256=file_hash(p)))

    def test_failed_local_review_keeps_warning(self):
        ep=SimpleNamespace(manifest={'trajectory':{'states':{'sha256':'current'}}})
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'review.json'
            p.write_text(json.dumps(dict(format='cloth-stability-review/1',source_states_sha256='current',
                                        status='needs_stability_review',checks={'tail_motion':False})))
            review=apply_quality_evidence(ep,{'warnings':['Cloth contact stability pending']},
                                         dict(path=str(p),sha256=file_hash(p)))
            self.assertEqual(review['warnings'],['Cloth contact stability pending'])


if __name__=='__main__':unittest.main()
