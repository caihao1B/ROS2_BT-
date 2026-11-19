import json
import tempfile
import unittest
from pathlib import Path

from replay_archive import bounded_samples


class CursorReview(unittest.TestCase):
    def test_sampling_ignores_stale_future_and_out_of_order_supervision(self):
        with tempfile.TemporaryDirectory() as temp:
            run = Path(temp)
            path = run / 'reference/reference.jsonl'
            path.parent.mkdir()
            samples = [(3, 3.05, 'future'), (3.01, 2.0, 'stale'),
                       (3.05, 3.03, 'newest'), (3.06, 3.02, 'older')]
            path.write_text(''.join(json.dumps(dict(t=t, native_supervisor=dict(t=s, gnss_reason=reason)))+'\n'
                                    for t, s, reason in samples))
            a, b, c = bounded_samples(run, [3., 3.07, 4.])
            self.assertNotIn('gnss', a)
            self.assertEqual(b['gnss'][:2], [3.03, 'newest'])
            self.assertEqual(c, {})
