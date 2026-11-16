import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from harness.episode import EpisodeLimits
from harness.gui_demo import camera_png, run_demo


class DemoTest(unittest.IsolatedAsyncioTestCase):
    async def test_demo_runs_same_episode_without_model_or_network(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'demo'
            with patch('harness.episode_runtime.AsyncCodex', side_effect=AssertionError('No model in demo')):
                result = await run_demo('unused', None, 'demo-robot', '화면 시연', run_dir=path, _delay=.001)
            self.assertEqual(result['status'], 'completion_claim')
            self.assertEqual(result['action_attempts'], 2)
            self.assertEqual(result['steps'], 5)
            self.assertTrue(result['simulated'])
            self.assertFalse(result['physical_success_verified'])
            self.assertEqual(json.loads((path / 'run.json').read_text())['mode'], 'demo')
            images = list(path.glob('observations/*/image-*.png'))
            self.assertGreaterEqual(len(images), 6)
            self.assertTrue(all(p.read_bytes().startswith(b'\x89PNG\r\n\x1a\n') for p in images))
            events = [json.loads(line) for line in (path / 'events.jsonl').read_text().splitlines()]
            self.assertEqual(sum(e['event'] == 'motion_feedback' for e in events), 2)

    async def test_demo_honors_action_budget(self):
        with tempfile.TemporaryDirectory() as folder:
            result = await run_demo('unused', None, 'demo', 'demo', run_dir=Path(folder) / 'demo',
                                    limits=EpisodeLimits(max_actions=1), _delay=.001)
            self.assertEqual(result['status'], 'budget_actions')
            self.assertEqual(result['action_attempts'], 1)

    def test_synthetic_images_change_with_action_and_view(self):
        self.assertNotEqual(camera_png('front', 0), camera_png('front', 5))
        self.assertNotEqual(camera_png('front', 0), camera_png('wrist', 0))


if __name__ == '__main__':
    unittest.main()
