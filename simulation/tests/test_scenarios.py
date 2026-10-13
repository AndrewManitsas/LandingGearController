"""Offline checks for scenario flight defaults and real-sensor readiness."""
import ast
import math
from pathlib import Path
import subprocess
import types
import unittest
import xml.etree.ElementTree as ET

SIM = Path(__file__).resolve().parents[1]

class ScenarioTests(unittest.TestCase):
    def test_every_world_loads_range_sensor_system(self):
        for world in ('alpha', 'beta', 'gamma', 'approach'):
            with self.subTest(world=world):
                root = ET.parse(SIM / f'worlds/testbed_{world}.sdf')
                names = {p.get('name') for p in root.findall('./world/plugin')}
                for name in ('Physics', 'UserCommands', 'SceneBroadcaster', 'Sensors'):
                    self.assertIn('gz::sim::systems::' + name, names)

    def test_flying_presets_allow_explicit_static_override(self):
        for world in ('alpha', 'beta', 'gamma'):
            for flight in ('true', 'false'):
                args = ['make', '-n', '-C', str(SIM), world, 'GUI=false']
                if flight == 'false':
                    args.append('FLIGHT=false')
                output = subprocess.check_output(args, text=True)
                self.assertIn(f'"flight:={flight}"', output)
                self.assertIn(f'"world:={world}"', output)

    def test_fallback_features_do_not_satisfy_sensor_readiness(self):
        # Exercise the actual evaluator without requiring ROS for offline tests.
        source = ast.parse((SIM / 'scripts/verify_system.py').read_text())
        cls = next(n for n in source.body if isinstance(n, ast.ClassDef)
                   and n.name == 'VerificationEvaluator')
        clock = [100.0]
        topics = ('fore', 'port', 'starboard', 'sonar')
        ns = {'Node': object, 'SENSOR_TOPICS': topics, 'math': math,
              'time': types.SimpleNamespace(monotonic=lambda: clock[0])}
        exec(compile(ast.Module(body=[cls], type_ignores=[]), '<evaluator>', 'exec'), ns)
        evaluator = ns['VerificationEvaluator'].__new__(ns['VerificationEvaluator'])
        evaluator.reset()
        for cb, value in ((evaluator.verdict_cb, 0), (evaluator.abort_cb, False),
                          (evaluator.gear_status_cb, 0.0)):
            cb(types.SimpleNamespace(data=value))
        evaluator.features_cb(types.SimpleNamespace(data=[0.85, 1e-5, 0, 1e-5, 0, 0, 0]))
        self.assertFalse(evaluator.ready())
        scan = types.SimpleNamespace(ranges=[0.5], range_min=0.03, range_max=2.5)
        for topic in topics:
            for _ in range(10):
                evaluator.scan_cb(topic, scan)
        self.assertTrue(evaluator.ready())
        clock[0] = 103.0
        self.assertFalse(evaluator.ready())

if __name__ == '__main__':
    unittest.main()
