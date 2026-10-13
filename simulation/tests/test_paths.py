"""Offline regression checks for the simulation's relocated resources."""
import ast
import importlib.util
import os
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

SIM = Path(__file__).resolve().parents[1]

class Item:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs

def load_launch():
    modules = {}
    definitions = {
        'launch': ['LaunchDescription'],
        'launch.actions': ['DeclareLaunchArgument', 'ExecuteProcess', 'TimerAction'],
        'launch.conditions': ['IfCondition', 'UnlessCondition'],
        'launch.substitutions': ['LaunchConfiguration', 'PythonExpression'],
        'launch_ros': [], 'launch_ros.actions': ['Node'],
    }
    for name, symbols in definitions.items():
        module = types.ModuleType(name)
        for symbol in symbols:
            setattr(module, symbol, type(symbol, (Item,), {}))
        modules[name] = module
    spec = importlib.util.spec_from_file_location('launch_under_test', SIM / 'launch/system_launch.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict('sys.modules', modules):
        spec.loader.exec_module(module)
    return module

def path_assignments(file):
    """Execute only real top-level path assignments, without ROS/ML imports."""
    parsed = ast.parse(file.read_text())
    selected = [n for n in parsed.body if isinstance(n, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id in
                        ('SIMULATION_DIR', 'DATASET_PATH', 'OUTPUT_HEADER', 'LAUNCH_FILE') for t in n.targets)]
    namespace = {'os': os, '__file__': str(file)}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(file), 'exec'), namespace)
    return namespace

class ResourceTests(unittest.TestCase):
    def test_all_worlds_and_resources_resolve_from_other_cwd(self):
        module = load_launch()
        old_cwd = os.getcwd()
        with tempfile.TemporaryDirectory() as tmp:
            try:
                os.chdir(tmp)
                desc = module.generate_launch_description()
            finally:
                os.chdir(old_cwd)
        actions = desc.args[0]
        gazebos = [a for a in actions if type(a).__name__ == 'ExecuteProcess'
                   and a.kwargs['cmd'][0] == 'gz']
        self.assertEqual(len(gazebos), 2)
        for action in gazebos:
            expression = action.kwargs['cmd'][-1]
            for world in ('alpha', 'beta', 'gamma', 'approach'):
                code = ''.join(world if isinstance(part, Item) else part for part in expression.args[0])
                self.assertEqual(Path(eval(code)), SIM / 'worlds' / f'testbed_{world}.sdf')
                self.assertTrue(Path(eval(code)).is_file())
        timer = next(a for a in actions if type(a).__name__ == 'TimerAction')
        spawn = timer.kwargs['actions'][0]
        self.assertTrue(Path(spawn.kwargs['arguments'][1]).is_file())
        bridge = next(a for a in actions if a.kwargs.get('executable') == 'parameter_bridge')
        self.assertTrue(Path(bridge.kwargs['parameters'][0]['config_file']).is_file())
        carrier = next(a for a in actions if type(a).__name__ == 'ExecuteProcess'
                       and a.kwargs['cmd'][0] == 'python3')
        self.assertTrue(Path(carrier.kwargs['cmd'][1]).is_file())

    def test_training_paths_stay_inside_simulation(self):
        ns = path_assignments(SIM / 'scripts/train_classifier.py')
        self.assertEqual(Path(ns['DATASET_PATH']), SIM / 'data/dataset.csv')
        self.assertEqual(Path(ns['OUTPUT_HEADER']), SIM / 'src/intelligence_domain/include/intelligence_domain/model_rules.hpp')
        self.assertTrue(Path(ns['DATASET_PATH']).is_file())
        self.assertTrue(Path(ns['OUTPUT_HEADER']).is_file())

    def test_recorder_default_targets_simulation_dataset(self):
        file = SIM / 'scripts/record_dataset.py'
        ns = path_assignments(file)
        call = next(n for n in ast.walk(ast.parse(file.read_text()))
                    if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                    and n.func.attr == 'add_argument' and n.args
                    and isinstance(n.args[0], ast.Constant) and n.args[0].value == '--output')
        default = next(k.value for k in call.keywords if k.arg == 'default')
        value = eval(compile(ast.Expression(default), str(file), 'eval'), ns)
        self.assertEqual(Path(value), SIM / 'data/dataset.csv')

    def test_verification_launch_is_absolute_and_static(self):
        file = SIM / 'scripts/verify_system.py'
        ns = path_assignments(file)
        self.assertEqual(Path(ns['LAUNCH_FILE']), SIM / 'launch/system_launch.py')
        parsed = ast.parse(file.read_text())
        cmd = next(n.value for n in ast.walk(parsed) if isinstance(n, ast.Assign)
                   and any(isinstance(t, ast.Name) and t.id == 'cmd' for t in n.targets))
        ns['world_name'] = 'alpha'
        args = eval(compile(ast.Expression(cmd), str(file), 'eval'), ns)
        self.assertIn('flight:=false', args)
        self.assertIn('gui:=false', args)
        self.assertTrue(Path(args[2]).is_file())

if __name__ == '__main__':
    unittest.main()
