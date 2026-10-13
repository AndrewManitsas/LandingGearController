"""Exercise cleanup in temporary workspaces without removing real build output."""
import importlib.util
from pathlib import Path
import tempfile
import shutil
import subprocess
import sys
import unittest

spec = importlib.util.spec_from_file_location(
    'clean_workspace', Path(__file__).resolve().parents[1] / 'scripts/clean_workspace.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class CleanupTests(unittest.TestCase):
    def test_generated_folders_only_and_repeated_cleanup(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for name in ('build', 'install', 'log', 'simulation', 'hardware', 'presentation'):
                (root / name).mkdir()
                (root / name / 'keep.txt').write_text(name)
            module.clean_workspace(root)
            module.clean_workspace(root)
            for name in ('build', 'install', 'log'):
                self.assertFalse((root / name).exists())
            for name in ('simulation', 'hardware', 'presentation'):
                self.assertEqual((root / name / 'keep.txt').read_text(), name)

    def test_cli_cleans_simulation_even_from_repository_root(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            sim = repo / 'simulation'
            (sim / 'scripts').mkdir(parents=True)
            script = sim / 'scripts/clean_workspace.py'
            shutil.copyfile(Path(module.__file__), script)
            for parent in (repo, sim):
                for name in ('build', 'install', 'log'):
                    (parent / name).mkdir()
                    (parent / name / 'marker').write_text('keep')
            subprocess.run([sys.executable, str(script)], cwd=repo, check=True,
                           capture_output=True, text=True)
            for name in ('build', 'install', 'log'):
                self.assertFalse((sim / name).exists())
                self.assertEqual((repo / name / 'marker').read_text(), 'keep')

    def test_external_link_is_rejected_before_any_deletion(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'repo'
            external = Path(tmp) / 'outside'
            root.mkdir()
            external.mkdir()
            (external / 'keep.txt').write_text('keep')
            (root / 'build').mkdir()
            (root / 'install').symlink_to(external, target_is_directory=True)
            with self.assertRaises(ValueError):
                module.clean_workspace(root)
            self.assertTrue((root / 'build').exists())
            self.assertEqual((external / 'keep.txt').read_text(), 'keep')

    def test_regular_file_is_rejected_before_any_deletion(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'build').mkdir()
            (root / 'install').write_text('not a generated directory')
            with self.assertRaises(ValueError):
                module.clean_workspace(root)
            self.assertTrue((root / 'build').exists())


if __name__ == '__main__':
    unittest.main()
