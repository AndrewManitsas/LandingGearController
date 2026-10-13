"""Remove only generated ROS workspace directories inside the simulation workspace."""
from pathlib import Path
import shutil

GENERATED_DIRECTORIES = ('build', 'install', 'log')


def clean_workspace(root):
    root = Path(root).resolve(strict=True)
    targets = [root / name for name in GENERATED_DIRECTORIES]
    # Validate every target before deleting anything. Refuse links and files so
    # an unexpected layout cannot cause cleanup outside the generated folders.
    for target in targets:
        if target.is_symlink():
            raise ValueError(f'Refusing symlink: {target}')
        if target.exists():
            if not target.is_dir() or target.resolve().parent != root:
                raise ValueError(f'Refusing unexpected cleanup target: {target}')
    for target in targets:
        if target.exists():
            shutil.rmtree(target)
            print(f'Removed {target.name}/')
        else:
            print(f'{target.name}/ already absent')


if __name__ == '__main__':
    # Derive the simulation workspace from this script, never from the caller's cwd.
    clean_workspace(Path(__file__).resolve().parents[1])
