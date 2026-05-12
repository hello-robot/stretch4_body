import os
import sys

version_file = os.path.join(os.path.dirname(__file__), 'stretch4_body', 'version.py')
with open(version_file) as f:
    d = {}
    exec(f.read(), d)
    version = d['__version__']

if os.environ.get('IS_EDITABLE_INSTALL') == '1':
    try:
        import subprocess
        git_hash = subprocess.check_output(
            ['git', 'rev-parse', '--short', 'HEAD'],
            cwd=os.path.dirname(os.path.abspath(__file__)),
            stderr=subprocess.DEVNULL
        ).decode('utf-8').strip()
        if git_hash:
            version = f"{version}+{git_hash}"
    except Exception:
        pass

print(version)
