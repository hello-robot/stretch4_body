from mesonpy import *
import shutil
import os

_orig_which = shutil.which

def _patched_which(cmd, mode=os.F_OK | os.X_OK, path=None):
    if cmd == 'ninja':
        # Return just the command name so meson-python doesn't hardcode the ephemeral pip-build-env path
        return 'ninja'
    return _orig_which(cmd, mode, path)

shutil.which = _patched_which

import mesonpy
if hasattr(mesonpy, 'build_editable'):
    def build_editable(wheel_directory, config_settings=None, metadata_directory=None):
        os.environ['IS_EDITABLE_INSTALL'] = '1'
        return mesonpy.build_editable(wheel_directory, config_settings, metadata_directory)

if hasattr(mesonpy, 'get_requires_for_build_editable'):
    def get_requires_for_build_editable(config_settings=None):
        os.environ['IS_EDITABLE_INSTALL'] = '1'
        return mesonpy.get_requires_for_build_editable(config_settings)

if hasattr(mesonpy, 'prepare_metadata_for_build_editable'):
    def prepare_metadata_for_build_editable(metadata_directory, config_settings=None):
        os.environ['IS_EDITABLE_INSTALL'] = '1'
        return mesonpy.prepare_metadata_for_build_editable(metadata_directory, config_settings)
