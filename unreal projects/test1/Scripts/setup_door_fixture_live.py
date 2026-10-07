"""Run fixture setup in the currently open CamryAssemblyPreview editor map.

Run only after Live Coding has successfully loaded the native DoorFixture class.
This entry point never changes or reloads the open map. The existing script
restores temporary test poses before saving the current map and reports that
cold-load verification is still pending.
"""
import runpy
from pathlib import Path

import unreal


runpy.run_path(
    str(Path(unreal.Paths.project_dir()).resolve() / 'Scripts/setup_door_fixture.py'),
    init_globals={'FIXTURE_LIVE': True},
)
