"""Editor startup/live entry: build and save only the verified new cell map."""
import runpy
from pathlib import Path
import unreal

runpy.run_path(str(Path(unreal.Paths.project_dir()).resolve() / 'Scripts/setup_door_assembly_cell.py'),
               run_name='__main__', init_globals={'DOOR_ASSEMBLY_SAVE': True})
