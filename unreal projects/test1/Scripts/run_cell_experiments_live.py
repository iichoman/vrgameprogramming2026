import runpy
from pathlib import Path
import unreal

runpy.run_path(str(Path(unreal.Paths.project_dir())/'Scripts/verify_cell_experiments.py'),
               run_name='__main__',init_globals={'VERIFY_CURRENT_LEVEL':True})
