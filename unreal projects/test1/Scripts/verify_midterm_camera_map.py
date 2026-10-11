"""Cold-load check for the isolated demo map. Does not save it."""
import json
from pathlib import Path
import unreal

assert unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).load_level('/Game/DigitalTwin/Maps/CamryCameraCell_Midterm')
all_actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()
assembly = next(a for a in all_actors if a.get_class().get_path_name() == '/Script/test1.CamryAssembly')
demo = next(a for a in all_actors if a.get_class().get_path_name() == '/Script/test1.MidtermCameraCell')
robot = demo.get_editor_property('robot')
assert demo.get_editor_property('assembly') == assembly
assert robot and 'BP_KUKA_Robot_InputFixed_C' in robot.get_class().get_path_name()
components = {c.get_name() for c in robot.get_components_by_class(unreal.SceneComponent)}
assert {f'J_A{i}' for i in range(1, 7)}.issubset(components)
assert len(demo.get_components_by_class(unreal.SceneCaptureComponent2D)) == 1
report = {'map_reloaded': True, 'assembly': assembly.get_name(), 'robot': robot.get_name(),
          'camera_controller': demo.get_name(), 'joints': sorted(n for n in components if n.startswith('J_A')),
          'runtime_play_tested': False}
(Path(unreal.Paths.project_dir()) / 'Saved/MidtermMapVerify.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
print('MIDTERM_CAMERA_MAP_VERIFY_PASS')
