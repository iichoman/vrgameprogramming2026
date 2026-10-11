"""Create a separate midterm demo map; never changes CamryAssemblyPreview."""
import json
import shutil
from pathlib import Path
import unreal

target = '/Game/DigitalTwin/Maps/CamryCameraCell_Midterm'
project = Path(unreal.Paths.project_dir())
maps = project / 'Content/DigitalTwin/Maps'
demo_file = maps / 'CamryCameraCell_Midterm.umap'
if not demo_file.is_file():
    shutil.copy2(maps / 'CamryAssemblyPreview.umap', demo_file)
levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
assert levels.load_level(target), 'Could not load demo map'
all_actors = actors.get_all_level_actors()
assembly = next(a for a in all_actors if a.get_class().get_path_name() == '/Script/test1.CamryAssembly')
robot_class = unreal.load_class(None, '/Game/Robot/Blueprints/BP_KUKA_Robot_InputFixed.BP_KUKA_Robot_InputFixed_C')
assert robot_class, 'Robot Blueprint class not found'
robot = next((a for a in all_actors if 'BP_KUKA_Robot_InputFixed_C' in a.get_class().get_path_name()), None)
if robot is None:
    robot = actors.spawn_actor_from_class(robot_class, unreal.Vector(200, -190, 0), unreal.Rotator())
    assert robot, 'Could not spawn robot'
    robot.set_actor_label('Midterm_KUKA_6Axis')
demo_class = unreal.load_class(None, '/Script/test1.MidtermCameraCell')
assert demo_class, 'Camera demo class not found'
demo = next((a for a in actors.get_all_level_actors() if a.get_class().get_path_name() == '/Script/test1.MidtermCameraCell'), None)
if demo is None:
    demo = actors.spawn_actor_from_class(demo_class, unreal.Vector(0, 0, 0), unreal.Rotator())
    assert demo, 'Could not spawn camera controller'
    demo.set_actor_label('Midterm_Camera_Interlock')
demo.set_editor_property('assembly', assembly)
demo.set_editor_property('robot', robot)
assert levels.save_current_level(), 'Could not save demo map'
(project / 'Saved/MidtermMapSetup.json').write_text(json.dumps({
    'source_untouched': '/Game/DigitalTwin/Maps/CamryAssemblyPreview',
    'demo_map': target,
    'robot_class': robot_class.get_path_name(),
    'controller_class': demo_class.get_path_name(),
    'robot_location_cm': [200, -190, 0],
    'camera_method': 'known landmark projected into SceneCapture camera frame, not image pixels',
}, indent=2), encoding='utf-8')
print('MIDTERM_CAMERA_MAP_CREATED')
