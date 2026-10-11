"""Read-only inspection of the robot Blueprint and existing maps."""
import json
from pathlib import Path
import unreal

project = Path(unreal.Paths.project_dir())
out = {}
asset = unreal.EditorAssetLibrary.load_asset('/Game/Robot/Blueprints/BP_KUKA_Robot')
out['robot_class'] = asset.get_path_name() if asset else None
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
for map_path in ('/Game/Robot/Maps/RobotControlLab_InputFixed', '/Game/DigitalTwin/Maps/CamryAssemblyPreview'):
    if not levels.load_level(map_path):
        out[map_path] = 'load failed'
        continue
    rows = []
    for actor in actors.get_all_level_actors():
        row = {'name': actor.get_name(), 'class': actor.get_class().get_path_name(),
               'location': [actor.get_actor_location().x, actor.get_actor_location().y, actor.get_actor_location().z]}
        if 'KUKA' in row['name'] or 'Robot' in row['class']:
            row['components'] = [
                {'name': c.get_name(), 'class': c.get_class().get_name(),
                 'rotation': [c.get_editor_property('relative_rotation').pitch,
                              c.get_editor_property('relative_rotation').yaw,
                              c.get_editor_property('relative_rotation').roll]}
                for c in actor.get_components_by_class(unreal.SceneComponent)
            ]
        rows.append(row)
    out[map_path] = rows
(project / 'Saved/MidtermAssetInspection.json').write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding='utf-8')
print('MIDTERM_ASSET_INSPECTION_DONE')
