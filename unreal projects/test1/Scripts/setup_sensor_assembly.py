"""Clone the integrated scene and replace only its coordinator with sensor control."""
import json
from pathlib import Path
import unreal

p=Path(unreal.Paths.project_dir()).resolve()
levels=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
dest='/Game/DigitalTwin/Maps/SensorAssemblyLine'
assert not unreal.EditorAssetLibrary.does_asset_exist(dest), 'Preserve existing user edits.'
assert levels.new_level_from_template(dest,'/Game/DigitalTwin/Maps/IntegratedAssemblyLine')
old=next(a for a in actors.get_all_level_actors() if isinstance(a,unreal.IntegratedAssemblyLine))
line=actors.spawn_actor_from_class(unreal.SensorAssemblyLine,unreal.Vector())
line.set_actor_label('SensorAssemblyLine')
line.set_editor_property('dock_tolerance_cm',0.5)
for name in ('wheel_cell','door_cell','vehicle_at_wheel_station','carriage_at_wheel_station','door_station_offset_cm','transfer_seconds','dock_seconds'):
    line.set_editor_property(name,old.get_editor_property(name))
assert actors.destroy_actor(old)
assert line.call_method('ResetLine'),line.get_editor_property('status')
assert levels.save_current_level()
(p/'Saved/SensorAssemblySetup.json').write_text(json.dumps({'success':True,'map':dest}),encoding='utf-8')
