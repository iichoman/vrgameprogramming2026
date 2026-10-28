"""Create a separate stochastic-recovery map from the validated sensor map."""
import hashlib
import json
from pathlib import Path
import unreal

p=Path(unreal.Paths.project_dir()).resolve()
source=p/'Content/DigitalTwin/Maps/SensorAssemblyLine.umap'
before=hashlib.sha256(source.read_bytes()).hexdigest()
levels=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
dest='/Game/DigitalTwin/Maps/AutonomousAssemblyLine'
assert not unreal.EditorAssetLibrary.does_asset_exist(dest), 'Preserve an existing map.'
assert levels.new_level_from_template(dest,'/Game/DigitalTwin/Maps/SensorAssemblyLine')
old=next(a for a in actors.get_all_level_actors() if isinstance(a,unreal.SensorAssemblyLine))
line=actors.spawn_actor_from_class(unreal.AutonomousAssemblyLine,unreal.Vector())
line.set_actor_label('AutonomousAssemblyLine')
for name in ('wheel_cell','door_cell','vehicle_at_wheel_station','carriage_at_wheel_station',
             'door_station_offset_cm','transfer_seconds','dock_seconds','dock_tolerance_cm'):
    line.set_editor_property(name,old.get_editor_property(name))
assert actors.destroy_actor(old)
assert line.call_method('ResetLine'),line.get_editor_property('status')
assert levels.save_current_level()
assert hashlib.sha256(source.read_bytes()).hexdigest()==before
(p/'Saved/AutonomousAssemblySetup.json').write_text(json.dumps({
    'success':True,'map':dest,'source_unchanged':True},indent=2),encoding='utf-8')
