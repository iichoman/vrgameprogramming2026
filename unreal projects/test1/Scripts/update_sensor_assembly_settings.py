"""Update the existing generated sensor map's docking tolerance."""
import unreal

levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
assert levels.load_level('/Game/DigitalTwin/Maps/SensorAssemblyLine')
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()
line = next(a for a in actors if isinstance(a, unreal.SensorAssemblyLine))
line.set_editor_property('dock_tolerance_cm', 0.5)
assert levels.save_current_level()
