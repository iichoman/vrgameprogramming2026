"""Import the supplied MISUMI SolidWorks assembly and inventory its hierarchy."""
import json
from pathlib import Path
import unreal

project = Path(unreal.Paths.project_dir()).resolve()
levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
actors_api = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
if unreal.EditorAssetLibrary.does_asset_exist('/Game/DigitalTwin/Maps/TraverseImport'):
    assert levels.load_level('/Game/DigitalTwin/Maps/TraverseImport')
    for actor in actors_api.get_all_level_actors():
        actors_api.destroy_actor(actor)
else:
    assert levels.new_level('/Game/DigitalTwin/Maps/TraverseImport')
scene = unreal.DatasmithSceneElement.construct_datasmith_scene_from_file(
    str(project/'SourceArt/Traverse187/No000187_ASSY/_0187_Assy.SLDASM'))
assert scene, 'CAD translator could not open assembly'
result = scene.import_scene('/Game/DigitalTwin/Traverse187')
assert result.import_succeed, 'CAD import failed'
# Some SolidWorks subassemblies are suppressed in the supplied top-level
# configuration. Import their supplied files too, preserving their CAD placement.
for name in ['_0187_SA02_1', '_0187_SA02_2', '_0187_SA03', '_0187_SA04', '_0187_SA05']:
    element = unreal.DatasmithSceneElement.construct_datasmith_scene_from_file(
        str(project/'SourceArt/Traverse187/No000187_ASSY'/ (name + '.SLDASM')))
    assert element
    imported = element.import_scene('/Game/DigitalTwin/Traverse187')
    assert imported.import_succeed, name
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()
rows = []
for a in actors:
    origin, extent = a.get_actor_bounds(False)
    rows.append({'name': a.get_name(), 'label': a.get_actor_label(),
                 'parent': a.get_attach_parent_actor().get_name() if a.get_attach_parent_actor() else None,
                 'location': list(a.get_actor_location().to_tuple()),
                 'center': list(origin.to_tuple()), 'extent': list(extent.to_tuple())})
unreal.EditorAssetLibrary.save_directory('/Game/DigitalTwin/Traverse187')
unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).save_current_level()
(project/'Saved/TraverseImport.json').write_text(json.dumps(rows, indent=2), encoding='utf-8')
