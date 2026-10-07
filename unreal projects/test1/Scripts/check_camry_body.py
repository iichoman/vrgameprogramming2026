"""Finish mesh shading setup and verify saved body/door review assets."""
import json
from pathlib import Path
import unreal

project = Path(unreal.Paths.project_dir()).resolve()
report_path = project/'Saved/CamryBodyImportReport.json'
report = json.loads(report_path.read_text(encoding='utf-8'))
assert report['success']
subsystem = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
if subsystem is None:
    # Commandlets do not initialize the StaticMeshEditor module's subsystem.
    # These methods act directly on their mesh argument and require no instance state.
    subsystem = unreal.new_object(unreal.StaticMeshEditorSubsystem)
results = []
for item in report['groups']:
    mesh = unreal.load_asset(item['asset'])
    # The CAE shell mesh has no UVs. Built-in tangents avoid UV-dependent MikkTSpace.
    settings = subsystem.get_lod_build_settings(mesh, 0)
    settings.set_editor_property('use_mikk_t_space', False)
    settings.set_editor_property('recompute_normals', True)
    settings.set_editor_property('recompute_tangents', True)
    subsystem.set_lod_build_settings(mesh, 0, settings)
    assert unreal.EditorAssetLibrary.save_loaded_asset(mesh)
    count = subsystem.get_number_verts(mesh, 0)
    assert count > 0
    results.append({'asset': item['asset'], 'render_vertices': count,
                    'use_mikk_t_space': subsystem.get_lod_build_settings(mesh, 0).use_mikk_t_space})
level = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
assert level.load_level(report['map'])
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()
report['shading_build_validation'] = results
report['saved_actor_count'] = len(actors)
report_path.write_text(json.dumps(report, indent=2), encoding='utf-8')
unreal.log('CAMRY_BODY_VALIDATION_SUCCESS')
