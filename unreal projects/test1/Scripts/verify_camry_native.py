"""Cold-load verification after migration and removal of the old Blueprint."""
import json
from pathlib import Path
import unreal

project = Path(unreal.Paths.project_dir()).resolve()
report_path = project / 'Saved/CamryNativeMigrationReport.json'
report = json.loads(report_path.read_text(encoding='utf-8'))
assert report['success']
levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
assert levels.load_level(report['map'])
assemblies = [a for a in actors.get_all_level_actors()
              if a.get_class().get_path_name() == '/Script/test1.CamryAssembly']
assert len(assemblies) == 1
components = {c.get_name(): c for c in assemblies[0].get_components_by_class(unreal.SceneComponent)}
assert set(components) == set(report['components'])
for name, expected in report['components'].items():
    c = components[name]
    t = c.get_world_transform()
    actual = [t.translation.x, t.translation.y, t.translation.z,
              t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w,
              t.scale3d.x, t.scale3d.y, t.scale3d.z]
    assert all(abs(a-b) < .001 for a,b in zip(actual, expected['world_transform'])), name
    assert (c.get_attach_parent().get_name() if c.get_attach_parent() else None) == expected['parent']
    if 'mesh' in expected:
        assert c.static_mesh and c.static_mesh.get_path_name() == expected['mesh'], name
        assert [c.get_material(i).get_path_name() if c.get_material(i) else None
                for i in range(c.get_num_materials())] == expected['materials'], name
if report['blueprint_removed']:
    assert not unreal.EditorAssetLibrary.does_asset_exist('/Game/DigitalTwin/Blueprints/BP_CamryAssembly')
report['cold_load_verified'] = True
report_path.write_text(json.dumps(report, indent=2), encoding='utf-8')
unreal.log('CAMRY_NATIVE_COLD_LOAD_SUCCESS')
