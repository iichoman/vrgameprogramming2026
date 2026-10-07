"""Import selected body-shell groups and create a body + doors review map."""
from pathlib import Path
import json
import traceback
import unreal

project = Path(unreal.Paths.project_dir()).resolve()
source = project.parent.parent/'car models/Camry_Converted/BodyGroups'
manifest = json.loads((source/'manifest.json').read_text(encoding='utf-8'))
dest = '/Game/DigitalTwin/Camry/Body'
map_path = '/Game/DigitalTwin/Maps/CamryBodyDoorPreview'
report_path = project/'Saved/CamryBodyImportReport.json'

def main():
    if unreal.EditorAssetLibrary.does_asset_exist(map_path) and (not report_path.exists() or json.loads(report_path.read_text(encoding='utf-8')).get('success')):
        raise RuntimeError('Review map already exists; will not overwrite user changes')
    options = unreal.FbxImportUI()
    for key, value in {'automated_import_should_detect_type': False,
                       'mesh_type_to_import': unreal.FBXImportType.FBXIT_STATIC_MESH,
                       'import_mesh': True, 'import_as_skeletal': False,
                       'import_materials': False, 'import_textures': False}.items():
        options.set_editor_property(key, value)
    data = options.get_editor_property('static_mesh_import_data')
    for key, value in {'combine_meshes': True, 'import_uniform_scale': 100.0,
                       'convert_scene': False, 'convert_scene_unit': False,
                       'auto_generate_collision': False, 'generate_lightmap_u_vs': False,
                       'normal_import_method': unreal.FBXNormalImportMethod.FBXNIM_COMPUTE_NORMALS}.items():
        data.set_editor_property(key, value)
    tools = unreal.AssetToolsHelpers.get_asset_tools()
    imported, info = [], []
    for name, expected in manifest.items():
        asset_path = dest + '/SM_Camry_' + name
        if unreal.EditorAssetLibrary.does_asset_exist(asset_path):
            mesh = unreal.load_asset(asset_path)
        else:
            task = unreal.AssetImportTask()
            for key, value in {'filename': str(source/(name+'.obj')),
                               'destination_path': dest, 'destination_name': 'SM_Camry_'+name,
                               'automated': True, 'replace_existing': False, 'save': True,
                               'options': options, 'factory': unreal.FbxFactory()}.items():
                task.set_editor_property(key, value)
            tools.import_asset_tasks([task])
            results = task.get_editor_property('imported_object_paths')
            assert len(results) == 1, results
            mesh = unreal.load_asset(results[0])
        assert isinstance(mesh, unreal.StaticMesh)
        b = mesh.get_bounds()
        actual = [2*b.box_extent.x, 2*b.box_extent.y, 2*b.box_extent.z]
        expected_cm = [(expected['max_m'][i]-expected['min_m'][i])*100 for i in range(3)]
        assert all(abs(a-e) < .1 for a,e in zip(actual, expected_cm)), (name, actual, expected_cm)
        imported.append(mesh)
        info.append({'group': name, 'asset': mesh.get_path_name(), 'parts': len(expected['parts']),
                     'source_faces': expected['faces'], 'dimensions_cm': actual})
        unreal.log('CAMRY_BODY_GROUP_IMPORTED: '+name)
    material_path = dest+'/M_CamryBody'
    material = unreal.load_asset(material_path) if unreal.EditorAssetLibrary.does_asset_exist(material_path) else tools.create_asset(
        'M_CamryBody', dest, unreal.Material, unreal.MaterialFactoryNew())
    material.set_editor_property('two_sided', True)
    expression = unreal.MaterialEditingLibrary.create_material_expression(material, unreal.MaterialExpressionConstant3Vector, -200, 0)
    expression.set_editor_property('constant', unreal.LinearColor(.45, .48, .52, 1))
    unreal.MaterialEditingLibrary.connect_material_property(expression, '', unreal.MaterialProperty.MP_BASE_COLOR)
    unreal.MaterialEditingLibrary.recompile_material(material)
    unreal.EditorAssetLibrary.save_loaded_asset(material)
    for mesh in imported:
        for slot in range(len(mesh.get_editor_property('static_materials'))):
            mesh.set_material(slot, material)
        unreal.EditorAssetLibrary.save_loaded_asset(mesh)
    level = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    if unreal.EditorAssetLibrary.does_asset_exist(map_path):
        assert level.load_level(map_path)
    else:
        assert level.new_level_from_template(map_path, '/Game/DigitalTwin/Maps/CamryDoorPreview')
    door_actors = [a for a in actors.get_all_level_actors() if isinstance(a, unreal.StaticMeshActor)
                   and a.static_mesh_component.static_mesh
                   and '/FrontDoorPreview/Parts/' in a.static_mesh_component.static_mesh.get_path_name()]
    assert len(door_actors) == 6
    for actor in door_actors:
        actor.set_folder_path('Camry/FrontDoors')
        p = actor.get_actor_location()
        assert abs(p.x-250) < .01 and abs(p.y) < .01 and abs(p.z) < .01
    for mesh in imported:
        actor = actors.spawn_actor_from_class(unreal.StaticMeshActor, unreal.Vector(250, 0, 0))
        assert actor is not None
        assert actor.static_mesh_component.set_static_mesh(mesh)
        actor.set_actor_label(mesh.get_name())
        actor.set_folder_path('Camry/Body')
    # Camera actor gives the user a persistent review viewpoint independent of editor viewport state.
    camera = actors.spawn_actor_from_class(unreal.CameraActor, unreal.Vector(620, -620, 330), unreal.Rotator(-18, 135, 0))
    camera.set_actor_label('Camry_OverviewCamera')
    assert level.save_current_level()
    assert level.load_level(map_path)
    saved = actors.get_all_level_actors()
    body = [a for a in saved if isinstance(a, unreal.StaticMeshActor) and a.static_mesh_component.static_mesh in imported]
    doors = [a for a in saved if isinstance(a, unreal.StaticMeshActor) and a.static_mesh_component.static_mesh
             and '/FrontDoorPreview/Parts/' in a.static_mesh_component.static_mesh.get_path_name()]
    assert len(body) == 5 and len(doors) == 6
    assert all(abs(a.get_actor_location().x-250) < .01 and abs(a.get_actor_location().y) < .01 and abs(a.get_actor_location().z) < .01 for a in body+doors)
    report_path.write_text(json.dumps({'success': True, 'map': map_path, 'groups': info,
        'body_parts': sum(len(v['parts']) for v in manifest.values()),
        'body_mesh_actors': len(body), 'door_mesh_actors': len(doors), 'source_alignment_preserved': True,
        'viewport_render_verified': False,
        'limitations': 'Selected body shell surfaces grouped for preview; no thickness, collision or assembly physics.'}, indent=2), encoding='utf-8')
    unreal.log('CAMRY_BODY_IMPORT_SUCCESS')

try:
    main()
except Exception:
    report_path.write_text(json.dumps({'success': False, 'error': traceback.format_exc()}, indent=2), encoding='utf-8')
    raise
