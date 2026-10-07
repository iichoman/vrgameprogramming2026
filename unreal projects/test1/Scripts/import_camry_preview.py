"""Run with UnrealEditor-Cmd -run=pythonscript; imports a door surface preview."""
import json
from pathlib import Path
import traceback
import unreal

PROJECT = Path(unreal.Paths.project_dir()).resolve()
ROOT = PROJECT.parent.parent
REPORT = PROJECT / 'Saved' / 'CamryImportReport.json'
DEST = '/Game/DigitalTwin/Camry/FrontDoorPreview'
MAP = '/Game/DigitalTwin/Maps/CamryDoorPreview'
SOURCE = ROOT / 'car models/Camry_Converted/camry_frontdoor_named_parts_m.obj'


def main():
    if unreal.EditorAssetLibrary.does_asset_exist(MAP):
        raise RuntimeError('Preview map already exists; refusing to overwrite it.')
    split_dir = SOURCE.parent / 'FrontDoorParts'
    split_dir.mkdir(exist_ok=True)
    split_files = []
    stream = None
    total_vertices = offset = 0
    try:
        for line in SOURCE.read_text(encoding='utf-8').splitlines():
            if line.startswith('o '):
                if stream:
                    stream.close()
                offset = total_vertices
                path = split_dir / (line[2:] + '.obj')
                split_files.append(path)
                stream = path.open('w', encoding='utf-8')
                stream.write(line + '\n')
            elif line.startswith('v '):
                stream.write(line + '\n')
                total_vertices += 1
            elif line.startswith('f '):
                ids = [int(v) - offset for v in line.split()[1:]]
                assert min(ids) > 0
                stream.write('f ' + ' '.join(map(str, ids)) + '\n')
    finally:
        if stream:
            stream.close()
    options = unreal.FbxImportUI()
    options.set_editor_property('automated_import_should_detect_type', False)
    options.set_editor_property('mesh_type_to_import', unreal.FBXImportType.FBXIT_STATIC_MESH)
    options.set_editor_property('import_mesh', True)
    options.set_editor_property('import_materials', False)
    options.set_editor_property('import_textures', False)
    options.set_editor_property('import_as_skeletal', False)
    data = options.get_editor_property('static_mesh_import_data')
    for key, value in {'combine_meshes': False, 'import_uniform_scale': 100.0,
                       'convert_scene': False, 'convert_scene_unit': False,
                       'auto_generate_collision': False,
                       'generate_lightmap_u_vs': False,
                       'normal_import_method': unreal.FBXNormalImportMethod.FBXNIM_COMPUTE_NORMALS}.items():
        data.set_editor_property(key, value)
    tasks = []
    for path in split_files:
        task = unreal.AssetImportTask()
        task.set_editor_property('filename', str(path))
        task.set_editor_property('destination_path', DEST + '/Parts')
        task.set_editor_property('destination_name', 'SM_' + path.stem)
        task.set_editor_property('automated', True)
        task.set_editor_property('replace_existing', False)
        task.set_editor_property('save', True)
        task.set_editor_property('options', options)
        task.set_editor_property('factory', unreal.FbxFactory())
        tasks.append(task)
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks(tasks)
    paths = [p for task in tasks for p in task.get_editor_property('imported_object_paths')]
    meshes = [unreal.load_asset(p) for p in paths]
    meshes = [m for m in meshes if isinstance(m, unreal.StaticMesh)]
    if len(meshes) != 6:
        raise RuntimeError(f'Expected 6 separate door meshes, got {len(meshes)}: {paths}')

    material = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        'M_CamryPreview', DEST, unreal.Material, unreal.MaterialFactoryNew())
    material.set_editor_property('two_sided', True)
    color = unreal.MaterialEditingLibrary.create_material_expression(
        material, unreal.MaterialExpressionConstant3Vector, -200, 0)
    color.set_editor_property('constant', unreal.LinearColor(0.25, 0.42, 0.55, 1))
    unreal.MaterialEditingLibrary.connect_material_property(color, '', unreal.MaterialProperty.MP_BASE_COLOR)
    unreal.MaterialEditingLibrary.recompile_material(material)
    unreal.EditorAssetLibrary.save_loaded_asset(material)
    mesh_info = []
    for mesh in meshes:
        for slot in range(len(mesh.get_editor_property('static_materials'))):
            mesh.set_material(slot, material)
        unreal.EditorAssetLibrary.save_loaded_asset(mesh)
        bounds = mesh.get_bounds()
        mesh_info.append({'asset': mesh.get_path_name(),
                          'origin_cm': [bounds.origin.x, bounds.origin.y, bounds.origin.z],
                          'extent_cm': [bounds.box_extent.x, bounds.box_extent.y, bounds.box_extent.z]})

    if not unreal.EditorLevelLibrary.new_level(MAP):
        raise RuntimeError('Could not create preview map')
    for mesh in meshes:
        # Every mesh retains source coordinates. Apply the same display translation.
        actor = unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.StaticMeshActor, unreal.Vector(250, 0, 0))
        actor.static_mesh_component.set_static_mesh(mesh)
        actor.set_actor_label(mesh.get_name())
    floor_mesh = unreal.load_asset('/Engine/BasicShapes/Plane.Plane')
    floor = unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.StaticMeshActor, unreal.Vector(0, 0, -5))
    floor.static_mesh_component.set_static_mesh(floor_mesh)
    floor.set_actor_scale3d(unreal.Vector(8, 8, 1))
    floor.set_actor_label('PreviewFloor')
    for yaw in (30, 210):
        light = unreal.EditorLevelLibrary.spawn_actor_from_class(
            unreal.DirectionalLight, unreal.Vector(0, 0, 300), unreal.Rotator(-40, yaw, 0))
        light.set_actor_label('PreviewLight_' + str(yaw))
    unreal.EditorLevelLibrary.set_level_viewport_camera_info(
        unreal.Vector(350, -380, 230), unreal.Rotator(-18, 140, 0))
    if not unreal.EditorLevelLibrary.save_current_level():
        raise RuntimeError('Could not save preview map')
    REPORT.write_text(json.dumps({'success': True, 'source': str(SOURCE), 'map': MAP,
                                  'meshes': mesh_info, 'scale_m_to_cm': 100,
                                  'limitations': 'Shell surface preview only. No thickness or physics.'},
                                 indent=2), encoding='utf-8')
    unreal.log('CAMRY_IMPORT_SUCCESS: ' + str(REPORT))


try:
    main()
except Exception:
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps({'success': False, 'error': traceback.format_exc()}, indent=2), encoding='utf-8')
    raise
