"""Create CAR_TEST from the existing whole-car assets using native actors.

Run in an editor commandlet; never replaces existing work maps or shared assets.
Use -FullVehicleVerifyOnly to check saved assets in a fresh process.
"""
import json
import traceback
from pathlib import Path
import unreal

PROJECT = Path(unreal.Paths.project_dir()).resolve()
SOURCE = PROJECT.parent.parent / 'car models/Camry_Converted/FullVehicle'
DEST = '/Game/DigitalTwin/Camry/FullVehicleTest'
MAP = '/Game/DigitalTwin/Maps/CAR_TEST'
REPORT = PROJECT / 'Saved/CAR_TESTReport.json'
MANIFEST_FILE = SOURCE / 'manifest.json'
if not MANIFEST_FILE.is_file():
    MANIFEST_FILE = PROJECT / 'Scripts/full_vehicle_manifest.json'
MANIFEST = json.loads(MANIFEST_FILE.read_text(encoding='utf-8'))
VERIFY_ONLY = '-FullVehicleVerifyOnly' in unreal.SystemLibrary.get_command_line()
ACTORS = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
LEVELS = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
SMS = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
if SMS is None:
    # Commandlets do not initialize this editor-module subsystem; these methods
    # operate directly on the supplied mesh, as in check_camry_body.py.
    SMS = unreal.new_object(unreal.StaticMeshEditorSubsystem)
ASSETS = unreal.AssetToolsHelpers.get_asset_tools()


def xyz(v):
    return [v.x, v.y, v.z]


def make_material(name, color, roughness, metallic=0.0, glass=False):
    path = DEST + '/Materials/M_FullVehicle_' + name
    if unreal.EditorAssetLibrary.does_asset_exist(path):
        return unreal.load_asset(path)
    mat = ASSETS.create_asset('M_FullVehicle_' + name, DEST + '/Materials',
                             unreal.Material, unreal.MaterialFactoryNew())
    mat.set_editor_property('two_sided', True)
    lib = unreal.MaterialEditingLibrary
    vector = lib.create_material_expression(mat, unreal.MaterialExpressionConstant3Vector, -300, 0)
    vector.set_editor_property('constant', unreal.LinearColor(*color, 1))
    lib.connect_material_property(vector, '', unreal.MaterialProperty.MP_BASE_COLOR)
    for prop, value, y in ((unreal.MaterialProperty.MP_ROUGHNESS, roughness, 100),
                           (unreal.MaterialProperty.MP_METALLIC, metallic, 200)):
        node = lib.create_material_expression(mat, unreal.MaterialExpressionConstant, -300, y)
        node.set_editor_property('r', value)
        lib.connect_material_property(node, '', prop)
    if glass:
        mat.set_editor_property('blend_mode', unreal.BlendMode.BLEND_TRANSLUCENT)
        opacity = lib.create_material_expression(mat, unreal.MaterialExpressionConstant, -300, 300)
        opacity.set_editor_property('r', 0.16)
        lib.connect_material_property(opacity, '', unreal.MaterialProperty.MP_OPACITY)
    lib.recompile_material(mat)
    assert unreal.EditorAssetLibrary.save_loaded_asset(mat)
    return mat


def import_mesh(entry, materials):
    asset = DEST + '/Meshes/' + entry['category'] + '/' + entry['name']
    if unreal.EditorAssetLibrary.does_asset_exist(asset):
        mesh = unreal.load_asset(asset)
    else:
        opts = unreal.FbxImportUI()
        for key, value in {
            'automated_import_should_detect_type': False,
            'mesh_type_to_import': unreal.FBXImportType.FBXIT_STATIC_MESH,
            'import_mesh': True, 'import_as_skeletal': False,
            'import_materials': False, 'import_textures': False,
        }.items():
            opts.set_editor_property(key, value)
        data = opts.get_editor_property('static_mesh_import_data')
        for key, value in {
            'combine_meshes': True, 'import_uniform_scale': 100.0,
            'convert_scene': False, 'convert_scene_unit': False,
            'auto_generate_collision': False, 'generate_lightmap_u_vs': False,
            'build_nanite': False,
            'normal_import_method': unreal.FBXNormalImportMethod.FBXNIM_COMPUTE_NORMALS,
        }.items():
            data.set_editor_property(key, value)
        task = unreal.AssetImportTask()
        for key, value in {
            'filename': str(SOURCE / entry['filename']),
            'destination_path': DEST + '/Meshes/' + entry['category'],
            'destination_name': entry['name'], 'automated': True,
            'replace_existing': False, 'save': False,
            'options': opts, 'factory': unreal.FbxFactory(),
        }.items():
            task.set_editor_property(key, value)
        ASSETS.import_asset_tasks([task])
        paths = task.get_editor_property('imported_object_paths')
        assert len(paths) == 1, (entry['name'], paths)
        mesh = unreal.load_asset(paths[0])
    assert isinstance(mesh, unreal.StaticMesh), asset
    mesh.set_editor_property('support_ray_tracing', False)
    mesh.set_editor_property('generate_mesh_distance_field', False)
    build = SMS.get_lod_build_settings(mesh, 0)
    build.set_editor_property('distance_field_resolution_scale', 0.0)
    build.set_editor_property('max_lumen_mesh_cards', 0)
    build.set_editor_property('use_mikk_t_space', False)
    SMS.set_lod_build_settings(mesh, 0, build)
    for slot in range(len(mesh.get_editor_property('static_materials'))):
        mesh.set_material(slot, materials[entry['material']])
    assert unreal.EditorAssetLibrary.save_loaded_asset(mesh)
    check_mesh(mesh, entry)
    unreal.log('CAMRY_FULL_IMPORTED ' + entry['name'])
    return mesh


def check_mesh(mesh, entry):
    b = mesh.get_bounds()
    actual_min = [b.origin.x-b.box_extent.x, b.origin.y-b.box_extent.y, b.origin.z-b.box_extent.z]
    actual_max = [b.origin.x+b.box_extent.x, b.origin.y+b.box_extent.y, b.origin.z+b.box_extent.z]
    low, high = entry['bounds_m']
    # FFbxDataConverter::ConvertPos flips Y even when convert_scene=False.
    # Match the handedness used by the existing Camry OBJ imports.
    expected_bounds = ([low[0], -high[1], low[2]], [high[0], -low[1], high[2]])
    for actual, expected in zip((actual_min, actual_max), expected_bounds):
        assert all(abs(a-e*100) < 0.15 for a, e in zip(actual, expected)), (entry['name'], actual, expected)
    assert not mesh.get_editor_property('support_ray_tracing')
    build = SMS.get_lod_build_settings(mesh, 0)
    assert build.get_editor_property('distance_field_resolution_scale') == 0


def place(cls, label, pos, rot=None, folder='PreviewSetup'):
    actor = ACTORS.spawn_actor_from_class(cls, unreal.Vector(*pos), rot or unreal.Rotator())
    assert actor
    actor.set_actor_label(label)
    actor.set_folder_path(folder)
    return actor


def create_map(meshes):
    assert LEVELS.new_level(MAP)
    world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    world.get_world_settings().set_editor_property('default_game_mode', unreal.GameModeBase)
    for entry, mesh in zip(MANIFEST['meshes'], meshes):
        actor = place(unreal.StaticMeshActor, entry['name'].replace('SM_CamryFull_', ''),
                      (250, 0, 0), folder='CamryFull/' + entry['category'])
        comp = actor.static_mesh_component
        assert comp.set_static_mesh(mesh)
        comp.set_collision_profile_name('NoCollision')
        comp.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
        actor.set_actor_enable_collision(False)
        comp.set_visible_in_ray_tracing(False)
        comp.set_affect_distance_field_lighting(False)
        comp.set_cast_shadow(False)
        # Retain all laminated glass layers, but render only one surface per pane.
        if not entry['visible']:
            comp.set_visibility(False)
            actor.set_actor_hidden_in_game(True)
            actor.set_folder_path('CamryFull/Glass/HiddenSourceLayers')
        actor.set_editor_property('tags', [unreal.Name('CamryFullVehicle'),
                                          unreal.Name('SourceParts_' + str(len(entry['parts'])))])
    floor = place(unreal.StaticMeshActor, 'PreviewFloor', (0, 0, -3))
    floor.static_mesh_component.set_static_mesh(unreal.load_asset('/Engine/BasicShapes/Plane'))
    floor.set_actor_scale3d(unreal.Vector(12, 12, 1))
    floor.static_mesh_component.set_visible_in_ray_tracing(False)
    floor.static_mesh_component.set_material(0, make_material('Floor', (.09, .11, .14), 0.9))
    for i, (yaw, intensity) in enumerate(((35, 4.0), (210, 2.0), (115, 1.0))):
        light = place(unreal.DirectionalLight, 'PreviewLight_' + str(i), (0, 0, 350),
                      unreal.Rotator(-35, yaw, 0))
        light.light_component.set_intensity(intensity)
        light.light_component.set_cast_shadows(False)
    pp = place(unreal.PostProcessVolume, 'Preview_Exposure_NoLumen', (0, 0, 0))
    pp.set_editor_property('unbound', True)
    settings = pp.get_editor_property('settings')
    for key, value in {
        'override_dynamic_global_illumination_method': True,
        'dynamic_global_illumination_method': unreal.DynamicGlobalIlluminationMethod.NONE,
        'override_reflection_method': True, 'reflection_method': unreal.ReflectionMethod.NONE,
        'override_auto_exposure_method': True, 'auto_exposure_method': unreal.AutoExposureMethod.AEM_MANUAL,
        'override_auto_exposure_apply_physical_camera_exposure': True,
        'auto_exposure_apply_physical_camera_exposure': False,
        'override_auto_exposure_bias': True, 'auto_exposure_bias': 0.0,
        'override_motion_blur_amount': True, 'motion_blur_amount': 0.0,
    }.items():
        settings.set_editor_property(key, value)
    pp.set_editor_property('settings', settings)
    overview_pos = unreal.Vector(580, -580, 340)
    overview_rot = unreal.MathLibrary.find_look_at_rotation(overview_pos, unreal.Vector(0, 0, 65))
    place(unreal.CameraActor, 'View_01_WholeVehicle', xyz(overview_pos), overview_rot, 'ReviewCameras')
    cabin_pos = unreal.Vector(65, -5, 128)
    cabin_rot = unreal.MathLibrary.find_look_at_rotation(cabin_pos, unreal.Vector(-120, 0, 65))
    place(unreal.CameraActor, 'View_02_InteriorSeats', xyz(cabin_pos), cabin_rot, 'ReviewCameras')
    place(unreal.PlayerStart, 'FreeFly_Start', xyz(overview_pos), overview_rot)
    unreal.EditorLevelLibrary.set_level_viewport_camera_info(overview_pos, overview_rot)
    assert LEVELS.save_current_level()


def verify_map():
    assert LEVELS.load_level(MAP)
    found = {}
    for actor in ACTORS.get_all_level_actors():
        if isinstance(actor, unreal.StaticMeshActor):
            mesh = actor.static_mesh_component.static_mesh
            if mesh and mesh.get_path_name().startswith(DEST + '/Meshes/'):
                assert mesh.get_name() not in found
                found[mesh.get_name()] = actor
    assert len(found) == len(MANIFEST['meshes'])
    results = []
    for entry in MANIFEST['meshes']:
        actor = found[entry['name']]
        mesh = actor.static_mesh_component.static_mesh
        check_mesh(mesh, entry)
        assert all(abs(a-b) < .001 for a, b in zip(xyz(actor.get_actor_location()), [250, 0, 0]))
        assert all(abs(a-1) < .001 for a in xyz(actor.get_actor_scale3d()))
        rotation = actor.get_actor_rotation()
        assert all(abs(v) < .001 for v in (rotation.pitch, rotation.yaw, rotation.roll))
        assert actor.static_mesh_component.get_editor_property('visible') == entry['visible']
        comp = actor.static_mesh_component
        assert str(comp.get_collision_profile_name()) == 'NoCollision', entry['name']
        assert comp.get_editor_property('body_instance').get_editor_property('collision_enabled') == unreal.CollisionEnabled.NO_COLLISION, entry['name']
        assert not actor.get_actor_enable_collision(), entry['name']
        assert comp.get_collision_enabled() == unreal.CollisionEnabled.NO_COLLISION, entry['name']
        assert mesh.get_num_triangles(0) > 0, entry['name']
        assert all(mesh.get_material(i) is not None for i in range(len(mesh.get_editor_property('static_materials')))), entry['name']
        results.append({'mesh': mesh.get_path_name(), 'category': entry['category'],
                        'visible': entry['visible'], 'source_parts': len(entry['parts']),
                        'unreal_triangles': mesh.get_num_triangles(0),
                        'source_triangles': entry['triangles']})
    return results


def main():
    if not VERIFY_ONLY:
        if unreal.EditorAssetLibrary.does_asset_exist(MAP):
            raise RuntimeError('Test map already exists. Refusing to replace it; use verify-only.')
        # Reuse the imported geometry without rebuilding or resaving shared meshes.
        meshes = []
        for entry in MANIFEST['meshes']:
            asset = DEST + '/Meshes/' + entry['category'] + '/' + entry['name']
            mesh = unreal.load_asset(asset)
            assert isinstance(mesh, unreal.StaticMesh), 'Missing imported asset: ' + asset
            check_mesh(mesh, entry)
            meshes.append(mesh)
        create_map(meshes)
    info = verify_map()
    report = {
        'success': True, 'map': MAP, 'meshes': info,
        'mesh_count': len(info), 'source_parts': sum(len(x['parts']) for x in MANIFEST['meshes']),
        'triangles': sum(x['triangles'] for x in MANIFEST['meshes']),
        'source_alignment_preserved': True, 'cold_load_verified': VERIFY_ONLY,
        'coordinate_conversion': 'source meters (x,y,z) -> Unreal centimeters (100*x,-100*y,100*z); actor offset (250,0,0)',
        'viewport_render_verified': False,
        'limitations': MANIFEST['limitations'],
    }
    path = REPORT.with_name('CAR_TESTColdLoadReport.json') if VERIFY_ONLY else REPORT
    path.write_text(json.dumps(report, indent=2), encoding='utf-8')
    unreal.log('CAR_TEST_SUCCESS ' + str(path))


try:
    main()
except Exception:
    REPORT.with_name('CAR_TESTError.json').write_text(
        json.dumps({'success': False, 'error': traceback.format_exc()}, indent=2), encoding='utf-8')
    raise
