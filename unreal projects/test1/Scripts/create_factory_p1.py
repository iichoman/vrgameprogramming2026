"""Create the initial factory blockout once; never rerun over an edited map.

Run using UnrealEditor-Cmd -run=pythonscript -script=<this file>.
Use -FactoryP1VerifyOnly for a read-only cold-load audit.
No custom classes, Blueprint assets, production equipment or process logic.
"""
import json
import traceback
from pathlib import Path
import unreal

PROJECT = Path(unreal.Paths.project_dir()).resolve()
MAP = '/Game/DigitalTwin/Maps/factory_p1'
MATERIALS = '/Game/DigitalTwin/Factory/Blockout/Materials'
LAYOUT = json.loads((PROJECT / 'Scripts/factory_p1_layout.json').read_text(encoding='utf-8'))
ACTORS = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
LEVELS = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
REPORT = PROJECT / 'Saved/FactoryP1Audit.json'
CUBE = None
MATS = {}


def require_initial_blockout():
    """Never regenerate a map upgraded to the native runtime, even with repair flags.

    Check the serialized name table before loading as well: an older DLL can
    fail to load a new C++ actor, which must not make an upgraded map appear safe
    to replace. A conservative name match is intentional for this legacy tool.
    """
    map_file = PROJECT / 'Content/DigitalTwin/Maps/factory_p1.umap'
    if map_file.is_file():
        data = map_file.read_bytes()
        if b'FactoryRuntime' in data or 'FactoryRuntime'.encode('utf-16-le') in data:
            raise RuntimeError(
                'factory_p1 contains FactoryRuntime. The legacy blockout generator '
                'cannot modify or verify an upgraded map, including with '
                '-FactoryP1RebuildDraft. Use Play editing or '
                'install_factory_runtime.py -FactoryRuntimeVerifyOnly instead.'
            )
    if unreal.EditorAssetLibrary.does_asset_exist(MAP):
        assert LEVELS.load_level(MAP), 'Cannot inspect the existing factory_p1'
        for actor in ACTORS.get_all_level_actors():
            tags = {str(tag) for tag in actor.get_editor_property('tags')}
            if (actor.get_class().get_path_name() in (
                    '/Script/test1.FactoryRuntime', '/Script/test1.FactoryEquipment')
                    or str(actor.get_folder_path()).startswith('FactoryRuntime')
                    or 'FactoryRuntime' in tags):
                raise RuntimeError(
                    'FactoryRuntime is installed. Refusing legacy blockout regeneration. '
                    'Use install_factory_runtime.py -FactoryRuntimeVerifyOnly.'
                )


def vector(values):
    return unreal.Vector(*values)


def spawn(cls, label, position, folder, rotation=None, extra_tags=()):
    actor = ACTORS.spawn_actor_from_class(cls, vector(position), rotation or unreal.Rotator())
    assert actor, label
    actor.set_actor_label(label)
    actor.set_folder_path('FactoryP1/' + folder)
    actor.set_editor_property('tags', [unreal.Name(t) for t in ('FactoryP1', label, *extra_tags)])
    return actor


def material(name, color):
    path = MATERIALS + '/M_FP1_' + name
    existing = unreal.load_asset(path) if unreal.EditorAssetLibrary.does_asset_exist(path) else None
    if existing:
        return existing
    asset = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        'M_FP1_' + name, MATERIALS, unreal.Material, unreal.MaterialFactoryNew())
    lib = unreal.MaterialEditingLibrary
    rgb = lib.create_material_expression(asset, unreal.MaterialExpressionConstant3Vector, -300, 0)
    rgb.set_editor_property('constant', unreal.LinearColor(*color, 1))
    lib.connect_material_property(rgb, '', unreal.MaterialProperty.MP_BASE_COLOR)
    rough = lib.create_material_expression(asset, unreal.MaterialExpressionConstant, -300, 120)
    rough.set_editor_property('r', 0.95)
    lib.connect_material_property(rough, '', unreal.MaterialProperty.MP_ROUGHNESS)
    lib.recompile_material(asset)
    assert unreal.EditorAssetLibrary.save_loaded_asset(asset)
    return asset


def box(label, position, size, mat, folder, solid=False, tags=()):
    actor = spawn(unreal.StaticMeshActor, label, position, folder, extra_tags=tags)
    comp = actor.static_mesh_component
    assert comp.set_static_mesh(CUBE)
    actor.set_actor_scale3d(vector([v / 100.0 for v in size]))
    comp.set_material(0, MATS[mat])
    comp.set_cast_shadow(False)
    comp.set_visible_in_ray_tracing(False)
    comp.set_affect_distance_field_lighting(False)
    comp.set_collision_profile_name('BlockAll' if solid else 'NoCollision')
    comp.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS if solid else unreal.CollisionEnabled.NO_COLLISION)
    actor.set_actor_enable_collision(solid)
    return actor


def text(label, value, position, size=75, color=(230, 236, 240, 255), rotation=None):
    actor = spawn(unreal.TextRenderActor, label, position, '40_Labels',
                  rotation or unreal.Rotator(pitch=90, yaw=90, roll=0))
    comp = actor.get_component_by_class(unreal.TextRenderComponent)
    comp.set_text(unreal.Text(value))
    comp.set_world_size(size)
    comp.set_horizontal_alignment(unreal.HorizTextAligment.EHTA_CENTER)
    comp.set_text_render_color(unreal.Color(*color))
    comp.set_cast_shadow(False)
    return actor


def outline(label, center, size, mat, folder, width=10):
    x, y = center
    dx, dy = size
    for suffix, pos, dim in (
        ('N', (x, y+dy/2, 1.8), (dx, width, 1)),
        ('S', (x, y-dy/2, 1.8), (dx, width, 1)),
        ('E', (x+dx/2, y, 1.8), (width, dy, 1)),
        ('W', (x-dx/2, y, 1.8), (width, dy, 1)),
    ):
        box(label + '_' + suffix, pos, dim, mat, folder)


def arrow(label, x, y, scale=1, mat='Route'):
    box(label + '_Stem', (x-35*scale, y, 3), (120*scale, 24*scale, 2), mat, '30_Routes')
    for side in (-1, 1):
        a = box(label + ('_L' if side < 0 else '_R'), (x+12*scale, y+22*side*scale, 3),
                (70*scale, 24*scale, 2), mat, '30_Routes')
        a.set_actor_rotation(unreal.Rotator(pitch=0, yaw=-side*45, roll=0), False)


def setup_view():
    for i, (yaw, intensity) in enumerate(((35, 3.5), (210, 2.0), (115, 1.0))):
        actor = spawn(unreal.DirectionalLight, 'FP1_Light_' + str(i), (3600, 0, 1000),
                      '90_Environment', unreal.Rotator(pitch=-50, yaw=yaw, roll=0))
        actor.light_component.set_intensity(intensity)
        actor.light_component.set_cast_shadows(False)
        actor.light_component.set_editor_property('forward_shading_priority', 10-i)
    pp = spawn(unreal.PostProcessVolume, 'FP1_Exposure_NoLumen', (0, 0, 0), '90_Environment')
    pp.set_editor_property('unbound', True)
    s = pp.get_editor_property('settings')
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
        s.set_editor_property(key, value)
    pp.set_editor_property('settings', s)
    pos = vector((3600, 5700, 6200))
    rot = unreal.MathLibrary.find_look_at_rotation(pos, vector((3600, 0, 0)))
    cam = spawn(unreal.CameraActor, 'FP1_View_Overview', (pos.x, pos.y, pos.z), '80_Views', rot)
    cam.camera_component.set_field_of_view(58)
    cam.camera_component.set_editor_property('aspect_ratio', 16.0/9.0)
    top = spawn(unreal.CameraActor, 'FP1_View_Plan', (3600, 0, 10000), '80_Views', unreal.Rotator(pitch=-90, yaw=-90, roll=0))
    top.camera_component.set_editor_property('projection_mode', unreal.CameraProjectionMode.ORTHOGRAPHIC)
    top.camera_component.set_editor_property('ortho_width', 8200)
    top.camera_component.set_editor_property('aspect_ratio', 16.0/9.0)
    # DefaultPawn free flight; start near the wheel bay rather than 80 m away.
    start = vector((3400, 1600, 900))
    start_rot = unreal.MathLibrary.find_look_at_rotation(start, vector((4200, 0, 100)))
    spawn(unreal.PlayerStart, 'FP1_FreeFly_Start', (start.x, start.y, start.z), '80_Views', start_rot)
    unreal.EditorLevelLibrary.set_level_viewport_camera_info(pos, rot)


def build():
    global CUBE, MATS
    exists = unreal.EditorAssetLibrary.does_asset_exist(MAP)
    # This switch is only for repairing this initial build before handover.
    if exists and '-FactoryP1RebuildDraft' not in unreal.SystemLibrary.get_command_line():
        raise RuntimeError('factory_p1 already exists. Open and edit the map; do not regenerate it.')
    require_initial_blockout()
    CUBE = unreal.load_asset('/Engine/BasicShapes/Cube')
    assert CUBE
    for name, color in LAYOUT['palette'].items():
        MATS[name] = material(name, color)
    if exists:
        assert LEVELS.load_level(MAP)
        owned = [a for a in ACTORS.get_all_level_actors()
                 if 'FactoryP1' in [str(t) for t in a.get_editor_property('tags')]]
        assert owned, 'Refuse to replace a map without FactoryP1-owned actors'
        for actor in owned:
            assert ACTORS.destroy_actor(actor)
    else:
        assert LEVELS.new_level(MAP)
    world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    world.get_world_settings().set_editor_property('default_game_mode', unreal.GameModeBase)
    box('FP1_Floor', (3600, 0, -15), (7200, 4000, 30), 'Floor', '00_Shell', solid=True)
    # Low cutaway perimeter; columns indicate a future 7 m clear-height envelope.
    for sign in (-1, 1):
        box('FP1_Wall_' + ('North' if sign > 0 else 'South'), (3600, sign*2000, 60),
            (7200, 24, 120), 'Shell', '00_Shell', solid=True)
    for x in (0, 7200):
        for sign in (-1, 1):
            box('FP1_EndWall_%s_%s' % (x, sign), (x, sign*1200, 60),
                (24, 1600, 120), 'Shell', '00_Shell', solid=True)
        outline('FP1_Gate_' + str(x), (x, 0), (180, 800), 'Route', '00_Shell')
    for x in range(0, 7201, 1200):
        for sign in (-1, 1):
            box('FP1_Column_%s_%s' % (x, sign), (x, sign*1980, 350),
                (35, 35, 700), 'Column', '00_Shell', solid=True)
    # Space allocation only. These colored surfaces are not machine foundations.
    for bay in LAYOUT['bays']:
        ident, x = bay['id'], bay['center_cm'][0]
        mat = 'Wheel' if ident == 'P04' else 'Cell'
        box('FP1_Zone_' + ident, (x, 0, 0.5), (1160, 1040, 1), mat, '10_ProcessReservations',
            tags=('ReservedProcessZone', 'Zone_' + ident))
        outline('FP1_Boundary_' + ident, (x, 0), (1160, 1040), 'WheelLine' if ident == 'P04' else 'Line', '10_ProcessReservations')
        text('FP1_Title_' + ident, bay['title'], (x, 390, 5), 120)
        text('FP1_Subtitle_' + ident, bay['subtitle'], (x, 265, 5), 56)
        text('FP1_Status_' + ident, 'DEVELOP FIRST' if ident == 'P04' else 'RESERVED', (x, -440, 5), 60)
        # Suggested travel envelope, not a physical conveyor or validated clearance.
        for y in (-200, 200):
            box('FP1_CarPath_%s_%s' % (ident, y), (x, y, 2.5), (1120, 8, 1), 'Route', '30_Routes')
        arrow('FP1_Flow_' + ident, x, 0, 1.6)
    for row in LAYOUT['side_zones']:
        ident = row['id']
        x, y = row['center_cm']
        mat = row['material']
        box('FP1_Zone_' + ident, (x, y, 0.5), (*row['size_cm'], 1), mat, '20_SupportReservations',
            tags=('ReservedSupportZone', 'Zone_' + ident))
        outline('FP1_Boundary_' + ident, (x, y), row['size_cm'], 'SupplyLine' if mat == 'Supply' else 'Line', '20_SupportReservations')
        text('FP1_Title_' + ident, row['title'], (x, y+140, 5), 100)
        text('FP1_Subtitle_' + ident, row['subtitle'], (x, y-15, 5), 55)
    # Longitudinal pedestrian planning strips. Crossings are deliberately explicit.
    for side in (-1, 1):
        y = 700*side
        box('FP1_Walkway_' + str(side), (3600, y, 1), (7000, 200, 2), 'Walk', '30_Routes')
        for edge in (-100, 100):
            box('FP1_WalkEdge_%s_%s' % (side, edge), (3600, y+edge, 2.5), (7000, 6, 1), 'WalkLine', '30_Routes')
        for x in (1200, 3600, 6000):
            text('FP1_WalkLabel_%s_%s' % (side, x), 'WALKWAY - PLANNING', (x, y-25, 5), 42)
    # Potential transfer points across walkways: warning marks, not safety devices.
    for x in (600, 1800, 3000, 4200, 5400):
        for y in (610, 645, 680, 715, 750, 785):
            box('FP1_SupplyCross_%s_%s' % (x, y), (x, y, 3.5), (140, 10, 1), 'SupplyLine', '30_Routes')
    text('FP1_ProjectTitle', 'FACTORY P1 / ASSEMBLY STUDY', (3600, 1830, 5), 120)
    text('FP1_AssumptionLabel', '72 x 40 m  |  DRAFT BLOCKOUT  |  NO EQUIPMENT INSTALLED', (3600, -1870, 5), 75)
    setup_view()
    assert LEVELS.save_current_level()


def verify():
    require_initial_blockout()
    assert LEVELS.load_level(MAP), 'Cannot cold-load factory_p1'
    actors = list(ACTORS.get_all_level_actors())
    owned = [a for a in actors if 'FactoryP1' in [str(t) for t in a.get_editor_property('tags')]]
    labels = {a.get_actor_label(): a for a in owned}
    assert len(labels) == len(owned), 'Duplicate planning IDs'
    expected = [b['id'] for b in LAYOUT['bays']] + [s['id'] for s in LAYOUT['side_zones']]
    for ident in expected:
        assert 'FP1_Zone_' + ident in labels, ident
    floor = labels['FP1_Floor']
    origin, extent = floor.get_actor_bounds(False)
    assert abs(origin.z + extent.z) < 0.1, 'Floor top must be Z=0'
    assert abs(extent.x*2-7200) < 0.1 and abs(extent.y*2-4000) < 0.1
    assert floor.static_mesh_component.get_collision_profile_name() == 'BlockAll'
    assert 'FP1_FreeFly_Start' in labels
    for name in ('FP1_View_Overview', 'FP1_View_Plan'):
        assert isinstance(labels[name], unreal.CameraActor)
    for actor in owned:
        assert actor.get_class().get_path_name().startswith('/Script/Engine.'), actor.get_class().get_path_name()
        if isinstance(actor, unreal.StaticMeshActor) and str(actor.get_folder_path()) != 'FactoryP1/00_Shell':
            assert actor.static_mesh_component.get_collision_enabled() == unreal.CollisionEnabled.NO_COLLISION
    world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    assert world.get_world_settings().get_editor_property('default_game_mode').get_path_name() == '/Script/Engine.GameModeBase'
    report = {'success': True, 'map': MAP, 'owned_actor_count': len(owned),
              'zone_ids': expected, 'hall_cm': [7200, 4000], 'floor_top_cm': 0,
              'native_engine_actors_only': True, 'equipment_installed': False,
              'planning_dimensions_not_validated_clearances': True}
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(report, indent=2), encoding='utf-8')
    unreal.log('FACTORY_P1_AUDIT ' + json.dumps(report))


try:
    if '-FactoryP1VerifyOnly' not in unreal.SystemLibrary.get_command_line():
        build()
    verify()
except Exception:
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps({'success': False, 'error': traceback.format_exc()}, indent=2), encoding='utf-8')
    raise
