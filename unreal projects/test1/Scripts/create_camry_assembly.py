"""Build a reusable component Blueprint while preserving approved mesh placement."""
import json
from pathlib import Path
import traceback
import unreal

project = Path(unreal.Paths.project_dir()).resolve()
report_path = project/'Saved/CamryAssemblyReport.json'
BP_PATH = '/Game/DigitalTwin/Blueprints/BP_CamryAssembly'
MAP_PATH = '/Game/DigitalTwin/Maps/CamryAssemblyPreview'
SOURCE_MAP = '/Game/DigitalTwin/Maps/CamryBodyDoorPreview'

def xyz(v):
    return [v.x, v.y, v.z]

def close(a, b, tolerance=.01):
    return all(abs(x-y) < tolerance for x,y in zip(a,b))

def main():
    if unreal.EditorAssetLibrary.does_asset_exist(BP_PATH) or unreal.EditorAssetLibrary.does_asset_exist(MAP_PATH):
        raise RuntimeError('Assembly asset/map already exists; refusing to overwrite.')
    levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    assert levels.load_level(SOURCE_MAP)
    sources = []
    for actor in actors.get_all_level_actors():
        if not isinstance(actor, unreal.StaticMeshActor):
            continue
        mesh = actor.static_mesh_component.static_mesh
        if mesh is None:
            continue
        path = mesh.get_path_name()
        if '/Camry/Body/' in path:
            group = 'Body'
        elif '/FrontDoorPreview/Parts/' in path:
            group = 'LeftDoor' if any(n in path for n in ('p2000352_', 'p2000358_', 'p2000359_')) else 'RightDoor'
        else:
            continue
        r = actor.get_actor_rotation()
        scale = actor.get_actor_scale3d()
        assert close([r.pitch,r.yaw,r.roll], [0,0,0]) and close(xyz(scale), [1,1,1]), 'Unexpected source rotation/scale'
        sources.append({'mesh': mesh, 'group': group, 'location': xyz(actor.get_actor_location())})
    assert {g:sum(s['group']==g for s in sources) for g in ('Body','LeftDoor','RightDoor')} == {'Body':5,'LeftDoor':3,'RightDoor':3}
    pivots = {'Body': [0,0,0]}
    for group in ('LeftDoor','RightDoor'):
        boxes = [(s, s['mesh'].get_bounds()) for s in sources if s['group']==group]
        lower = [min(s['location'][a]+xyz(b.origin)[a]-xyz(b.box_extent)[a] for s,b in boxes) for a in range(3)]
        upper = [max(s['location'][a]+xyz(b.origin)[a]+xyz(b.box_extent)[a] for s,b in boxes) for a in range(3)]
        pivots[group] = [(a+b)/2 for a,b in zip(lower,upper)]

    factory = unreal.BlueprintFactory()
    factory.set_editor_property('parent_class', unreal.Actor)
    bp = unreal.AssetToolsHelpers.get_asset_tools().create_asset('BP_CamryAssembly', '/Game/DigitalTwin/Blueprints', unreal.Blueprint, factory)
    assert bp is not None
    subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
    assert subsystem is not None
    library = unreal.SubobjectDataBlueprintFunctionLibrary
    handles = subsystem.k2_gather_subobject_data_for_blueprint(bp)
    context = handles[0]

    def add(parent, name, cls):
        params = unreal.AddNewSubobjectParams(parent_handle=parent, new_class=cls, blueprint_context=bp)
        handle, reason = subsystem.add_new_subobject(params)
        assert library.is_handle_valid(handle), str(reason)
        assert subsystem.rename_subobject(handle, unreal.Text(name))
        data = library.get_data(handle)
        obj = library.get_object_for_blueprint(data, bp)
        assert isinstance(obj, unreal.SceneComponent), (name, str(obj))
        obj.set_editor_property('mobility', unreal.ComponentMobility.MOVABLE)
        return handle, obj

    root_h, root = add(context, 'VehicleRoot', unreal.SceneComponent)
    assert subsystem.make_new_scene_root(context, root_h, bp)
    groups = {}
    for name, pivot in pivots.items():
        handle, component = add(root_h, name, unreal.SceneComponent)
        component.set_editor_property('relative_location', unreal.Vector(*pivot))
        groups[name] = handle
    for index,s in enumerate(sources):
        name = s['group']+'_'+s['mesh'].get_name().removeprefix('SM_')
        _, component = add(groups[s['group']], name, unreal.StaticMeshComponent)
        component.set_editor_property('static_mesh', s['mesh'])
        component.set_editor_property('relative_location', unreal.Vector(*[a-b for a,b in zip(s['location'],pivots[s['group']])]))
        component.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
        component.set_editor_property('component_tags', [unreal.Name(s['group'])])
    unreal.BlueprintEditorLibrary.compile_blueprint(bp)
    assert unreal.EditorAssetLibrary.save_loaded_asset(bp)
    cls = unreal.EditorAssetLibrary.load_blueprint_class(BP_PATH)
    assert cls is not None

    assert levels.new_level_from_template(MAP_PATH, SOURCE_MAP)
    old = [a for a in actors.get_all_level_actors() if isinstance(a, unreal.StaticMeshActor)
           and a.static_mesh_component.static_mesh in [s['mesh'] for s in sources]]
    assert len(old)==11
    assembly = actors.spawn_actor_from_class(cls, unreal.Vector(0,0,0))
    assert assembly is not None
    assembly.set_actor_label('CamryAssembly')
    components = assembly.get_components_by_class(unreal.StaticMeshComponent)
    assert len(components)==11
    for s in sources:
        component = next(c for c in components if c.static_mesh==s['mesh'])
        assert close(xyz(component.get_world_location()), s['location']), (s['group'], component.get_name())
    assert actors.destroy_actors(old)

    # Verify hierarchy behavior, then restore the exact approved pose before saving.
    original = {c.get_name():xyz(c.get_world_location()) for c in components}
    group_components = {g: next(c for c in assembly.get_components_by_class(unreal.SceneComponent)
                                if c.get_name()==g) for g in ('LeftDoor','RightDoor')}
    for group, node in group_components.items():
        saved = unreal.Vector(*xyz(node.get_editor_property('relative_location')))
        node.set_relative_location(unreal.Vector(saved.x, saved.y+10, saved.z), False, True)
        for c in components:
            delta = 10 if c.get_attach_parent().get_name() == group else 0
            expected = original[c.get_name()].copy()
            expected[1] += delta
            assert close(xyz(c.get_world_location()), expected), 'Door isolation failed'
        node.set_relative_location(saved, False, True)
    assembly.set_actor_location(unreal.Vector(20,30,40), False, True)
    for c in components:
        assert close(xyz(c.get_world_location()), [a+b for a,b in zip(original[c.get_name()], [20,30,40])])
    assembly.set_actor_location(unreal.Vector(0,0,0), False, True)
    assert levels.save_current_level()
    assert levels.load_level(MAP_PATH)
    saved_assemblies = [a for a in actors.get_all_level_actors() if a.get_class()==cls]
    assert len(saved_assemblies)==1
    saved_components = saved_assemblies[0].get_components_by_class(unreal.StaticMeshComponent)
    assert len(saved_components)==11
    for c in saved_components:
        assert close(xyz(c.get_world_location()), original[c.get_name()])
    report_path.write_text(json.dumps({'success':True, 'blueprint':BP_PATH, 'map':MAP_PATH,
        'body_meshes':5,'left_door_meshes':3,'right_door_meshes':3,'door_pivots_cm':pivots,
        'checks':['initial placement preserved','left door moves alone','right door moves alone',
                  'whole vehicle translates together','saved map reload preserves pose'],
        'door_pivot_note':'Door geometry centers for positioning, not physical hinge axes',
        'viewport_render_verified':False},indent=2),encoding='utf-8')
    unreal.log('CAMRY_ASSEMBLY_SUCCESS')

try:
    main()
except Exception:
    report_path.write_text(json.dumps({'success':False,'error':traceback.format_exc()},indent=2),encoding='utf-8')
    raise
