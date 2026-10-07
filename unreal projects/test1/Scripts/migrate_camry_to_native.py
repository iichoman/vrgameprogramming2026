"""Replace the saved assembly instance with native C++, preserving its pose/assets.

Run with UnrealEditor-Cmd after compiling test1Editor and closing the GUI editor.
Backups live outside Content. No Blueprint asset is needed by the resulting map.
"""
import json
import shutil
import traceback
from datetime import datetime
from pathlib import Path
import unreal

PROJECT = Path(unreal.Paths.project_dir()).resolve()
MAP = '/Game/DigitalTwin/Maps/CamryAssemblyPreview'
BP = '/Game/DigitalTwin/Blueprints/BP_CamryAssembly'
NATIVE = '/Script/test1.CamryAssembly'
REPORT = PROJECT / 'Saved/CamryNativeMigrationReport.json'


def vector(v):
    return [v.x, v.y, v.z]


def transform(t):
    q = t.rotation
    return vector(t.translation) + [q.x, q.y, q.z, q.w] + vector(t.scale3d)


def close(actual, expected, detail):
    assert len(actual) == len(expected)
    assert all(abs(a-b) < 0.001 for a, b in zip(actual, expected)), (detail, actual, expected)


def components(actor):
    return {c.get_name(): c for c in actor.get_components_by_class(unreal.SceneComponent)}


def snapshot(actor):
    result = {}
    for name, c in components(actor).items():
        row = {'world_transform': transform(c.get_world_transform()),
               'parent': c.get_attach_parent().get_name() if c.get_attach_parent() else None}
        if isinstance(c, unreal.StaticMeshComponent):
            assert c.static_mesh is not None, name
            row['mesh'] = c.static_mesh.get_path_name()
            row['materials'] = [c.get_material(i).get_path_name() if c.get_material(i) else None
                                for i in range(c.get_num_materials())]
        result[name] = row
    return result


def compare(actual, expected):
    assert set(actual) == set(expected), 'Component names differ'
    for name, row in expected.items():
        close(actual[name]['world_transform'], row['world_transform'], name)
        for key in ('parent', 'mesh', 'materials'):
            assert actual[name].get(key) == row.get(key), (name, key)


def verify_movement(actor):
    nodes = components(actor)
    original = snapshot(actor)
    for group in ('LeftDoor', 'RightDoor'):
        node = nodes[group]
        saved = vector(node.get_editor_property('relative_location'))
        before = {name: vector(c.get_world_location()) for name, c in nodes.items()}
        node.set_relative_location(unreal.Vector(saved[0], saved[1]+10, saved[2]), False, True)
        delta = [a-b for a, b in zip(vector(node.get_world_location()), before[group])]
        for name, c in nodes.items():
            moves = name == group or (c.get_attach_parent() and c.get_attach_parent().get_name() == group)
            expected = [a+b for a,b in zip(before[name], delta)] if moves else before[name]
            close(vector(c.get_world_location()), expected, 'Independent movement: '+name)
        node.set_relative_location(unreal.Vector(*saved), False, True)
        compare(snapshot(actor), original)
    saved = actor.get_actor_transform()
    location = saved.translation
    actor.set_actor_location(unreal.Vector(location.x+20, location.y+30, location.z+40), False, True)
    for name, c in nodes.items():
        close(vector(c.get_world_location()),
              [a+b for a,b in zip(original[name]['world_transform'][:3], [20,30,40])], name)
    actor.set_actor_transform(saved, False, True)
    compare(snapshot(actor), original)


def main():
    native = unreal.load_class(None, NATIVE)
    assert native is not None, 'Compile the native editor module first'
    levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    assert levels.load_level(MAP)
    old = [a for a in actors.get_all_level_actors()
           if a.get_class().get_path_name() in (BP+'.BP_CamryAssembly_C', NATIVE)]
    assert len(old) == 1, 'Expected exactly one assembly'
    old = old[0]
    expected = snapshot(old)
    assert len([r for r in expected.values() if 'mesh' in r]) == 11
    others = {a.get_path_name(): transform(a.get_actor_transform())
              for a in actors.get_all_level_actors() if a != old}

    backup = PROJECT / 'Saved/Backups' / ('CamryNative_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    backup.mkdir(parents=True)
    for relative in ('Maps/CamryAssemblyPreview.umap', 'Blueprints/BP_CamryAssembly.uasset'):
        source = PROJECT / 'Content/DigitalTwin' / relative
        if source.exists():
            shutil.copy2(source, backup / source.name)

    if old.get_class() == native:
        assembly = old
    else:
        assembly = actors.spawn_actor_from_class(native, old.get_actor_location(), old.get_actor_rotation())
        assert assembly is not None
        assembly.set_actor_transform(old.get_actor_transform(), False, True)
        assembly.set_actor_label(old.get_actor_label())
        assembly.set_folder_path(old.get_folder_path())
        assembly.set_editor_property('tags', old.get_editor_property('tags'))
        source_nodes, target_nodes = components(old), components(assembly)
        assert set(source_nodes) == set(target_nodes), 'Unexpected source hierarchy'
        # Preserve saved instance edits as well as the original approved defaults.
        for name, source in source_nodes.items():
            target = target_nodes[name]
            for prop in ('relative_location', 'relative_rotation', 'relative_scale3d', 'mobility'):
                target.set_editor_property(prop, source.get_editor_property(prop))
            target.set_editor_property('component_tags', source.get_editor_property('component_tags'))
            if isinstance(source, unreal.StaticMeshComponent):
                target.set_static_mesh(source.static_mesh)
                target.set_editor_property('override_materials', source.get_editor_property('override_materials'))
                target.set_collision_enabled(source.get_collision_enabled())
        compare(snapshot(assembly), expected)
        verify_movement(assembly)
        assert actors.destroy_actor(old)

    verify_movement(assembly)
    assert levels.save_current_level()
    assert levels.load_level(MAP)
    saved = [a for a in actors.get_all_level_actors() if a.get_class() == native]
    assert len(saved) == 1
    compare(snapshot(saved[0]), expected)
    assert not any(a.get_class().get_path_name().startswith(BP) for a in actors.get_all_level_actors())
    remaining = {a.get_path_name(): transform(a.get_actor_transform())
                 for a in actors.get_all_level_actors() if a != saved[0]}
    assert others.keys() == remaining.keys(), 'Other actors changed'
    for name in others:
        close(remaining[name], others[name], name)

    bp_removed = False
    referencers = []
    if unreal.EditorAssetLibrary.does_asset_exist(BP):
        referencers = list(unreal.EditorAssetLibrary.find_package_referencers_for_asset(BP, True))
        if not referencers:
            bp_removed = unreal.EditorAssetLibrary.delete_asset(BP)
            assert bp_removed, 'Could not remove the backed-up, unused Blueprint'
    REPORT.write_text(json.dumps({
        'success': True, 'native_class': NATIVE, 'map': MAP, 'backup': str(backup),
        'blueprint_removed': bp_removed, 'remaining_blueprint_referencers': referencers,
        'checks': ['all 11 mesh transforms, materials and hierarchy preserved',
                   'each door moves independently and restores', 'whole assembly translates',
                   'save/reload preserves native class and pose', 'other map actors unchanged'],
        'components': snapshot(saved[0]), 'viewport_render_verified': False,
    }, indent=2), encoding='utf-8')
    unreal.log('CAMRY_NATIVE_MIGRATION_SUCCESS')


try:
    main()
except Exception:
    REPORT.write_text(json.dumps({'success': False, 'error': traceback.format_exc()}, indent=2), encoding='utf-8')
    raise
