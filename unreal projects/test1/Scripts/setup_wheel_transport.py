"""Install imported 187 geometry into the saved wheel cell. Run after import_traverse.py.

Scene adaptation, not a load-rated vehicle conveyor design. Original CAD remains
available in TraverseImport; only station copies are adapted here.
"""
import json
from pathlib import Path
import unreal

p = Path(unreal.Paths.project_dir()).resolve()
levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
api = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)

def group_actor(label):
    actor = api.spawn_actor_from_class(unreal.StaticMeshActor, unreal.Vector())
    actor.set_actor_label(label)
    actor.static_mesh_component.set_mobility(unreal.ComponentMobility.MOVABLE)
    actor.static_mesh_component.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
    actor.tags = [unreal.Name('WheelTransport187')]
    return actor

def attach(a, parent):
    a.attach_to_actor(parent, unreal.Name(''), unreal.AttachmentRule.KEEP_WORLD,
                      unreal.AttachmentRule.KEEP_WORLD, unreal.AttachmentRule.KEEP_WORLD, False)

def main():
    assert levels.load_level('/Game/DigitalTwin/Maps/TraverseImport')
    parts = []
    for a in api.get_all_level_actors():
        if not isinstance(a, unreal.StaticMeshActor) or not a.static_mesh_component.static_mesh:
            continue
        chain = []
        parent = a.get_attach_parent_actor()
        while parent:
            chain.append(parent.get_actor_label())
            parent = parent.get_attach_parent_actor()
        if any('Work-A' in name for name in chain):
            continue  # Supplied sample load is replaced by the Camry body.
        parts.append((a.get_actor_label(), a.static_mesh_component.static_mesh,
                      a.get_actor_transform(), chain))
    assert len(parts) > 20, 'Incomplete CAD geometry; import subassemblies first.'
    assert levels.load_level('/Game/DigitalTwin/Maps/WheelAssemblyCell')
    cell = next(a for a in api.get_all_level_actors() if a.get_class().get_name() == 'WheelAssemblyCell')
    # Save a station map, never an upstream runtime reset pose.
    for a in list(api.get_all_level_actors()):
        if unreal.Name('WheelTransport187') in a.tags:
            api.destroy_actor(a)
    fixed = group_actor('Traverse187_FixedRail')
    moving = group_actor('Traverse187_MovingCarriage')
    count = {'fixed': 0, 'moving': 0}
    for label, mesh, transform, chain in parts:
        actor = api.spawn_actor_from_class(unreal.StaticMeshActor, unreal.Vector())
        actor.set_actor_label('T187_' + label)
        actor.tags = [unreal.Name('WheelTransport187')]
        comp = actor.static_mesh_component
        comp.set_static_mesh(mesh)
        comp.set_mobility(unreal.ComponentMobility.MOVABLE)
        comp.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
        is_moving = any('SA05' in name for name in chain)
        slider = any('SE2RZ36-11202' in name or 'SE2RZ36-11203' in name for name in chain)
        is_moving = is_moving or slider
        offset = unreal.Vector()
        rail = any('SA02' in name for name in chain)
        if rail:
            offset = unreal.Vector(-62 if any('SA02_1' in name for name in chain) else -8, 0, 0)
        elif any('SA03' in name for name in chain):
            offset = unreal.Vector(-70, -10, 9.57)
        elif any('SA04' in name for name in chain):
            offset = unreal.Vector(-70, -20, 0)
        elif any('SA05' in name for name in chain):
            offset = unreal.Vector(-69, 60, 0)
        location = transform.translation
        if rail and not slider:
            # Extend the fixed CAD rail/drive along its travel axis for a 6m
            # demonstration stroke; the moving blocks retain their spacing.
            location = unreal.Vector(location.x, location.y * 1.35 + 9, location.z)
            scale = transform.scale3d
            transform.scale3d = unreal.Vector(scale.x, scale.y * 1.35, scale.z)
        transform.translation = location + offset
        actor.set_actor_transform(transform, False, True)
        attach(actor, moving if is_moving else fixed)
        count['moving' if is_moving else 'fixed'] += 1
    assert count['moving'] > 0
    # Original CAD travels along Y. Rotate this onto vehicle X and adapt its
    # width/length for the vehicle, leaving the original import map untouched.
    for root in (fixed, moving):
        root.set_actor_transform(unreal.Transform(location=unreal.Vector(480, 70, 2),
            rotation=unreal.Rotator(yaw=90), scale=unreal.Vector(2, 8, 1)), False, True)
    cell.set_editor_property('transport_carriage', moving)
    cell.set_editor_property('enable_transport', True)
    cell.set_editor_property('transport_start_offset_cm', unreal.Vector(-600, 0, 0))
    cell.set_editor_property('transport_seconds', 8.0)
    cell.set_editor_property('dock_settle_seconds', 1.0)
    assert levels.save_current_level()
    (p/'Saved/WheelTransportSetup.json').write_text(json.dumps(count, indent=2), encoding='utf-8')

if __name__ == '__main__':
    main()
