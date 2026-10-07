"""Capture the saved assembly pose once, exercise the native API, and save references.
Run with -CamryPoseVerifyOnly in a second process to check serialization without saving.
"""
import json
import shutil
import traceback
from datetime import datetime
from pathlib import Path
import unreal

PROJECT = Path(unreal.Paths.project_dir()).resolve()
MAP = '/Game/DigitalTwin/Maps/CamryAssemblyPreview'
REPORT = PROJECT / 'Saved/DoorPoseReport.json'
VERIFY_ONLY = '-CamryPoseVerifyOnly' in unreal.SystemLibrary.get_command_line()


def xyz(v):
    return [v.x, v.y, v.z]


def pose(t):
    return xyz(t.translation) + [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w] + xyz(t.scale3d)


def close(a, b, message):
    assert all(abs(x-y) < 1.e-5 for x,y in zip(a,b)), (message, a, b)


def scene(actor):
    return {c.get_name(): c for c in actor.get_components_by_class(unreal.SceneComponent)}


def snapshot(actor):
    return {name: pose(c.get_world_transform()) for name,c in scene(actor).items()}


def check_snapshot(actor, expected):
    actual = snapshot(actor)
    assert actual.keys() == expected.keys()
    for name in expected:
        close(actual[name], expected[name], name)


def controllers(actor):
    return {c.get_name(): c for c in actor.get_components_by_class(unreal.DoorPoseComponent)}


def main():
    levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    assert levels.load_level(MAP)
    cls = unreal.load_class(None, '/Script/test1.CamryAssembly')
    assemblies = [a for a in actors.get_all_level_actors() if a.get_class() == cls]
    assert len(assemblies) == 1
    assembly = assemblies[0]
    original = snapshot(assembly)
    controls = controllers(assembly)
    assert set(controls) == {'LeftDoorPose', 'RightDoorPose'}

    if VERIFY_ONLY:
        report = json.loads(REPORT.read_text(encoding='utf-8'))
        assert report['success']
        check_snapshot(assembly, report['saved_scene'])
        for control in controls.values():
            assert control.call_method("HasReferencePose")
            assert control.call_method("ApplyReferenceOffsetMm", (unreal.Vector(1, -2, 3),))
            assert control.call_method("RestoreReferencePose")
        check_snapshot(assembly, report['saved_scene'])
        report['cold_load_verified'] = True
        REPORT.write_text(json.dumps(report, indent=2), encoding='utf-8')
        unreal.log('CAMRY_DOOR_POSE_COLD_LOAD_SUCCESS')
        return

    backup = PROJECT / 'Saved/Backups' / ('DoorPose_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    backup.mkdir(parents=True)
    shutil.copy2(PROJECT/'Content/DigitalTwin/Maps/CamryAssemblyPreview.umap', backup/'CamryAssemblyPreview.umap')
    selected = assembly.get_editor_property('selected_door')
    preview = assembly.get_editor_property('preview_offset_mm')
    checks = []
    for side, group in ((unreal.CamryDoorSide.LEFT, 'LeftDoor'), (unreal.CamryDoorSide.RIGHT, 'RightDoor')):
        control = controls[group+'Pose']
        assembly.set_editor_property('selected_door', side)
        if not control.call_method("HasReferencePose"):
            assembly.call_method("CaptureSelectedDoorReference")
        assert control.call_method("HasReferencePose")
        # Do not silently replace a previously saved reference with a displaced pose.
        assert control.call_method("RestoreReferencePose")
        check_snapshot(assembly, original)
        saved_reference = pose(control.get_editor_property('reference_pose_in_body'))
        offset = unreal.Vector(10, -20, 30)  # 1, -2, 3 centimeters in body axes.
        assembly.set_editor_property('preview_offset_mm', offset)
        for repeat in range(2):
            assembly.call_method("ApplySelectedDoorOffset")
            world_delta = scene(assembly)['Body'].get_world_transform().transform_direction(unreal.Vector(1,-2,3))
            for name, component in scene(assembly).items():
                expected = original[name].copy()
                if name == group or (component.get_attach_parent() and component.get_attach_parent().get_name() == group):
                    expected[:3] = [a+b for a,b in zip(expected[:3], xyz(world_delta))]
                close(pose(component.get_world_transform()), expected, group+' offset/isolation')
        assert not control.call_method("ApplyReferenceOffsetMm", (unreal.Vector(float('nan'), 0, 0),))
        assembly.call_method("RestoreSelectedDoorReference")
        check_snapshot(assembly, original)

        # A future carrier may command world position AND orientation independently of the baseline.
        door = scene(assembly)[group]
        desired = door.get_world_transform()
        desired.translation = unreal.Vector(30, 40, 50)
        desired.rotation = unreal.Rotator(pitch=10, yaw=25, roll=-5).quaternion()
        assert control.call_method("SetDoorWorldPose", (desired,))
        close(pose(door.get_world_transform()), pose(desired), 'External world command')
        close(pose(control.get_editor_property('reference_pose_in_body')), saved_reference, 'Reference unchanged')
        assert control.call_method("RestoreReferencePose")
        check_snapshot(assembly, original)
        checks.append(group+': mm conversion, repeat without accumulation, isolation, rotation restore, external command')

    saved_actor = assembly.get_actor_transform()
    assembly.set_actor_location(unreal.Vector(123,234,345), False, True)
    assembly.set_actor_rotation(unreal.Rotator(pitch=0,yaw=90,roll=0), True)
    moved = snapshot(assembly)
    left = scene(assembly)['LeftDoor']
    assert controls['LeftDoorPose'].call_method("ApplyReferenceOffsetMm", (unreal.Vector(10,0,0),))
    expected = moved['LeftDoor'][:3].copy()
    expected[1] += 1  # Body X becomes world Y at +90-degree yaw.
    close(xyz(left.get_world_location()), expected, 'Rotated body frame')
    assert controls['LeftDoorPose'].call_method("RestoreReferencePose")
    check_snapshot(assembly, moved)
    assembly.set_actor_transform(saved_actor, False, True)
    check_snapshot(assembly, original)
    checks.append('Body translation/90-degree rotation preserves reference and offset axes')

    # Exercise the interface with another attachment parent, without implementing robot transport.
    carrier = actors.spawn_actor_from_class(unreal.StaticMeshActor, unreal.Vector(200,300,400))
    parent = carrier.static_mesh_component
    parent.set_editor_property('mobility', unreal.ComponentMobility.MOVABLE)
    carrier.set_actor_rotation(unreal.Rotator(pitch=0,yaw=35,roll=0), True)
    keep = unreal.AttachmentRule.KEEP_WORLD
    original_parent = left.get_attach_parent()
    assert left.attach_to_component(parent, unreal.Name('None'), keep, keep, keep, False)
    carrier.set_actor_location(unreal.Vector(250,350,450), False, True)
    assert controls['LeftDoorPose'].call_method("RestoreReferencePose")
    close(pose(left.get_world_transform()), original['LeftDoor'], 'Restore while parented to carrier')
    assert left.get_attach_parent() == parent, 'Controller must not take over attachment ownership'
    assert left.attach_to_component(original_parent, unreal.Name('None'), keep, keep, keep, False)
    assert actors.destroy_actor(carrier)
    check_snapshot(assembly, original)
    checks.append('External carrier parent supported; restore uses body frame and does not reparent')

    fresh = actors.spawn_actor_from_class(cls, unreal.Vector(0,0,0))
    fresh_before = snapshot(fresh)
    fresh_control = controllers(fresh)['LeftDoorPose']
    assert not fresh_control.call_method("HasReferencePose")
    assert not fresh_control.call_method("RestoreReferencePose")
    check_snapshot(fresh, fresh_before)
    fresh.set_actor_scale3d(unreal.Vector(2,2,2))
    assert not fresh_control.call_method("CaptureReferencePose")
    assert actors.destroy_actor(fresh)
    checks.append('Missing baseline, nonfinite offset and scaled frames reject commands')

    assembly.set_editor_property('selected_door', selected)
    assembly.set_editor_property('preview_offset_mm', preview)
    check_snapshot(assembly, original)
    assert levels.save_current_level()
    assert levels.load_level(MAP)
    saved = next(a for a in actors.get_all_level_actors() if a.get_class() == cls)
    check_snapshot(saved, original)
    assert all(c.call_method("HasReferencePose") for c in controllers(saved).values())
    REPORT.write_text(json.dumps({'success':True, 'map':MAP, 'backup':str(backup),
        'checks': checks, 'saved_scene':snapshot(saved), 'cold_load_verified':False,
        'viewport_buttons_manually_verified':False}, indent=2), encoding='utf-8')
    unreal.log('CAMRY_DOOR_POSE_SUCCESS')


try:
    main()
except Exception:
    (PROJECT/'Saved/DoorPoseFailure.txt').write_text(traceback.format_exc(), encoding='utf-8')
    raise
