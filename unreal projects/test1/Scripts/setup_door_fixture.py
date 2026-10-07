"""Create/verify the native left-door fixture preview using the reviewed layout.

Run in a dedicated Unreal Python commandlet after compiling test1, or use
setup_door_fixture_live.py inside the existing editor after Live Coding succeeds.
Do not run a second editor/commandlet that writes a map open in another editor.
With -FixtureVerifyOnly, this script never creates actors or saves the map.
All temporary test changes are restored, including a displaced saved door pose.
This verifies contact geometry and frame ownership, not load capacity or seating.
"""
import json
import math
import shutil
import traceback
from datetime import datetime
from pathlib import Path

import unreal


PROJECT = Path(unreal.Paths.project_dir()).resolve()
MAP = '/Game/DigitalTwin/Maps/CamryAssemblyPreview'
REPORT = PROJECT / 'Saved/DoorFixtureReport.json'
FAILURE = PROJECT / 'Saved/DoorFixtureFailure.txt'
# Keep the analyzed source authoritative in the full workspace. A standalone
# copy of the Unreal project may carry the same data alongside this script.
ANALYZED_LAYOUT = PROJECT.parents[1] / 'car models/Camry_Converted/InnerDoorAnalysis/fixture_layout.json'
LAYOUT = ANALYZED_LAYOUT if ANALYZED_LAYOUT.is_file() else PROJECT / 'Scripts/fixture_layout.json'
VERIFY_ONLY = '-FixtureVerifyOnly' in unreal.SystemLibrary.get_command_line()
LIVE_EDITOR = bool(globals().get('FIXTURE_LIVE', False))
TOLERANCE_CM = 1.e-4
CONTACT_FIELDS = (
    'name', 'role', 'door_point_local_cm', 'outward_normal_local',
    'preview_size_mm', 'preview_stem_length_mm', 'region_only',
    'part_id', 'notes', 'offset_mm',
)


def xyz(value):
    return [float(value.x), float(value.y), float(value.z)]


def vector(values):
    assert len(values) == 3 and all(math.isfinite(float(x)) for x in values)
    return unreal.Vector(*values)


def transform_values(value):
    return xyz(value.translation) + [float(value.rotation.x), float(value.rotation.y),
        float(value.rotation.z), float(value.rotation.w)] + xyz(value.scale3d)


def independent_transform(value):
    # Unreal reflected struct wrappers can alias a live property. Reconstruct it.
    copied = unreal.Transform()
    copied.translation = vector(xyz(value.translation))
    copied.rotation = unreal.Quat(value.rotation.x, value.rotation.y,
                                  value.rotation.z, value.rotation.w)
    copied.scale3d = vector(xyz(value.scale3d))
    return copied


def close(actual, expected, context, tolerance=TOLERANCE_CM):
    assert len(actual) == len(expected)
    assert all(math.isfinite(float(a)) and abs(a-b) <= tolerance
               for a, b in zip(actual, expected)), (context, actual, expected)


def scene(actor):
    return {c.get_name(): c for c in actor.get_components_by_class(unreal.SceneComponent)}


def snapshot(actor):
    return {name: transform_values(component.get_world_transform())
            for name, component in scene(actor).items()}


def assert_snapshot(actor, expected):
    actual = snapshot(actor)
    assert actual.keys() == expected.keys(), 'Unexpected assembly component changes'
    for name in expected:
        close(actual[name], expected[name], 'Restored ' + name)


def clone_contact(source):
    result = unreal.DoorFixtureContact()
    for field in CONTACT_FIELDS:
        value = source.get_editor_property(field)
        if field in ('door_point_local_cm', 'outward_normal_local',
                     'preview_size_mm', 'offset_mm'):
            value = vector(xyz(value))
        result.set_editor_property(field, value)
    return result


def load_contacts():
    document = json.loads(LAYOUT.read_text(encoding='utf-8-sig'))
    contacts = []
    roles = {name: getattr(unreal.DoorFixtureRole, name.upper())
             for name in ('Locator', 'Support', 'Stopper', 'Clamp')}
    for source in document['contacts']:
        contact = unreal.DoorFixtureContact()
        contact.set_editor_property('name', unreal.Name(source['name']))
        contact.set_editor_property('role', roles[source['role']])
        for field in ('door_point_local_cm', 'outward_normal_local', 'preview_size_mm'):
            contact.set_editor_property(field, vector(source[field]))
        contact.set_editor_property('preview_stem_length_mm', float(source['preview_stem_length_mm']))
        contact.set_editor_property('region_only', bool(source['region_only']))
        contact.set_editor_property('part_id', int(source['part_id']))
        contact.set_editor_property('notes', str(source['notes']))
        offset = source.get('offset_mm', [0, 0, 0])
        if offset == 0:
            offset = [0, 0, 0]
        contact.set_editor_property('offset_mm', vector(offset))
        contacts.append(contact)
    assert 0 < len(contacts) <= 12
    assert len({str(p.get_editor_property('name')) for p in contacts}) == len(contacts)
    return document, contacts


def serialized_contacts(contacts):
    output = []
    for contact in contacts:
        row = {}
        for field in CONTACT_FIELDS:
            value = contact.get_editor_property(field)
            if field in ('door_point_local_cm', 'outward_normal_local',
                         'preview_size_mm', 'offset_mm'):
                value = xyz(value)
            elif field in ('name', 'role'):
                value = str(value)
            row[field] = value
        output.append(row)
    return output


def fixture_results(fixture):
    assert fixture.call_method('RefreshFixture'), fixture.get_editor_property('last_status')
    assert fixture.get_editor_property('layout_valid')
    result = {}
    for entry in fixture.get_editor_property('results'):
        name = str(entry.get_editor_property('name'))
        assert name not in result
        result[name] = {
            'fixture_world_cm': xyz(entry.get_editor_property('fixture_world_cm')),
            'door_world_cm': xyz(entry.get_editor_property('door_world_cm')),
            'normal_displacement_mm': float(entry.get_editor_property('normal_displacement_mm')),
            'tangential_displacement_mm': float(entry.get_editor_property('tangential_displacement_mm')),
        }
    return result


def normalized(values):
    length = math.sqrt(sum(x*x for x in values))
    assert length > 1.e-8
    return [x/length for x in values]


def assert_geometry(fixture, body, reference, door, contacts, retraction_mm):
    """Independently evaluate reference/body composition and signed point residuals."""
    actual = fixture_results(fixture)
    assert len(actual) == len(contacts)
    body_world = body.get_world_transform()
    door_world = door.get_world_transform()
    for contact in contacts:
        name = str(contact.get_editor_property('name'))
        local = xyz(contact.get_editor_property('door_point_local_cm'))
        normal = normalized(xyz(contact.get_editor_property('outward_normal_local')))
        offset = xyz(contact.get_editor_property('offset_mm'))
        anchor = vector([p + o*.1 + n*retraction_mm*.1
                         for p, o, n in zip(local, offset, normal)])
        expected_fixture = body_world.transform_location(reference.transform_location(anchor))
        expected_door = door_world.transform_location(vector(local))
        world_normal = normalized(xyz(body_world.transform_direction(
            reference.transform_direction(vector(normal)))))
        residual_mm = [10*(d-f) for d, f in zip(xyz(expected_door), xyz(expected_fixture))]
        signed_normal = sum(d*n for d, n in zip(residual_mm, world_normal))
        tangent = math.sqrt(sum((d-signed_normal*n)**2 for d, n in zip(residual_mm, world_normal)))
        row = actual[name]
        close(row['fixture_world_cm'], xyz(expected_fixture), name + ' reference-frame anchor')
        close(row['door_world_cm'], xyz(expected_door), name + ' current door sample')
        close([row['normal_displacement_mm']], [signed_normal], name + ' signed residual')
        close([row['tangential_displacement_mm']], [tangent], name + ' tangent residual')
        close(xyz(fixture.call_method('GetContactWorldPosition', (unreal.Name(name),))),
              xyz(expected_fixture), name + ' contact world API')
    return actual


def main():
    levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    if LIVE_EDITOR:
        assert not levels.is_in_play_in_editor(), 'Stop Play before configuring the fixture'
        editor = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
        world = editor.get_editor_world()
        current_map = world.get_path_name().split('.', 1)[0] if world else None
        assert current_map == MAP, ('Open CamryAssemblyPreview first; live setup never switches maps', current_map)
    else:
        assert levels.load_level(MAP), 'Existing preview map could not be loaded'
    assembly_class = unreal.load_class(None, '/Script/test1.CamryAssembly')
    fixture_class = unreal.load_class(None, '/Script/test1.DoorFixture')
    assert assembly_class and fixture_class, 'Build the native classes before running setup'
    all_actors = list(actors.get_all_level_actors())
    assemblies = [a for a in all_actors if a.get_class() == assembly_class]
    fixtures = [a for a in all_actors if a.get_class() == fixture_class]
    assert len(assemblies) == 1, 'Expected exactly one CamryAssembly'
    assert len(fixtures) <= 1, 'Multiple fixtures require explicit manual selection'
    assembly = assemblies[0]
    components = scene(assembly)
    body, door = components['Body'], components['LeftDoor']
    pose = next(c for c in assembly.get_components_by_class(unreal.DoorPoseComponent)
                if c.get_name() == 'LeftDoorPose')
    assert pose.call_method('HasReferencePose'), 'A saved left-door reference is required'
    backup = None
    if not VERIFY_ONLY:
        # Exactly one backup, before actor creation, property changes, or test mutations.
        backup = PROJECT/'Saved/Backups'/('DoorFixture_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        backup.mkdir(parents=True)
        shutil.copy2(PROJECT/'Content/DigitalTwin/Maps/CamryAssemblyPreview.umap',
                     backup/'CamryAssemblyPreview.umap')
    # Seed transient inspection label transforms before preserving the scene.
    assembly.call_method('RefreshDoorInspection')
    original_assembly = independent_transform(assembly.get_actor_transform())
    original_door = independent_transform(door.get_world_transform())
    reference = independent_transform(pose.get_editor_property('reference_pose_in_body'))
    original_scene = snapshot(assembly)
    original_others = {a.get_path_name(): (a, transform_values(a.get_actor_transform()))
                       for a in all_actors if a != assembly and a not in fixtures}
    assert fixtures or not VERIFY_ONLY, 'Verify-only requires an existing saved fixture'
    created = not fixtures
    fixture = fixtures[0] if fixtures else actors.spawn_actor_from_class(fixture_class, unreal.Vector())
    assert fixture
    if created:
        fixture.set_actor_label('DoorFixture_Left')
    current_contacts = fixture.get_editor_property('contacts')
    needs_initialization = created or not current_contacts
    if needs_initialization:
        assert not VERIFY_ONLY, 'Verify-only cannot initialize an empty layout'
        document, contacts = load_contacts()
        fixture.set_editor_property('assembly', assembly)
        fixture.set_editor_property('contacts', contacts)
        fixture.set_editor_property('layout_source', str(document.get('layout_source', LAYOUT.name)))
        assert fixture.call_method('AlignToDoorReference'), fixture.get_editor_property('last_status')
    else:
        assert fixture.get_editor_property('assembly') == assembly, 'Existing fixture targets a different assembly'
    contacts = [clone_contact(c) for c in fixture.get_editor_property('contacts')]
    original_fixture = independent_transform(fixture.get_actor_transform())
    fixture_root = fixture.get_editor_property('root_component')
    original_parent = fixture_root.get_attach_parent()
    original_relative = independent_transform(fixture_root.get_relative_transform())
    original_retraction = fixture.get_editor_property('preview_retraction_mm')
    original_show = fixture.get_editor_property('show_fixture')
    original_labels = fixture.get_editor_property('show_labels')
    checks = []
    try:
        assert fixture.call_method('AlignToDoorReference')
        assert fixture_root.get_attach_parent() == body
        close(transform_values(fixture_root.get_relative_transform()), transform_values(reference),
              'Fixture attaches with the exact saved reference transform')
        fixture.set_editor_property('preview_retraction_mm', 0.0)
        assert pose.call_method('RestoreReferencePose')
        baseline = assert_geometry(fixture, body, reference, door, contacts, 0.0)
        before_fixture = transform_values(fixture.get_actor_transform())
        count = len(fixture.get_components_by_class(unreal.ActorComponent))
        for repeat in range(3):
            assert fixture.call_method('RefreshFixture')
            assert len(fixture.get_components_by_class(unreal.ActorComponent)) == count
        checks.append('Saved reference composition, stable component count and repeatable refresh')

        for offset in (unreal.Vector(1, -2, 3), unreal.Vector(1, -2, 3)):
            assert pose.call_method('ApplyReferenceOffsetMm', (offset,))
            actual = assert_geometry(fixture, body, reference, door, contacts, 0.0)
            close(transform_values(fixture.get_actor_transform()), before_fixture, 'Door motion cannot drag fixture')
            for name, row in actual.items():
                close(row['fixture_world_cm'], baseline[name]['fixture_world_cm'], name + ' fixed anchor')
                assert sum((a-b)**2 for a, b in zip(row['door_world_cm'], baseline[name]['door_world_cm'])) > .01
        checks.append('Nonaccumulating door offsets update residuals without moving fixture anchors')

        # Parent motion must propagate while the fixture/reference transform stays fixed.
        assembly.set_actor_rotation(unreal.Rotator(pitch=0, yaw=90, roll=0), True)
        assembly.set_actor_location(unreal.Vector(200, 300, 100), False, True)
        assert pose.call_method('ApplyReferenceOffsetMm', (unreal.Vector(1, -2, 3),))
        assert_geometry(fixture, body, reference, door, contacts, 0.0)
        close(transform_values(fixture_root.get_relative_transform()), transform_values(reference),
              'Parent motion preserves fixture-local reference')
        checks.append('Body translation and rotation preserve local fixture geometry')

        fixture.set_editor_property('preview_retraction_mm', 12.5)
        assert_geometry(fixture, body, reference, door, contacts, 12.5)
        checks.append('Preview retraction moves each fixture anchor outward by the specified millimeters')

        # Display flags must never change the physical contact residuals.
        expected = fixture_results(fixture)
        for show, labels in ((False, False), (True, False), (True, True)):
            fixture.set_editor_property('show_fixture', show)
            fixture.set_editor_property('show_labels', labels)
            assert fixture_results(fixture) == expected
            for label in fixture.get_components_by_class(unreal.TextRenderComponent):
                index = int(label.get_name().removeprefix('FixtureLabel')) - 1
                assert bool(label.get_editor_property('visible')) == (show and labels and index < len(contacts))
            for mesh in fixture.get_components_by_class(unreal.StaticMeshComponent):
                is_stem = mesh.get_name().startswith('FixtureStem')
                prefix = 'FixtureStem' if is_stem else 'FixturePad'
                index = int(mesh.get_name().removeprefix(prefix)) - 1
                expected_visible = show and index < len(contacts)
                if expected_visible and is_stem:
                    expected_visible = contacts[index].get_editor_property('preview_stem_length_mm') > 0
                assert bool(mesh.get_editor_property('visible')) == expected_visible
            for arrow in fixture.get_components_by_class(unreal.ArrowComponent):
                index = int(arrow.get_name().removeprefix('FixtureDirection')) - 1
                assert bool(arrow.get_editor_property('visible')) == (show and index < len(contacts))
        checks.append('Geometry and label visibility toggle independently of contact calculations')

        bad = clone_contact(contacts[0])
        bad.set_editor_property('outward_normal_local', unreal.Vector())
        fixture.set_editor_property('contacts', [bad])
        assert not fixture.call_method('RefreshFixture')
        assert not fixture.get_editor_property('layout_valid')
        assert not fixture.get_editor_property('results'), 'Invalid normals must clear stale results'
        fixture.set_editor_property('contacts', [clone_contact(c) for c in contacts])
        assert_geometry(fixture, body, reference, door, contacts, 12.5)
        checks.append('Invalid contact normal rejects layout, clears stale results, and recovers')
    finally:
        # Never replace a saved displaced door pose with the normal reference pose.
        assembly.set_actor_transform(original_assembly, False, True)
        assert pose.call_method('SetDoorWorldPose', (original_door,))
        fixture.set_editor_property('contacts', [clone_contact(c) for c in contacts])
        fixture.set_editor_property('preview_retraction_mm', original_retraction)
        fixture.set_editor_property('show_fixture', original_show)
        fixture.set_editor_property('show_labels', original_labels)
        if original_parent:
            keep = unreal.AttachmentRule.KEEP_WORLD
            assert fixture_root.attach_to_component(original_parent, unreal.Name('None'), keep, keep, keep, False)
            fixture_root.set_relative_transform(original_relative, False, True)
        else:
            keep = unreal.DetachmentRule.KEEP_WORLD
            fixture.detach_from_actor(keep, keep, keep)
            fixture.set_actor_transform(original_fixture, False, True)
        fixture.call_method('RefreshFixture')

    assert_snapshot(assembly, original_scene)
    close(transform_values(pose.get_editor_property('reference_pose_in_body')), transform_values(reference),
          'Saved reference is never captured or overwritten')
    close(transform_values(fixture.get_actor_transform()), transform_values(original_fixture),
          'Original fixture world transform restored')
    close(transform_values(fixture_root.get_relative_transform()), transform_values(original_relative),
          'Original fixture relative transform restored')
    for actor, saved in original_others.values():
        close(transform_values(actor.get_actor_transform()), saved, actor.get_name() + ' untouched')
    checks.append('Original door pose, fixture settings, other actors and reference restored')
    saved_fixture = transform_values(fixture.get_actor_transform())
    saved_contacts = serialized_contacts(fixture.get_editor_property('contacts'))
    saved_results = fixture_results(fixture)
    if not VERIFY_ONLY:
        if LIVE_EDITOR:
            current_map = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world().get_path_name().split('.', 1)[0]
            assert current_map == MAP, 'Refusing to save a different current map'
        assert levels.save_current_level()
    if not VERIFY_ONLY and not LIVE_EDITOR:
        assert levels.load_level(MAP)
        loaded = list(actors.get_all_level_actors())
        fixture = next(a for a in loaded if a.get_class() == fixture_class)
        assembly = next(a for a in loaded if a.get_class() == assembly_class)
        assert_snapshot(assembly, original_scene)
        close(transform_values(fixture.get_actor_transform()), saved_fixture, 'Serialized fixture transform')
        assert serialized_contacts(fixture.get_editor_property('contacts')) == saved_contacts
        roundtrip_results = fixture_results(fixture)
        assert roundtrip_results.keys() == saved_results.keys()
        for name in saved_results:
            for field in ('fixture_world_cm', 'door_world_cm'):
                close(roundtrip_results[name][field], saved_results[name][field], 'Round trip ' + name + ' ' + field)
        checks.append('Saved map round trip preserves contacts, door pose and fixture positions')
    elif LIVE_EDITOR:
        checks.append('Existing editor world preserved without loading or reloading a map')
    report = {
        'success': True, 'map': MAP, 'created_fixture': created,
        'initialized_contacts': needs_initialization, 'verify_only': VERIFY_ONLY,
        'live_editor': LIVE_EDITOR, 'cold_load_verified': VERIFY_ONLY and not LIVE_EDITOR,
        'map_roundtrip_verified': not VERIFY_ONLY and not LIVE_EDITOR,
        'backup': str(backup) if backup else None,
        'backup_scope': 'Previous on-disk map; live unsaved edits stay in memory and are saved only after successful checks.',
        'checks': checks, 'contacts': saved_contacts, 'results': saved_results,
        'saved_assembly_scene': original_scene, 'saved_fixture_transform': saved_fixture,
        'viewport_manually_verified': False,
        'scope': 'Geometric fixture preview only; no seating solver, force model, or engineering contact approval.',
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    unreal.log('DOOR_FIXTURE_LIVE_SUCCESS' if LIVE_EDITOR else
               ('DOOR_FIXTURE_SUCCESS' if not VERIFY_ONLY else 'DOOR_FIXTURE_COLD_LOAD_SUCCESS'))


try:
    main()
except Exception:
    FAILURE.parent.mkdir(parents=True, exist_ok=True)
    failure = traceback.format_exc()
    FAILURE.write_text(failure, encoding='utf-8')
    REPORT.write_text(json.dumps({'success': False, 'verify_only': VERIFY_ONLY, 'live_editor': LIVE_EDITOR,
        'failure_path': str(FAILURE), 'error': failure}, ensure_ascii=False, indent=2), encoding='utf-8')
    raise
