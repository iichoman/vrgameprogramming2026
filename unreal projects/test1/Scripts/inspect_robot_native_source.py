"""Read-only source inspection for the native door assembly cell.

Loads the existing preview and spawns temporary robot actors; never saves assets.
Run in UnrealEditor-Cmd with PythonScriptPlugin and EditorScriptingUtilities enabled.
"""
import json
import traceback
from pathlib import Path
import unreal

PROJECT = Path(unreal.Paths.project_dir()).resolve()
REPORT = PROJECT / 'Saved/RobotSourceInspection.json'


def xyz(v):
    return [float(v.x), float(v.y), float(v.z)]


def transform(t):
    q = t.rotation
    r = q.rotator()
    return {'translation_cm': xyz(t.translation), 'quaternion_xyzw': [q.x, q.y, q.z, q.w],
            'rotation_pyr_deg': [r.pitch, r.yaw, r.roll], 'scale': xyz(t.scale3d)}


def simple(v):
    if isinstance(v, (str, int, float, bool)) or v is None:
        return v
    if isinstance(v, unreal.Vector):
        return xyz(v)
    if isinstance(v, unreal.Rotator):
        return [v.pitch, v.yaw, v.roll]
    if isinstance(v, unreal.Transform):
        return transform(v)
    if isinstance(v, unreal.DoorFixtureContact):
        return {name: simple(v.get_editor_property(name)) for name in (
            'name', 'role', 'part_id', 'door_point_local_cm', 'outward_normal_local',
            'preview_size_mm', 'preview_stem_length_mm', 'offset_mm', 'region_only', 'notes')}
    try:
        return [simple(x) for x in v]
    except Exception:
        return str(v)


def component(c):
    parent = c.get_attach_parent()
    result = {'name': c.get_name(), 'class': c.get_class().get_path_name(),
              'parent': parent.get_name() if parent else None,
              'world': transform(c.get_world_transform()),
              'relative_location_cm': xyz(c.get_editor_property('relative_location')),
              'relative_rotation_pyr_deg': simple(c.get_editor_property('relative_rotation')),
              'relative_scale': xyz(c.get_editor_property('relative_scale3d')),
              'mobility': str(c.mobility)}
    if isinstance(c, unreal.StaticMeshComponent):
        mesh = c.get_editor_property('static_mesh')
        result['mesh'] = mesh.get_path_name() if mesh else None
        result['bounds_world'] = simple(c.get_component_bounds()) if hasattr(c, 'get_component_bounds') else None
        if mesh:
            box = mesh.get_bounding_box()
            result['mesh_bounds_local_cm'] = {'min': xyz(box.min), 'max': xyz(box.max)}
    return result


def actor_snapshot(a):
    return {'name': a.get_name(), 'label': a.get_actor_label(), 'class': a.get_class().get_path_name(),
            'actor_transform': transform(a.get_actor_transform()),
            'components': [component(c) for c in a.get_components_by_class(unreal.SceneComponent)]}


def main():
    out = {'success': False, 'errors': [], 'robots': [], 'preview': []}
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    assert levels.load_level('/Game/DigitalTwin/Maps/CamryAssemblyPreview')
    for a in actors.get_all_level_actors():
        if any(s in a.get_class().get_path_name() for s in ('CamryAssembly', 'DoorFixture')):
            entry = actor_snapshot(a)
            entry['related_properties'] = {}
            for name in ('contacts', 'assembly', 'layout_source', 'preview_retraction_mm'):
                try:
                    entry['related_properties'][name] = simple(a.get_editor_property(name))
                except Exception:
                    pass
            for c in a.get_components_by_class(unreal.ActorComponent):
                if 'DoorPose' in c.get_class().get_path_name():
                    entry.setdefault('door_pose', []).append({
                        'name': c.get_name(), 'reference_pose_in_body': transform(c.get_editor_property('reference_pose_in_body')),
                        'has_reference': c.call_method('HasReferencePose')})
            out['preview'].append(entry)
    REPORT.write_text(json.dumps(out, indent=2), encoding='utf-8')
    for path in ('/Game/Robot/Blueprints/BP_KUKA_Robot', '/Game/Robot/Blueprints/BP_KUKA_Robot_InputFixed'):
        a = None
        try:
            cls = unreal.EditorAssetLibrary.load_blueprint_class(path)
            if not cls:
                raise RuntimeError('Blueprint failed to load: ' + path)
            a = actors.spawn_actor_from_class(cls, unreal.Vector(0,0,0))
            entry = actor_snapshot(a)
            entry['asset'] = path
            entry['python_names'] = [n for n in dir(a) if any(s in n.lower() for s in ('joint', 'angle', 'axis', 'tool', 'tcp', 'home', 'a1', 'a2', 'a3', 'a4', 'a5', 'a6'))]
            entry['properties'] = {}
            for name in entry['python_names'] + ['JointAngles', 'JointAnglesDeg', 'SelectedAxis', 'JointLimits', 'JointMinAngles', 'JointMaxAngles', 'A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'Tool', 'ReviewLimitDeg', 'JogSpeedDegPerSec'] + ['A%d_Deg' % i for i in range(1,7)]:
                try:
                    value = a.get_editor_property(name)
                    entry['properties'][name] = simple(value)
                except Exception:
                    pass
            entry['joint_probes'] = []
            for index in range(0, 7):
                probe = {'requested_index': index, 'requested_degrees': 10}
                try:
                    a.call_method('SetJointAngle', (index, 10.0))
                    probe['after'] = [component(c) for c in a.get_components_by_class(unreal.SceneComponent)]
                    a.call_method('SetJointAngle', (index, 0.0))
                except Exception as e:
                    probe['error'] = str(e)
                entry['joint_probes'].append(probe)
            out['robots'].append(entry)
        except Exception:
            out['errors'].append({'asset': path, 'traceback': traceback.format_exc()})
        finally:
            if a:
                actors.destroy_actor(a)
            REPORT.write_text(json.dumps(out, indent=2), encoding='utf-8')
    out['success'] = bool(out['preview']) and len(out['robots']) == 2
    REPORT.write_text(json.dumps(out, indent=2), encoding='utf-8')
    unreal.log('ROBOT_SOURCE_INSPECTION_COMPLETE ' + str(REPORT))


main()
