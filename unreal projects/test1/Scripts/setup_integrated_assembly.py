"""Create a separate line map; retain source maps and the 42-module inventory."""
import json
import sys
import traceback
from pathlib import Path
import unreal

P = Path(unreal.Paths.project_dir()).resolve()
sys.path.insert(0, str(P/'Scripts'))
import setup_door_assembly_cell as door
import setup_wheel_assembly_cell as wheel
import verify_door_assembly_cell as verify_door
wheel.door = door
MAP = '/Game/DigitalTwin/Maps/IntegratedAssemblyLine'
levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
report = {'success': False, 'map': MAP}

def attach(a, parent):
    a.attach_to_actor(parent, unreal.Name('None'), unreal.AttachmentRule.KEEP_WORLD,
                      unreal.AttachmentRule.KEEP_WORLD, unreal.AttachmentRule.KEEP_WORLD, False)

try:
    assert not unreal.EditorAssetLibrary.does_asset_exist(MAP), 'New map already exists; do not overwrite user edits.'
    assert levels.new_level_from_template(MAP, '/Game/DigitalTwin/Maps/WheelAssemblyCell')
    all_actors = actors.get_all_level_actors()
    cell = next(a for a in all_actors if isinstance(a, unreal.WheelAssemblyCell))
    car = cell.get_editor_property('assembly')
    carriage = cell.get_editor_property('transport_carriage')
    car.set_actor_transform(unreal.Transform(), False, True)
    cell.set_editor_property('externally_driven', True)
    cell.set_editor_property('enable_transport', False)
    comps = {c.get_name(): c for c in car.get_components_by_class(unreal.SceneComponent)}
    pose = next(c for c in car.get_components_by_class(unreal.DoorPoseComponent) if c.get_name() == 'LeftDoorPose')
    assert pose.call_method('RestoreReferencePose')
    for c in car.get_components_by_class(unreal.StaticMeshComponent):
        c.set_static_mesh(None)  # Keep the calibrated frames, replace all old geometry.
    for c in cell.get_components_by_class(unreal.StaticMeshComponent):
        c.set_static_mesh(None)
    car.get_component_by_class(unreal.DoorInspectionComponent).set_editor_property('show_inspection', False)

    manifest = json.loads((P/'Scripts/process_vehicle_manifest.json').read_text(encoding='utf-8'))
    entries = {m['id']: m for m in manifest['modules']}
    wheel_ids = ['Wheel_FL', 'Wheel_RL', 'Wheel_FR', 'Wheel_RR']
    centers = [unreal.Vector(250 + entries[k]['pivot_ue_cm'][0], *entries[k]['pivot_ue_cm'][1:]) for k in wheel_ids]
    cell.set_editor_property('mount_centers_cm', centers)
    report['wheel_ik'] = [wheel.preflight(cell.get_editor_property('left_robot' if side == 0 else 'right_robot'), cell, side) for side in (0,1)]

    robot = actors.spawn_actor_from_class(unreal.RobotArm, unreal.Vector(40,-420,0))
    robot.set_actor_label('DoorStation_Robot')
    robot.set_actor_rotation(unreal.Rotator(yaw=90), True)
    calibration = json.loads((P/'Scripts/robot_calibration.json').read_text(encoding='utf-8'))
    door.configure_robot(robot, calibration)
    dc = actors.spawn_actor_from_class(unreal.DoorAssemblyCell, unreal.Vector())
    dc.set_actor_label('DoorStation_Cell')
    dc.set_editor_property('assembly', car)
    dc.set_editor_property('robot', robot)
    dc.set_editor_property('externally_driven', True)
    dc.set_editor_property('record_csv', False)
    report['door_ik'] = door.solve_waypoints(robot, dc, verify_door)['selected']
    robot.set_actor_location(unreal.Vector(1040,-420,0), False, True)
    assert pose.call_method('RestoreReferencePose')
    wheel_roots = {c.get_name():c for c in cell.get_components_by_class(unreal.SceneComponent)}
    for i in range(4):
        wheel_roots['Wheel'+str(i)].set_world_transform(cell.call_method('GetMountWorld',(i,)),False,True)

    mesh_entries = {m['name']:m for m in manifest['meshes']}
    modules = []
    for m in manifest['modules']:
        p = m['pivot_ue_cm']
        module = actors.spawn_actor_from_class(unreal.VehicleProcessModule, unreal.Vector(250+p[0],p[1],p[2]))
        module.set_actor_label(m['id'])
        module.set_folder_path('VehicleModules/'+m['id'])
        module.set_editor_property('module_id', unreal.Name(m['id']))
        module.set_editor_property('vehicle_frame', car)
        module.set_editor_property('wheel_motion_enabled', False)
        module.set_editor_property('mesh_offset_cm', unreal.Vector(*[-v for v in p]))
        parts = []
        for name in m['meshes']:
            item = unreal.VehicleModuleMesh()
            mesh = unreal.load_asset('/Game/DigitalTwin/Camry/ProcessModules/Meshes/'+m['id']+'/'+name)
            assert mesh, name
            item.set_editor_property('mesh', mesh)
            item.set_editor_property('preview_visible', mesh_entries[name]['visible'])
            parts.append(item)
        module.set_editor_property('mesh_parts', parts)
        module.call_method('RebuildVisuals')
        assert module.call_method('CaptureReferencePose')
        if m['id'] in wheel_ids:
            parent = wheel_roots['Wheel'+str(wheel_ids.index(m['id']))]
        elif m['id'] == 'Door_FL':
            parent = comps['LeftDoor']
        else:
            parent = car.root_component
        module.attach_to_component(parent, unreal.Name('None'), unreal.AttachmentRule.KEEP_WORLD,
                                   unreal.AttachmentRule.KEEP_WORLD, unreal.AttachmentRule.KEEP_WORLD, False)
        modules.append(module)
    assert len(modules) == 42

    # Extend the adapted visual rail to cover both stations (not source CAD dimensions).
    fixed = next(a for a in all_actors if a.get_actor_label() == 'Traverse187_FixedRail')
    fixed.set_actor_location(fixed.get_actor_location()+unreal.Vector(500,0,0),False,True)
    scale = fixed.get_actor_scale3d()
    fixed.set_actor_scale3d(unreal.Vector(scale.x,scale.y*2.3,scale.z))
    # Large floor for the additional station.
    floor = actors.spawn_actor_from_class(unreal.StaticMeshActor,unreal.Vector(200,0,-10))
    floor.set_actor_label('IntegratedLine_Floor')
    floor.static_mesh_component.set_static_mesh(unreal.load_asset('/Engine/BasicShapes/Cube'))
    floor.set_actor_scale3d(unreal.Vector(30,16,.1))

    line = actors.spawn_actor_from_class(unreal.IntegratedAssemblyLine,unreal.Vector())
    line.set_actor_label('IntegratedAssemblyLine')
    line.set_editor_property('wheel_cell',cell)
    line.set_editor_property('door_cell',dc)
    line.set_editor_property('vehicle_at_wheel_station',car.get_actor_transform())
    line.set_editor_property('carriage_at_wheel_station',carriage.get_actor_transform())
    cell.set_editor_property('enable_transport',True)
    assert line.call_method('ResetLine'),line.get_editor_property('status')
    world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    world.get_world_settings().set_editor_property('default_game_mode',unreal.IntegratedAssemblyGameMode)
    eye = unreal.Vector(750,-1600,1000)
    unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).set_level_viewport_camera_info(eye,unreal.MathLibrary.find_look_at_rotation(eye,unreal.Vector(200,0,70)))
    assert levels.save_current_level()
    report.update(success=True,module_count=len(modules),wheel_centers_cm=[[v.x,v.y,v.z] for v in centers])
except Exception:
    report['failure'] = traceback.format_exc()
    unreal.log_error(report['failure'])
finally:
    (P/'Saved/IntegratedAssemblySetup.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
assert report['success'],report.get('failure')
