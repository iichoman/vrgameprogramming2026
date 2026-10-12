"""Exercise the real native fixed-step cell without saving a map.

Run in the editor after setup, or with UnrealEditor-Cmd after closing the editor.
The report distinguishes ideal fastening and assumed calibration from real plant
validation. Successful verification restores the pickup/home Idle configuration.
"""
import json
import math
import traceback
from pathlib import Path
import unreal

PROJECT = Path(unreal.Paths.project_dir()).resolve()
MAP = '/Game/DigitalTwin/Maps/DoorAssemblyCell'
REPORT = PROJECT / 'Saved/DoorAssemblyVerification.json'


def xyz(v):
    return [float(v.x), float(v.y), float(v.z)]


def qm(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def qi(a):
    norm = sum(v*v for v in a)
    return [-a[0]/norm, -a[1]/norm, -a[2]/norm, a[3]/norm]


def qr(q, v):
    return qm(qm(q, list(v)+[0.0]), qi(q))[:3]


def snapshot(t):
    q = t.rotation
    return {'translation_cm': xyz(t.translation), 'quaternion_xyzw': [q.x,q.y,q.z,q.w]}


def compose(local, parent):
    offset = qr(parent['quaternion_xyzw'], local['translation_cm'])
    return {'translation_cm': [a+b for a,b in zip(parent['translation_cm'],offset)],
            'quaternion_xyzw': qm(parent['quaternion_xyzw'], local['quaternion_xyzw'])}


def relative(world, parent):
    q = qi(parent['quaternion_xyzw'])
    return {'translation_cm': qr(q,[a-b for a,b in zip(world['translation_cm'],parent['translation_cm'])]),
            'quaternion_xyzw': qm(q,world['quaternion_xyzw'])}


def pose_error(a, b):
    distance_mm = 10*math.sqrt(sum((x-y)**2 for x,y in zip(a['translation_cm'],b['translation_cm'])))
    qa,qb = a['quaternion_xyzw'],b['quaternion_xyzw']
    norm_product = math.sqrt(sum(x*x for x in qa)*sum(x*x for x in qb))
    assert norm_product>1.e-12, 'Cannot compare a zero quaternion.'
    dot = abs(sum(x*y for x,y in zip(qa,qb)))/norm_product
    angle_deg = math.degrees(2*math.acos(min(1.0,max(0.0,dot))))
    return distance_mm, angle_deg


def assert_pose(a, b, position_mm=0.005, angle_deg=0.001, message='pose'):
    errors = pose_error(a,b)
    assert errors[0] <= position_mm and errors[1] <= angle_deg, (message,errors,a,b)


def find_cell():
    actor_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    cls = unreal.load_class(None, '/Script/test1.DoorAssemblyCell')
    assert cls, 'Compile/reload the native DoorAssemblyCell class first.'
    cells = [a for a in actor_subsystem.get_all_level_actors() if a.get_class() == cls]
    assert len(cells) == 1, ('Exactly one native assembly cell is required.',len(cells))
    return cells[0]


def verify_cell(cell=None, write_report=True):
    cell = cell or find_cell()
    robot = cell.get_editor_property('robot')
    assembly = cell.get_editor_property('assembly')
    assert robot and assembly
    door = next(c for c in assembly.get_components_by_class(unreal.SceneComponent) if c.get_name()=='LeftDoor')
    last_joint = next(c for c in robot.get_components_by_class(unreal.SceneComponent) if c.get_name()=='JointA6')
    references = [snapshot(c.get_editor_property('reference_pose_in_body'))
                  for c in assembly.get_components_by_class(unreal.DoorPoseComponent)]
    result = {'success': False, 'map': MAP, 'runs': [], 'checks': [],
              'plant_model': 'PID velocity servo, acceleration limits, first-order motor response; ideal rigid fastening.',
              'calibration_limits': 'Source asset geometry; joint ranges and TCP contact point require manufacturer/physical validation.',
              'cold_reload_verified': False, 'visual_editor_verified': False}
    # Reflected structs can alias native storage; keep numeric values before fault injection.
    original_pickup_values = xyz(cell.get_editor_property('pickup_offset_body_cm'))

    def restore_pickup():
        cell.set_editor_property('pickup_offset_body_cm', unreal.Vector(*original_pickup_values))

    def get_pose():
        return snapshot(door.get_world_transform())

    def check_tcp():
        # Independent scene-component hierarchy comparison against FK state.
        measured = compose(snapshot(robot.get_editor_property('tool_in_last_joint_cm')),
                           snapshot(last_joint.get_world_transform()))
        assert_pose(snapshot(robot.call_method('GetTCPWorld')), measured, message='FK versus scene joints')

    def run_cycle(frame_dt):
        assert cell.call_method('ResetCycle'), cell.get_editor_property('status')
        check_tcp()
        pickup = snapshot(cell.call_method('GetPickupDoorWorld'))
        assert_pose(get_pose(),pickup,message='Reset places door at pickup')
        assert cell.call_method('StartCycle'), cell.get_editor_property('status')
        states=[]
        grip_relative=None
        largest_tracking_error_mm=0.0
        for frame in range(int(180/frame_dt)):
            ok=cell.call_method('AdvanceSimulation',(frame_dt,))
            state=cell.call_method('GetStateLabel')
            if not states or states[-1]!=state:
                states.append(state)
            assert ok and state!='Fault', ('Native cycle failed',state,cell.get_editor_property('status'))
            assert math.isfinite(cell.get_editor_property('position_error_mm'))
            assert math.isfinite(cell.get_editor_property('orientation_error_deg'))
            largest_tracking_error_mm=max(largest_tracking_error_mm,cell.get_editor_property('position_error_mm'))
            if cell.get_editor_property('door_held') and not cell.get_editor_property('door_fastened'):
                actual_relative=relative(get_pose(),snapshot(robot.call_method('GetTCPWorld')))
                if grip_relative is None:
                    grip_relative=actual_relative
                assert_pose(actual_relative,grip_relative,message='Held door follows measured TCP')
            if frame % 30 == 0:
                check_tcp()
            if state=='Complete':
                break
        assert state=='Complete', ('Cycle exceeded test duration',state)
        assert cell.get_editor_property('door_fastened') and not cell.get_editor_property('door_held')
        # Release can last one fixed step and occur between render-frame samples;
        # its effect is checked separately by held=False and fastened=True.
        required=['Grip','Lift','Transfer','ApproachMount','Align','Fasten','Retract','Complete']
        assert all(s in states for s in required), ('Missing process stages',states)
        mount = snapshot(cell.call_method('GetMountDoorWorld'))
        mount_errors=pose_error(get_pose(),mount)
        assert mount_errors[0] <= cell.get_editor_property('position_tolerance_mm')
        assert mount_errors[1] <= cell.get_editor_property('orientation_tolerance_deg')
        achieved=get_pose()
        assert cell.call_method('AdvanceSimulation',(0.5,))
        assert_pose(get_pose(),achieved,message='Fastened door stays on body after robot retracts')
        return {'frame_dt_seconds':frame_dt,'states':states,
                'cycle_time_seconds':cell.get_editor_property('simulation_time_seconds'),
                'final_door_pose':achieved,'door_mount_position_error_mm':mount_errors[0],
                'door_mount_orientation_error_deg':mount_errors[1],
                'max_motion_tracking_error_mm':largest_tracking_error_mm,
                'final_joint_angles_deg':list(robot.get_editor_property('current_angles_degrees')),
                'csv':cell.get_editor_property('log_file')}

    try:
        for dt in (1.0/60.0,1.0/30.0):
            result['runs'].append(run_cycle(dt))
        first, second = result['runs']
        assert abs(first['cycle_time_seconds']-second['cycle_time_seconds']) <= 1.0/120.0+1.e-8
        assert_pose(first['final_door_pose'],second['final_door_pose'],message='Fixed-step frame independence')
        result['checks'].append('Two full cycles at 60/30 FPS yield the same achieved pose and simulation time.')
        result['checks'].append('Full-pose IK, scene FK agreement, measured TCP transport and retained achieved fastening pose.')

        # Stop while carrying a door, then confirm drives and held door do not move.
        assert cell.call_method('ResetCycle')
        assert cell.call_method('StartCycle')
        for _ in range(60*90):
            assert cell.call_method('AdvanceSimulation',(1.0/60.0,)), cell.get_editor_property('status')
            if cell.get_editor_property('door_held'):
                break
        assert cell.get_editor_property('door_held'), 'The stop test never reached transport.'
        cell.call_method('StopCycle')
        stopped_door=get_pose()
        stopped_tcp=snapshot(robot.call_method('GetTCPWorld'))
        assert cell.call_method('GetStateLabel')=='Stopped'
        for _ in range(120):
            assert cell.call_method('AdvanceSimulation',(1.0/60.0,))
        assert_pose(get_pose(),stopped_door,message='Door holds during stop')
        assert_pose(snapshot(robot.call_method('GetTCPWorld')),stopped_tcp,message='Robot holds during stop')
        assert not cell.get_editor_property('door_fastened')
        assert cell.call_method('ResetCycle')
        assert_pose(get_pose(),snapshot(cell.call_method('GetPickupDoorWorld')),message='Stop recovery')
        result['checks'].append('Stop during transport holds measured joint/door poses; Reset restores pickup and clears fastening.')

        # Failed IK may not move the arm or authorize a grip/fastening.
        cell.set_editor_property('pickup_offset_body_cm',unreal.Vector(0,-10000,20))
        assert cell.call_method('ResetCycle')
        before_angles=list(robot.get_editor_property('current_angles_degrees'))
        before_targets=list(robot.get_editor_property('target_angles_degrees'))
        before_tcp=snapshot(robot.call_method('GetTCPWorld'))
        assert not cell.call_method('StartCycle')
        assert cell.call_method('GetStateLabel')=='Fault'
        assert not cell.get_editor_property('door_held') and not cell.get_editor_property('door_fastened')
        assert before_angles==list(robot.get_editor_property('current_angles_degrees'))
        assert before_targets==list(robot.get_editor_property('target_angles_degrees'))
        assert_pose(snapshot(robot.call_method('GetTCPWorld')),before_tcp,message='Unreachable target does not move arm')
        result['unreachable_status']=cell.get_editor_property('status')
        result['checks'].append('Unreachable pickup fails IK without moving the robot or fastening the door.')
        restore_pickup()
        assert cell.call_method('ResetCycle')
        assert not cell.call_method('AdvanceSimulation',(float('nan'),))
        assert cell.call_method('GetStateLabel')=='Idle'
        result['checks'].append('Nonfinite simulation input rejects without changing Idle state.')
        assert references==[snapshot(c.get_editor_property('reference_pose_in_body'))
                             for c in assembly.get_components_by_class(unreal.DoorPoseComponent)]
        result['checks'].append('Serialized normal door reference stays unchanged by transport/reset.')
        result['success']=True
        result['saved_idle_door_pose']=get_pose()
        result['saved_initial_joint_angles_deg']=list(robot.get_editor_property('initial_angles_degrees'))
    except Exception:
        result['failure']=traceback.format_exc()
        raise
    finally:
        # Cleanup diagnostics must not replace the original verification failure.
        try:
            restore_pickup()
            if not cell.call_method('ResetCycle'):
                result['cleanup_warning']=cell.get_editor_property('status')
        except Exception:
            result['cleanup_warning']=traceback.format_exc()
        if write_report:
            REPORT.parent.mkdir(parents=True,exist_ok=True)
            REPORT.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    unreal.log('DOOR_ASSEMBLY_VERIFICATION_SUCCESS ' + str(REPORT))
    return result


def main(load=True):
    if load:
        assert unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).load_level(MAP)
    return verify_cell()


if __name__=='__main__':
    main(load=not bool(globals().get('DOOR_ASSEMBLY_VERIFY_CURRENT_LEVEL',False)))
