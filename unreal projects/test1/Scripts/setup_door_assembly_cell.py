"""Create/configure the native robot cell from the existing vehicle preview.

Run after native reflection has loaded in the current editor. The source map is
never saved or modified. The new map is saved only after full native verification
and when DOOR_ASSEMBLY_SAVE=True is explicitly passed by the caller.
"""
import importlib.util
import json
import math
import traceback
from pathlib import Path
import unreal

PROJECT=Path(unreal.Paths.project_dir()).resolve()
SOURCE_MAP='/Game/DigitalTwin/Maps/CamryAssemblyPreview'
MAP='/Game/DigitalTwin/Maps/DoorAssemblyCell'
REPORT=PROJECT/'Saved/DoorAssemblySetup.json'


def vec(v):
    return unreal.Vector(*[float(x) for x in v])


def transform(data):
    result=unreal.Transform()
    result.translation=vec(data['translation_cm'])
    result.rotation=unreal.Quat(*data['quaternion_xyzw'])
    result.scale3d=vec(data.get('scale',[1,1,1]))
    return result


def qm(a,b):
    x,y,z,w=a;X,Y,Z,W=b
    return [w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z]


def inward_grasp_quaternion(twist_degrees):
    # TCP/tool -Z points toward body +Y from the left/outside face of the door.
    # Twisting about world Y leaves that approach normal unchanged.
    twist=math.radians(twist_degrees)*0.5
    half=math.sqrt(0.5)
    return qm([0,math.sin(twist),0,math.cos(twist)],[half,0,0,half])


def configure_robot(robot, calibration):
    definitions=[]
    for item in calibration['joints']:
        joint=unreal.RobotJointSetup()
        joint.set_editor_property('name',item['name'])
        joint.set_editor_property('home_local_cm',transform(item['home_local']))
        # Native convention: positive right-handed quaternion rotation about +Z.
        # BP SetJointAngle uses the opposite sign, captured separately in JSON.
        joint.set_editor_property('axis_local',vec(item['axis_local']))
        joint.set_editor_property('minimum_degrees',item['min_angle_deg'])
        joint.set_editor_property('maximum_degrees',item['max_angle_deg'])
        joint.set_editor_property('max_velocity_degrees',45.0)
        joint.set_editor_property('max_acceleration_degrees',90.0)
        definitions.append(joint)
    bindings=[]
    for item in calibration['mesh_bindings']:
        binding=unreal.RobotMeshBinding()
        mesh=unreal.load_asset(item['mesh'])
        assert mesh, ('Missing robot mesh',item['mesh'])
        binding.set_editor_property('mesh',mesh)
        binding.set_editor_property('joint_index',item['parent_joint_index'])
        binding.set_editor_property('relative_pose_cm',transform(item['local_transform']))
        bindings.append(binding)
    robot.set_editor_property('joints',definitions)
    robot.set_editor_property('mesh_bindings',bindings)
    robot.set_editor_property('tool_in_last_joint_cm',transform(calibration['tcp_in_last_joint']))
    robot.set_editor_property('initial_angles_degrees',[0.0]*6)
    robot.set_editor_property('calibration_source',
        'robot_calibration.json: measured source KUKA BP hierarchy; exact manufacturer model, joint ranges and contact TCP unverified. '
        'Native angles rotate about local +Z; source BP angles have opposite sign. '
        'PID velocity servo with first-order motor response; contact/load dynamics are not calibrated.')
    assert robot.call_method('InitializeArm'), robot.get_editor_property('last_status')


def actor_of_class(actor_subsystem,path,label):
    cls=unreal.load_class(None,path)
    assert cls, ('Native class missing; build and reload reflection first.',path)
    existing=[a for a in actor_subsystem.get_all_level_actors() if a.get_class()==cls]
    assert len(existing)<=1, ('Duplicate native actors in assembly map.',path,len(existing))
    result=existing[0] if existing else actor_subsystem.spawn_actor_from_class(cls,unreal.Vector(0,0,0))
    result.set_actor_label(label)
    return result


def solve_waypoints(robot,cell,verify):
    assert cell.call_method('ResetCycle'),cell.get_editor_property('status')
    mount=verify.snapshot(cell.call_method('GetMountDoorWorld'))
    pickup=cell.get_editor_property('pickup_offset_body_cm')
    approach=cell.get_editor_property('approach_offset_body_cm')
    lift=cell.get_editor_property('lift_offset_body_cm')
    transfer=cell.get_editor_property('transfer_offset_body_cm')
    # Preview map uses an identity body; command offsets are still transformed in
    # native runtime by the body's frame. Explicitly reject any unintended setup.
    body=next(c for c in cell.get_editor_property('assembly').get_components_by_class(unreal.SceneComponent) if c.get_name()=='Body')
    body_pose=verify.snapshot(body.get_world_transform())
    body_rotation=body_pose['quaternion_xyzw']
    def target(offset,quaternion):
        displacement=verify.qr(body_rotation,offset)
        grasp=verify.qr(mount['quaternion_xyzw'],[0,-18,0])
        return transform({'translation_cm':[a+b+c for a,b,c in zip(mount['translation_cm'],displacement,grasp)],
                          'quaternion_xyzw':quaternion})
    offsets=[('ApproachPickup',[pickup.x+approach.x,pickup.y+approach.y,pickup.z+approach.z]),
             ('Grip',[pickup.x,pickup.y,pickup.z]),
             ('Lift',[pickup.x+lift.x,pickup.y+lift.y,pickup.z+lift.z]),
             ('Transfer',[transfer.x,transfer.y,transfer.z]),
             ('ApproachMount',[approach.x,approach.y,approach.z]),
             ('Align',[0,0,0]),('Retract',[approach.x,approach.y,approach.z])]
    seeds=[[0,40,-65,0,25,0],[0,60,-40,0,-20,0],[0,-40,65,0,-25,0],
           [0,40,-65,45,25,-45],[0,0,0,0,0,0]]
    attempts=[]
    for twist in (0,45,-45,90,-90,180):
        quaternion=inward_grasp_quaternion(twist)
        for seed in seeds:
            assert robot.call_method('SetInitialAngles',(seed,))
            attempt={'twist_degrees':twist,'seed_deg':seed,'waypoints':[]}
            # Solve the demanding low mount first, then warm-start each waypoint.
            if not robot.call_method('CommandTCPWorld',(target([0,0,0],quaternion),)):
                attempt['failure']=robot.get_editor_property('last_status')
                attempts.append(attempt)
                continue
            solution=list(robot.get_editor_property('target_angles_degrees'))
            assert robot.call_method('SetInitialAngles',(solution,))
            valid=True
            for name,offset in offsets:
                if not robot.call_method('CommandTCPWorld',(target(offset,quaternion),)):
                    attempt['failure']=name+': '+robot.get_editor_property('last_status')
                    valid=False
                    break
                solution=list(robot.get_editor_property('target_angles_degrees'))
                attempt['waypoints'].append({'stage':name,'joint_angles_deg':solution})
                assert robot.call_method('SetInitialAngles',(solution,))
            home_offset=[pickup.x+approach.x,pickup.y+approach.y,pickup.z+approach.z+35]
            if valid and robot.call_method('CommandTCPWorld',(target(home_offset,quaternion),)):
                home=list(robot.get_editor_property('target_angles_degrees'))
                assert robot.call_method('SetInitialAngles',(home,))
                # Express selected TCP orientation in the door's own frame.
                local_rotation=qm(verify.qi(mount['quaternion_xyzw']),quaternion)
                cell.set_editor_property('tcp_in_door_cm',transform({'translation_cm':[0,-18,0],
                                                                    'quaternion_xyzw':local_rotation}))
                assert cell.call_method('ResetCycle'),cell.get_editor_property('status')
                attempt['initial_home_angles_deg']=home
                attempts.append(attempt)
                return {'selected':attempt,'attempts':attempts}
            if valid:
                attempt['failure']='Home: '+robot.get_editor_property('last_status')
            attempts.append(attempt)
    raise AssertionError(('No common grasp orientation can reach every waypoint.',attempts))


def main(save=False):
    report={'success':False,'map':MAP,'source_map':SOURCE_MAP,'saved':False}
    try:
        calibration=json.loads((PROJECT/'Scripts/robot_calibration.json').read_text(encoding='utf-8'))
        assert len(calibration['joints'])==6
        assert all(math.isfinite(v) for j in calibration['joints'] for v in j['home_local']['translation_cm']+j['home_local']['quaternion_xyzw'])
        levels=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
        if not unreal.EditorAssetLibrary.does_asset_exist(MAP):
            # Worlds must be copied through the map-specific API. Generic asset
            # duplication leaves a standalone UWorld that fails map-load GC.
            assert levels.new_level_from_template(MAP,SOURCE_MAP), 'Could not create map from preview template.'
        else:
            current=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world().get_path_name().split('.')[0]
            if current!=MAP:
                assert levels.load_level(MAP)
        actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
        assembly_class=unreal.load_class(None,'/Script/test1.CamryAssembly')
        assemblies=[a for a in actors.get_all_level_actors() if a.get_class()==assembly_class]
        assert len(assemblies)==1, 'The copied preview requires one existing vehicle assembly.'
        assembly=assemblies[0]
        # The preview has multiple directional lights. Give the copied map one
        # unambiguous primary light for forward shading/reflections.
        directional=[]
        for actor in actors.get_all_level_actors():
            directional.extend(actor.get_components_by_class(unreal.DirectionalLightComponent))
        for index,light in enumerate(sorted(directional,key=lambda c:c.get_path_name())):
            light.set_editor_property('forward_shading_priority',len(directional)-index)
        robot=actor_of_class(actors,'/Script/test1.RobotArm','RobotArm_PID_IK')
        robot.set_actor_location(unreal.Vector(40,-420,0),False,True)
        robot.set_actor_rotation(unreal.Rotator(pitch=0,yaw=90,roll=0),True)
        robot.set_actor_scale3d(unreal.Vector(1,1,1))
        configure_robot(robot,calibration)
        cell=actor_of_class(actors,'/Script/test1.DoorAssemblyCell','DoorAssemblyCell')
        cell.set_actor_location(unreal.Vector(0,0,0),False,True)
        cell.set_editor_property('assembly',assembly)
        cell.set_editor_property('robot',robot)
        cell.set_editor_property('pickup_offset_body_cm',unreal.Vector(0,-100,20))
        cell.set_editor_property('approach_offset_body_cm',unreal.Vector(0,-25,0))
        cell.set_editor_property('lift_offset_body_cm',unreal.Vector(0,0,25))
        cell.set_editor_property('transfer_offset_body_cm',unreal.Vector(0,-50,25))
        cell.set_editor_property('auto_start',False)
        cell.set_editor_property('record_csv',True)
        cell.set_editor_property('fixed_step_seconds',1.0/120.0)
        cell.set_editor_property('position_tolerance_mm',2.0)
        cell.set_editor_property('orientation_tolerance_deg',0.5)
        cell.set_editor_property('settle_seconds',0.3)
        cell.set_editor_property('stage_timeout_seconds',30.0)
        cell.set_editor_property('fastening_seconds',1.0)
        game_mode=unreal.load_class(None,'/Script/test1.DoorAssemblyGameMode')
        assert game_mode, 'Native operator GameMode is missing.'
        world=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
        world.get_world_settings().set_editor_property('default_game_mode',game_mode)
        spec=importlib.util.spec_from_file_location('door_assembly_verify',str(PROJECT/'Scripts/verify_door_assembly_cell.py'))
        verification=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(verification)
        report['ik_preflight']=solve_waypoints(robot,cell,verification)
        report['verification']=verification.verify_cell(cell)
        assert report['verification']['success']
        report['success']=True
        report['robot_pose']=verification.snapshot(robot.get_actor_transform())
        report['tcp_in_door']=verification.snapshot(cell.get_editor_property('tcp_in_door_cm'))
        eye=unreal.Vector(320,-650,310)
        aim=unreal.Vector(30,-180,100)
        unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).set_level_viewport_camera_info(
            eye,unreal.MathLibrary.find_look_at_rotation(eye,aim))
        if save:
            assert levels.save_current_level(), 'Could not save the verified cell map.'
            report['saved']=True
        unreal.log('DOOR_ASSEMBLY_SETUP_SUCCESS; saved='+str(report['saved']))
        return report
    except Exception:
        report['failure']=traceback.format_exc()
        raise
    finally:
        REPORT.parent.mkdir(parents=True,exist_ok=True)
        REPORT.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':
    main(save=bool(globals().get('DOOR_ASSEMBLY_SAVE',False)))
