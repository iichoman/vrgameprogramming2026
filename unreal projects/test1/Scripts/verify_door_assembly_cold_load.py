"""Cold-load the saved native cell and verify persisted references plus runtime.

Run in a fresh UnrealEditor-Cmd process after closing the GUI editor:
  -EnablePlugins=PythonScriptPlugin,EditorScriptingUtilities -run=pythonscript
  -script=".../Scripts/verify_door_assembly_cold_load.py" -unattended -NullRHI

Never saves a map or edits project settings. Output files are confined to Saved.
The existing source preview is loaded read-only to compare its serialized door
references and fixture contacts against the saved cell, independently of reports.
"""
import hashlib
import importlib.util
import json
import math
import traceback
from datetime import datetime, timezone
from pathlib import Path
import unreal

PROJECT=Path(unreal.Paths.project_dir()).resolve()
SOURCE_MAP='/Game/DigitalTwin/Maps/CamryAssemblyPreview'
MAP='/Game/DigitalTwin/Maps/DoorAssemblyCell'
REPORT=PROJECT/'Saved/DoorAssemblyColdLoadVerification.json'
PRIMARY_REPORT=PROJECT/'Saved/DoorAssemblyVerification.json'


def xyz(v):
    return [float(v.x),float(v.y),float(v.z)]


def exact_vector(actual,expected,label,tolerance=1.e-5):
    assert len(actual)==len(expected)
    assert all(math.isfinite(x) for x in actual), (label,'nonfinite vector',actual)
    assert max(abs(a-b) for a,b in zip(actual,expected))<=tolerance, (label,actual,expected)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def actors_one_each():
    subsystem=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    all_actors=subsystem.get_all_level_actors()
    result={}
    for name in ('CamryAssembly','DoorFixture','RobotArm','DoorAssemblyCell'):
        cls=unreal.load_class(None,'/Script/test1.'+name)
        assert cls, ('Missing persisted native class',name)
        matches=[a for a in all_actors if a.get_class()==cls]
        assert len(matches)==1, ('Expected exactly one native actor',name,len(matches))
        result[name]=matches[0]
    return result


def vehicle_baseline(assembly,verification):
    pose={}
    for c in assembly.get_components_by_class(unreal.DoorPoseComponent):
        assert c.call_method('HasReferencePose'), ('Missing saved door reference',c.get_name())
        pose[c.get_name()]=verification.snapshot(c.get_editor_property('reference_pose_in_body'))
    assert set(pose)=={'LeftDoorPose','RightDoorPose'}
    return pose


def fixture_snapshot(fixture):
    result=[]
    for contact in fixture.get_editor_property('contacts'):
        item={}
        for name in ('name','role','notes'):
            item[name]=str(contact.get_editor_property(name))
        for name in ('part_id','region_only'):
            item[name]=contact.get_editor_property(name)
        for name in ('door_point_local_cm','outward_normal_local','preview_size_mm','offset_mm'):
            item[name]=xyz(contact.get_editor_property(name))
        item['preview_stem_length_mm']=float(contact.get_editor_property('preview_stem_length_mm'))
        result.append(item)
    assert len(result)>=6, ('Expected existing fixture contacts',len(result))
    return {'contacts':result,'layout_source':str(fixture.get_editor_property('layout_source')),
            'preview_retraction_mm':float(fixture.get_editor_property('preview_retraction_mm'))}


def source_baseline(levels,verification):
    assert levels.load_level(SOURCE_MAP)
    subsystem=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    all_actors=subsystem.get_all_level_actors()
    assembly_cls=unreal.load_class(None,'/Script/test1.CamryAssembly')
    fixture_cls=unreal.load_class(None,'/Script/test1.DoorFixture')
    assemblies=[a for a in all_actors if a.get_class()==assembly_cls]
    fixtures=[a for a in all_actors if a.get_class()==fixture_cls]
    assert len(assemblies)==1 and len(fixtures)==1
    assert fixtures[0].get_editor_property('assembly')==assemblies[0]
    return {'door_references':vehicle_baseline(assemblies[0],verification),
            'fixture':fixture_snapshot(fixtures[0]),
            'assembly_transform':verification.snapshot(assemblies[0].get_actor_transform())}


def main():
    result={'success':False,'cold_reload_verified':False,'map':MAP,'source_map':SOURCE_MAP,
            'checks':[],'executed_utc':datetime.now(timezone.utc).isoformat(),
            'visual_editor_verified':False,'map_saved_by_this_script':False}
    map_files=[PROJECT/'Content/DigitalTwin/Maps/CamryAssemblyPreview.umap',
               PROJECT/'Content/DigitalTwin/Maps/DoorAssemblyCell.umap']
    before_hashes={str(path.relative_to(PROJECT)):digest(path) for path in map_files}
    try:
        spec=importlib.util.spec_from_file_location('door_assembly_cold_runtime',str(PROJECT/'Scripts/verify_door_assembly_cell.py'))
        verification=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(verification)
        levels=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
        source=source_baseline(levels,verification)
        assert levels.load_level(MAP)
        native=actors_one_each()
        assembly=native['CamryAssembly'];fixture=native['DoorFixture']
        robot=native['RobotArm'];cell=native['DoorAssemblyCell']
        assert cell.get_editor_property('assembly')==assembly
        assert cell.get_editor_property('robot')==robot
        assert fixture.get_editor_property('assembly')==assembly
        result['actor_classes']={name:a.get_class().get_path_name() for name,a in native.items()}
        result['checks'].append('Exactly one saved native vehicle, fixture, robot and assembly cell; references resolve to this map.')

        world=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
        mode=world.get_world_settings().get_editor_property('default_game_mode')
        assert mode and mode.get_path_name()=='/Script/test1.DoorAssemblyGameMode', ('Saved operator GameMode',mode)
        result['game_mode']=mode.get_path_name()
        result['checks'].append('Native DoorAssemblyGameMode override survives cold map load.')

        saved_references=vehicle_baseline(assembly,verification)
        for name,pose in source['door_references'].items():
            verification.assert_pose(saved_references[name],pose,message='Original serialized '+name)
        assert fixture_snapshot(fixture)==source['fixture'], 'Fixture contacts changed while copying/setup.'
        verification.assert_pose(verification.snapshot(assembly.get_actor_transform()),source['assembly_transform'],
                                 message='Vehicle placement preserved')
        result['door_references']=saved_references
        result['fixture_contact_count']=len(source['fixture']['contacts'])
        result['checks'].append('Both normal door references, vehicle placement and all source fixture contacts are preserved.')

        calibration=json.loads((PROJECT/'Scripts/robot_calibration.json').read_text(encoding='utf-8'))
        joints=list(robot.get_editor_property('joints'))
        assert len(joints)==6
        for joint,expected in zip(joints,calibration['joints']):
            assert str(joint.get_editor_property('name'))==expected['name']
            verification.assert_pose(verification.snapshot(joint.get_editor_property('home_local_cm')),expected['home_local'],
                                     message='Saved joint pivot '+expected['name'])
            exact_vector(xyz(joint.get_editor_property('axis_local')),expected['axis_local'],'Saved joint axis '+expected['name'])
            assert abs(joint.get_editor_property('minimum_degrees')-expected['min_angle_deg'])<1.e-9
            assert abs(joint.get_editor_property('maximum_degrees')-expected['max_angle_deg'])<1.e-9
            assert joint.get_editor_property('max_velocity_degrees')>0
            assert joint.get_editor_property('max_acceleration_degrees')>0
        verification.assert_pose(verification.snapshot(robot.get_editor_property('tool_in_last_joint_cm')),
                                 calibration['tcp_in_last_joint'],message='Saved TCP calibration')
        bindings=list(robot.get_editor_property('mesh_bindings'))
        assert len(bindings)==len(calibration['mesh_bindings']) and len(bindings)>=17
        for binding,expected in zip(bindings,calibration['mesh_bindings']):
            mesh=binding.get_editor_property('mesh')
            assert mesh and mesh.get_path_name()==expected['mesh'], ('Unresolved saved robot mesh',expected['mesh'])
            assert binding.get_editor_property('joint_index')==expected['parent_joint_index']
            verification.assert_pose(verification.snapshot(binding.get_editor_property('relative_pose_cm')),
                                     expected['local_transform'],message='Saved mesh binding '+expected['name'])
        result['mesh_binding_count']=len(bindings)
        result['checks'].append('Six measured joint pivots/axes, TCP and all 17 source robot mesh bindings survive serialization.')

        # Keep a scalar snapshot rather than a reflected struct view: Unreal may
        # alias UPROPERTY memory and otherwise mutate an alleged saved value.
        pickup=xyz(cell.get_editor_property('pickup_offset_body_cm'))
        exact_vector(pickup,[0,-100,20],'Normal saved pickup offset')
        exact_vector(xyz(cell.get_editor_property('approach_offset_body_cm')),[0,-25,0],'Saved approach offset')
        exact_vector(xyz(cell.get_editor_property('lift_offset_body_cm')),[0,0,25],'Saved lift offset')
        exact_vector(xyz(cell.get_editor_property('transfer_offset_body_cm')),[0,-50,25],'Saved transfer offset')
        assert not cell.get_editor_property('auto_start')
        assert abs(cell.get_editor_property('fixed_step_seconds')-1.0/120.0)<1.e-12
        for name in ('position_tolerance_mm','orientation_tolerance_deg','settle_seconds',
                     'stage_timeout_seconds','fastening_seconds'):
            value=float(cell.get_editor_property(name))
            assert math.isfinite(value) and value>0, ('Invalid saved control setting',name,value)
        initial=[float(x) for x in robot.get_editor_property('initial_angles_degrees')]
        assert len(initial)==6 and all(math.isfinite(x) for x in initial)
        setup_path=PROJECT/'Saved/DoorAssemblySetup.json'
        if setup_path.exists():
            setup=json.loads(setup_path.read_text(encoding='utf-8'))
            assert setup['success'] and setup['saved']
            exact_vector(initial,setup['ik_preflight']['selected']['initial_home_angles_deg'],'Saved home joint angles')
        door=next(c for c in assembly.get_components_by_class(unreal.SceneComponent) if c.get_name()=='LeftDoor')
        saved_door=verification.snapshot(door.get_world_transform())
        expected_pickup=verification.compose(source['door_references']['LeftDoorPose'],source['assembly_transform'])
        expected_pickup['translation_cm']=[a+b for a,b in zip(expected_pickup['translation_cm'],
                                             verification.qr(source['assembly_transform']['quaternion_xyzw'],pickup))]
        verification.assert_pose(saved_door,expected_pickup,message='Serialized door at normal pickup station')
        assert cell.call_method('ResetCycle'),str(cell.get_editor_property('status'))
        assert cell.call_method('GetStateLabel')=='Idle'
        assert not cell.get_editor_property('door_held') and not cell.get_editor_property('door_fastened')
        exact_vector(list(robot.get_editor_property('current_angles_degrees')),initial,'Reset restores saved robot home')
        verification.assert_pose(verification.snapshot(door.get_world_transform()),expected_pickup,
                                 message='Cold Reset preserves normal pickup')
        result['pickup_offset_body_cm']=pickup
        result['serialized_idle_door_pose']=saved_door
        result['initial_joint_angles_deg']=initial
        result['checks'].append('Normal pickup offset (0,-100,20) and achieved home configuration are saved; Reset restores Idle.')

        # Uses the same native PID/IK runtime; no setup/preflight or fresh component
        # calibration is allowed to repair the persisted values before this test.
        runtime=verification.verify_cell(cell,write_report=False)
        assert runtime['success']
        runtime['cold_reload_verified']=True
        result['runtime_verification']=runtime
        exact_vector(xyz(cell.get_editor_property('pickup_offset_body_cm')),[0,-100,20],
                     'Pickup offset restored after unreachable-target test')
        verification.assert_pose(verification.snapshot(door.get_world_transform()),expected_pickup,
                                 message='Final cleanup restores normal pickup')
        for name,pose in source['door_references'].items():
            verification.assert_pose(vehicle_baseline(assembly,verification)[name],pose,message='Baseline after cold runtime')
        result['checks'].append('Cold-loaded saved actors complete both PID/IK cycles, stop/recovery and unreachable-target rejection.')
        after_hashes={str(path.relative_to(PROJECT)):digest(path) for path in map_files}
        assert before_hashes==after_hashes, 'A map file changed during read-only cold verification.'
        result['map_sha256']=after_hashes
        result['checks'].append('Source preview and new cell .umap SHA-256 values stay unchanged throughout verification.')
        result['success']=True
        result['cold_reload_verified']=True
        if PRIMARY_REPORT.exists():
            primary=json.loads(PRIMARY_REPORT.read_text(encoding='utf-8'))
            if primary.get('success'):
                primary['cold_reload_verified']=True
                primary['cold_reload_report']=str(REPORT.relative_to(PROJECT))
                PRIMARY_REPORT.write_text(json.dumps(primary,indent=2)+'\n',encoding='utf-8')
        unreal.log('DOOR_ASSEMBLY_COLD_LOAD_SUCCESS')
        return result
    except Exception:
        result['failure']=traceback.format_exc()
        raise
    finally:
        REPORT.parent.mkdir(parents=True,exist_ok=True)
        REPORT.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':
    main()
