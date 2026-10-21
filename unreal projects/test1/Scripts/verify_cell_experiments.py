"""Verify saved native PID cell experiments. Never saves or repairs map assets.

Fresh process: UnrealEditor-Cmd project -EnablePlugins=PythonScriptPlugin,EditorScriptingUtilities
  -run=pythonscript -script=.../verify_cell_experiments.py -unattended -NullRHI
Editor current level: runpy.run_path(script, init_globals={'VERIFY_CURRENT_LEVEL': True})
"""
import csv
import hashlib
import json
import math
from pathlib import Path
import traceback
import unreal

PROJECT=Path(unreal.Paths.project_dir()).resolve()
REPORT=PROJECT/'Saved/DoorCellExperiments.json'
MAP='/Game/DigitalTwin/Maps/DoorAssemblyCell'

def enum(v):
    return str(v).split('.')[-1].split(':')[0].strip('<> ')

def pose(t):
    return [t.translation.x,t.translation.y,t.translation.z,t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]

def check_close(a,b,tolerance=1.e-5):
    assert len(a)==len(b) and max(abs(x-y) for x,y in zip(a,b))<=tolerance,(a,b)

def main():
    report={'success':False,'runs':[],'checks':[], 'model':'Virtual pose sensors; ideal pickup detection and rigid fastening; six signed baseline gauges.'}
    paths=[PROJECT/'Content/DigitalTwin/Maps/CamryAssemblyPreview.umap',PROJECT/'Content/DigitalTwin/Maps/DoorAssemblyCell.umap']
    hashes={str(p.relative_to(PROJECT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    cell=None
    try:
        if not globals().get('VERIFY_CURRENT_LEVEL',False):
            assert unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).load_level(MAP)
        cls=unreal.load_class(None,'/Script/test1.DoorAssemblyCell')
        cells=[a for a in unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors() if a.get_class()==cls]
        assert len(cells)==1
        cell=cells[0]
        robot=cell.get_editor_property('robot');assembly=cell.get_editor_property('assembly')
        door=next(c for c in assembly.get_components_by_class(unreal.SceneComponent) if c.get_name()=='LeftDoor')
        controller=next(c for c in assembly.get_components_by_class(unreal.DoorPoseComponent) if c.get_name()=='LeftDoorPose')
        inspection=assembly.get_components_by_class(unreal.DoorInspectionComponent)[0]
        reference=pose(controller.get_editor_property('reference_pose_in_body'))
        def baselines():
            return [[float(v.x),float(v.y),float(v.z)] for v in [p.get_editor_property('reference_door_point_body_cm') for p in inspection.get_editor_property('points')]]
        station_baselines=baselines()
        S=unreal.DoorCellScenario;M=unreal.DoorCellControlMode
        def select(s,mode=M.INDEPENDENT_VERIFICATION):
            assert cell.call_method('SelectScenario',(s,mode)),cell.get_editor_property('status')
        def run(s,mode=M.INDEPENDENT_VERIFICATION,dt=1/60,settings=None):
            select(s,mode)
            if settings:
                for name,value in settings.items():cell.set_editor_property(name,value)
                assert cell.call_method('ResetCycle')
            initial=pose(door.get_world_transform())
            assert cell.call_method('StartCycle'),cell.get_editor_property('status')
            check_close(initial,pose(door.get_world_transform()))
            states=[]
            for _ in range(math.ceil(120/dt)):
                ok=cell.call_method('AdvanceSimulation',(dt,))
                state=cell.call_method('GetStateLabel')
                if not states or states[-1]!=state:states.append(state)
                if state in ('Complete','Fault'):
                    assert ok==(state!='Fault')
                    break
                assert ok,cell.get_editor_property('status')
            assert state in ('Complete','Fault'),(state,cell.get_editor_property('status'))
            outcome=enum(cell.get_editor_property('outcome'))
            stations=[{'name':str(q.get_editor_property('name')),'gap_mm':q.get_editor_property('gap_change_mm'),
                       'flush_mm':q.get_editor_property('flush_change_mm'),'pass':q.get_editor_property('passed')} for q in cell.get_editor_property('station_quality')]
            data={'scenario':enum(s),'mode':enum(mode),'frame_dt':dt,'state':state,'outcome':outcome,'states':states,
                  'time_s':cell.get_editor_property('simulation_time_seconds'),'pose':pose(door.get_world_transform()),
                  'held':cell.get_editor_property('door_held'),'fastened':cell.get_editor_property('door_fastened'),
                  'align_error_mm':cell.get_editor_property('alignment_position_error_mm'),'stations':stations,
                  'status':cell.get_editor_property('status'),'csv':cell.get_editor_property('log_file')}
            csv_path=Path(unreal.Paths.convert_relative_path_to_full(data['csv']))
            rows=list(csv.DictReader(csv_path.open(encoding='utf-8-sig')))
            assert rows and 'p6_flush_mm' in rows[-1] and rows[-1]['outcome'].upper()==outcome.replace('_',''),(outcome,rows[-1] if rows else None)
            metadata=json.loads(csv_path.with_suffix('.json').read_text(encoding='utf-8-sig'))
            assert metadata['outcome'].upper()==outcome.replace('_','') and len(metadata['stations'])==6
            report['runs'].append(data)
            return data
        a=run(S.NORMAL);b=run(S.NORMAL,dt=1/30)
        assert a['state']==b['state']=='Complete' and a['outcome']==b['outcome']=='PASS'
        check_close(a['pose'],b['pose']);assert abs(a['time_s']-b['time_s'])<=1/120+1.e-8
        report['checks'].append('Normal six-point quality passes; 30/60 FPS outcomes and final poses match.')
        bad=run(S.SENSOR_BIAS,M.BASIC)
        assert bad['state']=='Complete' and bad['outcome']=='QUALITY_FAIL' and bad['fastened']
        assert bad['align_error_mm']<=cell.get_editor_property('position_tolerance_mm')
        assert any(not q['pass'] for q in bad['stations'])
        blocked=run(S.SENSOR_BIAS)
        assert blocked['state']=='Fault' and blocked['outcome']=='BLOCKED' and not blocked['fastened']
        assert 'Independent' in blocked['status']
        report['checks'].append('Identical +5mm alignment bias causes Basic bad fastening; independent verification blocks before fastening.')
        shifted=run(S.DISPLACED_DOOR)
        assert shifted['outcome']=='PASS' and 'Grip' in shifted['states'] and shifted['fastened']
        report['checks'].append('Displaced/rotated initial door is picked from its current pose and passes final quality.')
        combined={'initial_door_offset_mm':unreal.Vector(20,-30,10),'initial_door_rotation_deg':unreal.Rotator(0,2,0),
                  'alignment_sensor_bias_mm':unreal.Vector(0,5,0)}
        assert run(S.CUSTOM,M.BASIC,settings=combined)['outcome']=='QUALITY_FAIL'
        assert run(S.CUSTOM,settings=combined)['outcome']=='BLOCKED'
        angular={'alignment_sensor_rotation_bias_deg':unreal.Rotator(0,0,1)}
        assert run(S.CUSTOM,M.BASIC,settings=angular)['outcome']=='QUALITY_FAIL'
        assert run(S.CUSTOM,settings=angular)['outcome']=='BLOCKED'
        report['checks'].append('Combined placement plus sensor bias and separate angular sensor bias reproduce bad fastening versus blocking.')
        for s in (S.UNREACHABLE_DOOR,S.REVERSED_DOOR,S.MISSING_DOOR):
            select(s)
            before=pose(door.get_world_transform());angles=list(robot.get_editor_property('current_angles_degrees'))
            targets=list(robot.get_editor_property('target_angles_degrees'))
            assert not cell.call_method('StartCycle')
            assert cell.call_method('GetStateLabel')=='Fault'
            assert not cell.get_editor_property('door_held') and not cell.get_editor_property('door_fastened')
            check_close(before,pose(door.get_world_transform()))
            assert angles==list(robot.get_editor_property('current_angles_degrees'))
            assert targets==list(robot.get_editor_property('target_angles_degrees'))
            assert not cell.call_method('AdvanceSimulation',(0.1,))
        report['checks'].append('Out-of-window/reversed/missing door fails before motion, grasp or fastening.')
        select(S.NORMAL)
        base=robot.get_actor_location();base_values=[base.x,base.y,base.z]
        try:
            robot.set_actor_location(unreal.Vector(base.x,base.y-10000,base.z),False,True)
            angles=list(robot.get_editor_property('current_angles_degrees'));targets=list(robot.get_editor_property('target_angles_degrees'))
            assert not cell.call_method('StartCycle') and 'IK cannot reach' in cell.get_editor_property('status')
            assert angles==list(robot.get_editor_property('current_angles_degrees')) and targets==list(robot.get_editor_property('target_angles_degrees'))
        finally:robot.set_actor_location(unreal.Vector(*base_values),False,True)
        report['checks'].append('IK preflight failure independently rejects start without changing servo angles or targets.')
        select(S.NORMAL)
        mount=cell.call_method('GetMountDoorWorld')
        for offset,expected in ((1.999,True),(2.001,False),(-1.999,True),(-2.001,False)):
            t=unreal.Transform();t.translation=unreal.Vector(mount.translation.x,mount.translation.y+offset/10,mount.translation.z);t.rotation=mount.rotation
            assert controller.call_method('SetDoorWorldPose',(t,))
            assert cell.call_method('RefreshQuality')
            assert cell.get_editor_property('quality_passed')==expected
        report['checks'].append('Both signed sides of the +/-2mm flush quality boundary classify correctly.')
        select(S.NORMAL);cell.set_editor_property('alignment_sensor_valid',False)
        assert not cell.call_method('StartCycle') and not cell.get_editor_property('door_held')
        select(S.NORMAL);cell.set_editor_property('verification_sensor_valid',False)
        assert cell.call_method('StartCycle')
        for _ in range(7200):
            if not cell.call_method('AdvanceSimulation',(1/60,)):break
        assert cell.call_method('GetStateLabel')=='Fault' and not cell.get_editor_property('door_fastened')
        report['checks'].append('Invalid alignment input rejects Start; invalid independent verification blocks fastening.')
        select(S.NORMAL)
        # A manual edit after Reset must be detected at Start rather than overwritten.
        t=door.get_world_transform();t.translation=unreal.Vector(t.translation.x+1,t.translation.y-1,t.translation.z)
        controller.call_method('SetDoorWorldPose',(t,));placed=pose(door.get_world_transform())
        assert cell.call_method('StartCycle');check_close(placed,pose(door.get_world_transform()))
        assert not cell.call_method('SelectScenario',(S.NORMAL,M.BASIC))
        cell.call_method('StopCycle');stop=pose(door.get_world_transform())
        for _ in range(30):assert cell.call_method('AdvanceSimulation',(1/60,))
        check_close(stop,pose(door.get_world_transform()))
        select(S.NORMAL);assert cell.call_method('StartCycle')
        t=door.get_world_transform();t.translation=unreal.Vector(t.translation.x+1,t.translation.y,t.translation.z)
        controller.call_method('SetDoorWorldPose',(t,))
        assert not cell.call_method('AdvanceSimulation',(1/60,))
        assert not cell.get_editor_property('door_held') and not cell.get_editor_property('door_fastened')
        report['checks'].append('Start preserves manually placed pose; active preset edits reject; Stop holds; external pickup motion faults.')
        select(S.NORMAL)
        check_close(reference,pose(controller.get_editor_property('reference_pose_in_body')))
        assert station_baselines==baselines()
        assert hashes=={str(p.relative_to(PROJECT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
        report['checks'].append('Normal door/inspection baselines and both map files remain unchanged.')
        report['success']=True
    except Exception:
        report['failure']=traceback.format_exc()
        raise
    finally:
        if cell:
            try:
                cell.call_method('StopCycle')
                cell.call_method('SelectScenario',(unreal.DoorCellScenario.NORMAL,unreal.DoorCellControlMode.INDEPENDENT_VERIFICATION))
            except Exception:report['cleanup_failure']=traceback.format_exc()
        REPORT.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    unreal.log('DOOR_CELL_EXPERIMENTS_SUCCESS')

if __name__=='__main__':main()
