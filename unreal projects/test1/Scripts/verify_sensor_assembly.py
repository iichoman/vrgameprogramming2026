"""Observe sensor-gated starts and physically induced faults in the saved map."""
import csv
import hashlib
import json
import traceback
from pathlib import Path
import unreal

p=Path(unreal.Paths.project_dir()).resolve()
report={'success':False,'cases':[]}
files=list((p/'Content/DigitalTwin/Maps').glob('*.umap'))
hashes={f:hashlib.sha256(f.read_bytes()).hexdigest() for f in files}
try:
    assert unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).load_level('/Game/DigitalTwin/Maps/SensorAssemblyLine')
    api=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    line=next(a for a in api.get_all_level_actors() if isinstance(a,unreal.SensorAssemblyLine))
    wheel=line.get_editor_property('wheel_cell')
    car=wheel.get_editor_property('assembly')
    robots=[wheel.get_editor_property(n) for n in ('left_robot','right_robot')]
    sensors=line.get_editor_property('sensors')
    def phase():return str(line.get_editor_property('arrival_phase')).split('.')[-1].split(':')[0]
    def stage():return str(line.get_editor_property('stage')).split('.')[-1].split(':')[0]
    def step(dt=1/60):return line.call_method('AdvanceLine',(dt,))
    def reset(demo):
        assert line.call_method('ResetLine')
        assert line.call_method('SelectDemo',(demo,))
        assert not line.call_method('StartLine') # speed has not yet been sampled
        for _ in range(6):step()
    def run(demo,dt=1/60):
        reset(demo)
        initial=[list(r.get_editor_property('current_angles_degrees')) for r in robots]
        assert line.call_method('StartLine')
        for _ in range(round(120/dt)):
            ok=step(dt)
            if phase()!='RELEASED':
                assert wheel.get_editor_property('mounted_count')==0
                assert [list(r.get_editor_property('current_angles_degrees')) for r in robots]==initial
            if not ok or stage()=='COMPLETE':break
        for _ in range(90):step(dt) # allow the commanded stop to decelerate
        log=Path(line.get_editor_property('event_log'))
        rows=list(csv.DictReader(log.open(encoding='utf-8-sig')))
        result={'demo':str(demo),'dt':dt,'phase':phase(),'stage':stage(),
                'status':line.get_editor_property('status'),'log':str(log),
                'left_pickup_replans':wheel.get_editor_property('left_pickup_replans'),
                'left_grip_retries':wheel.get_editor_property('left_grip_retries'),
                'events':[r['event'] for r in rows if r['event']!='SAMPLE']}
        report['cases'].append(result)
        if demo in (unreal.ArrivalDemo.NORMAL,unreal.ArrivalDemo.WHEEL_DISPLACED):
            assert stage()=='COMPLETE',result
            assert 'DOCK_ACCEPTED' in result['events'] and 'WHEEL_START_COMMAND' in result['events']
            assert wheel.get_editor_property('mounted_count')==4
            if demo==unreal.ArrivalDemo.WHEEL_DISPLACED:
                assert result['left_pickup_replans']>=1,result
        elif demo in (unreal.ArrivalDemo.WHEEL_MISSING,unreal.ArrivalDemo.GRIP_FAILURE,unreal.ArrivalDemo.WHEEL_POSE_DROPOUT):
            assert phase()=='RELEASED' and stage()=='FAULT',result
            assert 'WHEEL_START_COMMAND' in result['events'] and 'ASSEMBLY_RESULT' in result['events']
            assert wheel.get_editor_property('mounted_count')==0,result
            if demo==unreal.ArrivalDemo.GRIP_FAILURE:
                assert result['left_grip_retries']==1,result
        else:
            assert phase()=='FAULT' and 'WHEEL_START_COMMAND' not in result['events'],result
            assert 'FAULT_STOP_COMMAND' in result['events'] and 'STOP_CONFIRMED' in result['events']
            assert not line.call_method('StartLine')
        return result
    run(unreal.ArrivalDemo.NORMAL)
    run(unreal.ArrivalDemo.NORMAL,1/30)
    for demo in (unreal.ArrivalDemo.JAM,unreal.ArrivalDemo.MIS_DOCK,unreal.ArrivalDemo.POSITION_DROPOUT):run(demo)
    for demo in (unreal.ArrivalDemo.WHEEL_MISSING,unreal.ArrivalDemo.WHEEL_DISPLACED,
                 unreal.ArrivalDemo.GRIP_FAILURE,unreal.ArrivalDemo.WHEEL_POSE_DROPOUT):run(demo)
    # With Normal selected, remove the wheel directly after docking. The arm
    # must react to its measured presence, not the selected demo identifier.
    reset(unreal.ArrivalDemo.NORMAL)
    assert line.call_method('StartLine')
    for _ in range(1200):
        step()
        if phase()=='RELEASED':break
    assert phase()=='RELEASED'
    wheel.call_method('InjectLeftWheelMissing')
    for _ in range(120):
        if not step():break
    assert stage()=='FAULT' and wheel.get_editor_property('mounted_count')==0
    report['world_wheel_missing_detected']=True
    # Independently alter the world with Normal selected: controller must still detect it.
    reset(unreal.ArrivalDemo.NORMAL)
    obstacle=api.spawn_actor_from_class(unreal.StaticMeshActor,unreal.Vector())
    obstacle.set_editor_property('tags',[unreal.Name('SensorWorkpiece')])
    for _ in range(6):step()
    assert not line.call_method('StartLine')
    api.destroy_actor(obstacle)
    for _ in range(6):step()
    assert line.call_method('StartLine')
    for _ in range(120):step()
    line.call_method('StopLine')
    stopped_at=car.get_actor_location()
    for _ in range(120):step()
    settled=car.get_actor_location()
    assert (settled-stopped_at).length()>0 # deceleration is modeled
    for _ in range(60):step()
    assert (car.get_actor_location()-settled).length()<1e-5
    assert not line.call_method('StartLine')
    # Switch can be true while position remains wrong; no timed release.
    reset(unreal.ArrivalDemo.NORMAL)
    assert line.call_method('StartLine')
    for _ in range(900):
        step()
        if phase()=='DOCKING':break
    assert phase()=='DOCKING'
    car.set_actor_location(car.get_actor_location()+unreal.Vector(0,1,0),False,True)
    for _ in range(1000):
        if not step():break
    assert phase()=='FAULT' and wheel.get_editor_property('mounted_count')==0
    assert all(hashlib.sha256(f.read_bytes()).hexdigest()==h for f,h in hashes.items())
    report.update(success=True,occupied_zone_block=True,operator_deceleration=True,world_fault_detected=True,maps_unchanged=True)
except Exception:
    report['failure']=traceback.format_exc()
    unreal.log_error(report['failure'])
finally:
    (p/'Saved/SensorAssemblyVerification.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
assert report['success'],report.get('failure')
