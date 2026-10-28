"""Replay seeded random events and verify measured recovery to completed cars."""
import csv
import hashlib
import json
import traceback
from pathlib import Path
import unreal

p=Path(unreal.Paths.project_dir()).resolve()
maps=list((p/'Content/DigitalTwin/Maps').glob('*.umap'))
hashes={f:hashlib.sha256(f.read_bytes()).hexdigest() for f in maps}
report={'success':False,'runs':[]}
try:
    levels=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    assert levels.load_level('/Game/DigitalTwin/Maps/AutonomousAssemblyLine')
    actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()
    line=next(a for a in actors if isinstance(a,unreal.AutonomousAssemblyLine))
    wheel=line.get_editor_property('wheel_cell')
    def stage():return str(line.get_editor_property('stage')).split('.')[-1].split(':')[0]
    for run in range(10):
        line.set_editor_property('base_seed',9300+run*37)
        assert line.call_method('ResetLine')
        for _ in range(6):line.call_method('AdvanceLine',(1/60,))
        assert line.call_method('StartLine')
        for _ in range(9000):
            ok=line.call_method('AdvanceLine',(1/30,))
            if not ok or stage()=='COMPLETE':break
        log=Path(line.get_editor_property('event_log'))
        rows=list(csv.DictReader(log.open(encoding='utf-8-sig')))
        events=[r['event'] for r in rows]
        fault_names=[r['reason'] for r in rows if r['event']=='RANDOM_EVENT']
        result={'seed':line.get_editor_property('current_seed'),'stage':stage(),
                'status':line.get_editor_property('status'),
                'injected':line.get_editor_property('injected_events'),
                'recovered':line.get_editor_property('recovered_events'),
                'wheel_count':wheel.get_editor_property('mounted_count'),
                'pickup_replans':wheel.get_editor_property('left_pickup_replans'),
                'grip_retries':wheel.get_editor_property('left_grip_retries'),
                'random_events':fault_names,
                'wait_decisions':[r['reason'] for r in rows if r['event']=='ROBOT_DECISION' and 'Wait' in r['reason']]}
        report['runs'].append(result)
        assert stage()=='COMPLETE' and result['wheel_count']==4,result
        assert result['injected']==2 and result['recovered']==2 and len(fault_names)==2,result
        assert 'DOCK_ACCEPTED' in events and 'WHEEL_START_COMMAND' in events,result
        if any('파지 실패' in name for name in fault_names):
            assert result['grip_retries']>=1,result
        if any('누락' in name or '센서 통신 끊김' in name for name in fault_names):
            assert result['wait_decisions'],result
    assert all(hashlib.sha256(f.read_bytes()).hexdigest()==h for f,h in hashes.items())
    report.update(success=True,maps_unchanged=True)
except Exception:
    report['failure']=traceback.format_exc()
    unreal.log_error(report['failure'])
finally:
    (p/'Saved/AutonomousAssemblyVerification.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
assert report['success'],report.get('failure')
