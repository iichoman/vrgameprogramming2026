"""Cold-load line, retained module geometry, sequencing, Stop/Reset and fault gates."""
import hashlib
import json
import traceback
from pathlib import Path
import unreal

P=Path(unreal.Paths.project_dir()).resolve()
paths=list((P/'Content/DigitalTwin/Maps').glob('*.umap'))
hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
report={'success':False,'runs':[]}
try:
    assert unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).load_level('/Game/DigitalTwin/Maps/IntegratedAssemblyLine')
    actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()
    line=next(a for a in actors if isinstance(a,unreal.IntegratedAssemblyLine))
    wheel=line.get_editor_property('wheel_cell')
    door=line.get_editor_property('door_cell')
    car=wheel.get_editor_property('assembly')
    modules={str(a.get_editor_property('module_id')):a for a in actors if isinstance(a,unreal.VehicleProcessModule)}
    assert len(modules)==42
    assert all(not m.get_editor_property('wheel_motion_enabled') for m in modules.values())
    assert all(m.call_method('GetVisualCount')>0 for m in modules.values())
    assert not any(c.static_mesh for c in car.get_components_by_class(unreal.StaticMeshComponent))
    assert not any(c.static_mesh for c in wheel.get_components_by_class(unreal.StaticMeshComponent))
    def stage(): return str(line.get_editor_property('stage')).split('.')[-1].split(':')[0]
    def reset(): assert line.call_method('ResetLine'),line.get_editor_property('status')
    def step(dt=1/60):
        ok=line.call_method('AdvanceLine',(dt,))
        assert ok,(stage(),line.get_editor_property('status'))
    def until(target):
        for _ in range(15000):
            if stage()==target:return
            step()
        raise AssertionError(('timeout',target,stage()))
    for dt in (1/60,1/30):
        reset()
        body_start=car.get_actor_location()
        door_supply=modules['Door_FL'].get_actor_transform()
        wheel_pickups=[modules[k].get_actor_transform() for k in ('Wheel_FL','Wheel_RL','Wheel_FR','Wheel_RR')]
        assert line.call_method('StartLine')
        for _ in range(round(4/dt)):step(dt)
        assert (car.get_actor_location()-body_start).length()>100
        assert (modules['Door_FL'].get_actor_location()-door_supply.translation).length()<.001
        for i,k in enumerate(('Wheel_FL','Wheel_RL','Wheel_FR','Wheel_RR')):
            assert (modules[k].get_actor_location()-wheel_pickups[i].translation).length()<.001
        stages=[stage()]
        for _ in range(round(180/dt)):
            step(dt)
            if stage()!=stages[-1]:stages.append(stage())
            if stage()=='COMPLETE':break
        result={'dt':dt,'stages':stages,'time':line.get_editor_property('elapsed_seconds'),
                'wheel_count':wheel.get_editor_property('mounted_count'),'door_fastened':door.get_editor_property('door_fastened'),
                'wheel_errors_mm':list(wheel.get_editor_property('mount_position_error_mm')),
                'door_quality':door.get_editor_property('quality_passed')}
        report['runs'].append(result)
        assert stages==['WHEELS','TRANSFER','DOOR','COMPLETE'],result
        assert result['wheel_count']==4 and result['door_fastened'] and result['door_quality']
        for i,k in enumerate(('Wheel_FL','Wheel_RL','Wheel_FR','Wheel_RR')):
            assert (modules[k].get_actor_location()-wheel.call_method('GetMountWorld',(i,)).translation).length()<.2
        assert abs(car.get_actor_location().x-1000)<.001
    # Stop preserves all parts in every moving phase; reset returns the same start.
    for target in ('WHEELS','TRANSFER','DOOR'):
        reset(); assert line.call_method('StartLine'); until(target)
        for _ in range(120):step()
        line.call_method('StopLine')
        before={k:m.get_actor_location() for k,m in modules.items()}
        for _ in range(60):step()
        assert all((modules[k].get_actor_location()-v).length()<1e-6 for k,v in before.items())
        assert not line.call_method('StartLine')
        reset(); assert abs(car.get_actor_location().x+600)<.001
    # Wheel sensor failure blocks before moving; door failure prevents completion.
    wheel.set_editor_property('alignment_sensor_valid',False)
    assert not line.call_method('StartLine') and stage()=='FAULT'
    wheel.set_editor_property('alignment_sensor_valid',True)
    reset(); assert line.call_method('StartLine'); until('TRANSFER')
    car.set_actor_location(car.get_actor_location()+unreal.Vector(0,1,0),False,True)
    assert not line.call_method('AdvanceLine',(1/60,)) and stage()=='FAULT'
    reset(); door.set_editor_property('alignment_sensor_valid',False)
    assert line.call_method('StartLine'); until('TRANSFER')
    for _ in range(1000):
        if not line.call_method('AdvanceLine',(1/60,)):break
    assert stage()=='FAULT' and not door.get_editor_property('door_fastened')
    assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    report.update(success=True,module_count=42,stop_reset_all_phases=True,fault_gates=True,map_hashes_unchanged=True)
except Exception:
    report['failure']=traceback.format_exc()
    unreal.log_error(report['failure'])
finally:
    (P/'Saved/IntegratedAssemblyVerification.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
assert report['success'],report.get('failure')
