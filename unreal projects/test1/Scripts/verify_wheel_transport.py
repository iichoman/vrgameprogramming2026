"""Cold-load transport interlocks and full assembly, without saving the map."""
import hashlib
import json
import traceback
from pathlib import Path
import unreal

p = Path(unreal.Paths.project_dir()).resolve()
map_file = p/'Content/DigitalTwin/Maps/WheelAssemblyCell.umap'
baseline = hashlib.sha256(map_file.read_bytes()).hexdigest()
report = {'success': False, 'runs': []}
try:
    assert unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).load_level('/Game/DigitalTwin/Maps/WheelAssemblyCell')
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    cell = next(a for a in actors.get_all_level_actors() if a.get_class().get_name() == 'WheelAssemblyCell')
    assert cell.get_editor_property('enable_transport')
    car = cell.get_editor_property('assembly')
    carriage = cell.get_editor_property('transport_carriage')
    station = car.get_actor_transform()
    left = cell.get_editor_property('left_robot')
    right = cell.get_editor_property('right_robot')
    def tick(dt, seconds):
        for _ in range(round(seconds/dt)):
            cell.call_method('AdvanceSimulation', (dt,))
    def reset():
        assert cell.call_method('SelectScenario', (unreal.WheelScenario.NORMAL, unreal.WheelControlMode.INDEPENDENT_VERIFICATION))
    def pos(a):
        return a.get_actor_location()
    reset()
    start = pos(car)
    wheels = [cell.call_method('GetWheelWorld', (i,)).translation for i in range(4)]
    angles = [list(r.get_editor_property('current_angles_degrees')) for r in (left, right)]
    assert cell.call_method('StartCycle')
    tick(1/60, 4)
    assert (pos(car)-start).length() > 100, (str(pos(car)), str(start), cell.get_editor_property('status'), str(cell.get_editor_property('outcome')), cell.get_editor_property('transport_progress'))
    assert cell.get_editor_property('mounted_count') == 0
    assert [list(r.get_editor_property('current_angles_degrees')) for r in (left, right)] == angles
    for i in range(4):
        assert (cell.call_method('GetWheelWorld', (i,)).translation-wheels[i]).length() < 1e-6
    cell.call_method('StopCycle')
    stopped_car, stopped_carriage = pos(car), pos(carriage)
    tick(1/60, 2)
    assert (pos(car)-stopped_car).length() < 1e-6 and (pos(carriage)-stopped_carriage).length() < 1e-6
    assert not cell.call_method('StartCycle')
    reset()
    assert (pos(car)-start).length() < 1e-6
    for frame in (1/60, 1/30):
        reset()
        assert cell.call_method('StartCycle')
        for _ in range(round(120/frame)):
            cell.call_method('AdvanceSimulation', (frame,))
            outcome = str(cell.get_editor_property('outcome'))
            if 'NOT_EVALUATED' not in outcome:
                break
        result = {'frame': frame, 'outcome': outcome,
                  'mounted': cell.get_editor_property('mounted_count'),
                  'time': cell.get_editor_property('simulation_time_seconds'),
                  'errors_mm': list(cell.get_editor_property('mount_position_error_mm')),
                  'status': cell.get_editor_property('status')}
        report['runs'].append(result)
        assert 'PASS' in outcome and result['mounted'] == 4, result
        assert (pos(car)-station.translation).length() < 1e-3
    reset()
    assert cell.call_method('StartCycle')
    tick(1/60, 1)
    carriage.set_actor_location(pos(carriage)+unreal.Vector(0, 1, 0), False, True)
    tick(1/60, 0.1)
    assert 'BLOCKED' in str(cell.get_editor_property('outcome'))
    reset()
    for scenario in (unreal.WheelScenario.MISSING_WHEEL, unreal.WheelScenario.UNREACHABLE_WHEEL):
        assert cell.call_method('SelectScenario', (scenario, unreal.WheelControlMode.INDEPENDENT_VERIFICATION))
        before = pos(car)
        assert not cell.call_method('StartCycle')
        assert (pos(car)-before).length() < 1e-6
    assert hashlib.sha256(map_file.read_bytes()).hexdigest() == baseline
    report['success'] = True
except Exception:
    report['failure'] = traceback.format_exc()
    unreal.log_error(report['failure'])
finally:
    (p/'Saved/WheelTransportVerification.json').write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
assert report['success'], report.get('failure')

# Preserve all existing wheel success/failure scenario behavior as well.
import runpy
runpy.run_path(str(p/'Scripts/verify_wheel_assembly_cell.py'), run_name='__main__')
