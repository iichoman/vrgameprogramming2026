"""Cold-load and exercise the saved wheel scene. Read-only for map assets."""
import hashlib
import json
import math
import traceback
from pathlib import Path
import unreal

PROJECT = Path(unreal.Paths.project_dir()).resolve()
MAP = '/Game/DigitalTwin/Maps/WheelAssemblyCell'
REPORT = PROJECT / 'Saved/WheelAssemblyVerification.json'


def enum(value):
    return str(value).split('.')[-1].split(':')[0].strip('<> ')


def main():
    report = {'success': False, 'runs': []}
    map_path = PROJECT / 'Content/DigitalTwin/Maps/WheelAssemblyCell.umap'
    baseline = hashlib.sha256(map_path.read_bytes()).hexdigest()
    try:
        assert unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).load_level(MAP)
        actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()
        cls = unreal.load_class(None, '/Script/test1.WheelAssemblyCell')
        cells = [a for a in actors if a.get_class() == cls]
        assert len(cells) == 1
        cell = cells[0]
        left, right = cell.get_editor_property('left_robot'), cell.get_editor_property('right_robot')
        assert left and right and left != right and cell.get_editor_property('assembly')
        S, M = unreal.WheelScenario, unreal.WheelControlMode

        def run(scenario, mode=M.INDEPENDENT_VERIFICATION, frame=1 / 60):
            assert cell.call_method('SelectScenario', (scenario, mode)), cell.get_editor_property('status')
            started = cell.call_method('StartCycle')
            stages = []
            if started:
                for _ in range(math.ceil(120 / frame)):
                    ok = cell.call_method('AdvanceSimulation', (frame,))
                    outcome = enum(cell.get_editor_property('outcome'))
                    pair = [cell.call_method('GetLaneStage', (True,)), cell.call_method('GetLaneStage', (False,))]
                    if not stages or stages[-1] != pair:
                        stages.append(pair)
                    if outcome != 'NOT_EVALUATED':
                        break
                    assert ok, cell.get_editor_property('status')
            result = {
                'scenario': enum(scenario), 'mode': enum(mode), 'frame': frame,
                'started': started, 'outcome': enum(cell.get_editor_property('outcome')),
                'mounted': cell.get_editor_property('mounted_count'),
                'time_s': cell.get_editor_property('simulation_time_seconds'),
                'position_mm': list(cell.get_editor_property('mount_position_error_mm')),
                'orientation_deg': list(cell.get_editor_property('mount_orientation_error_deg')),
                'fastened': list(cell.get_editor_property('fastened_wheels')),
                'left_stage': cell.call_method('GetLaneStage', (True,)),
                'right_stage': cell.call_method('GetLaneStage', (False,)),
                'status': cell.get_editor_property('status'),
                'stages': stages,
                'left_angles': list(left.get_editor_property('current_angles_degrees')),
                'left_targets': list(left.get_editor_property('target_angles_degrees')),
            }
            report['runs'].append(result)
            return result

        normal = run(S.NORMAL)
        assert normal['outcome'] == 'PASS' and normal['mounted'] == 4 and all(normal['fastened']), normal
        normal30 = run(S.NORMAL, frame=1 / 30)
        assert normal30['outcome'] == 'PASS' and max(abs(a - b) for a, b in zip(normal['position_mm'], normal30['position_mm'])) < 1e-3
        basic = run(S.SENSOR_BIAS, M.BASIC)
        assert basic['outcome'] == 'QUALITY_FAIL' and basic['mounted'] == 4, basic
        verify = run(S.SENSOR_BIAS)
        assert verify['outcome'] == 'BLOCKED' and verify['mounted'] < 4, verify
        displaced = run(S.DISPLACED_WHEEL)
        assert displaced['outcome'] == 'PASS', displaced
        for scenario in (S.UNREACHABLE_WHEEL, S.REVERSED_WHEEL, S.MISSING_WHEEL):
            blocked = run(scenario)
            assert blocked['outcome'] == 'BLOCKED' and blocked['mounted'] == 0 and not blocked['started'], blocked
        assert hashlib.sha256(map_path.read_bytes()).hexdigest() == baseline, 'Verification modified the map.'
        report['success'] = True
    except Exception:
        report['failure'] = traceback.format_exc()
        unreal.log_error(report['failure'])
    finally:
        REPORT.parent.mkdir(parents=True, exist_ok=True)
        REPORT.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    assert report['success'], report.get('failure')


if __name__ == '__main__':
    main()
