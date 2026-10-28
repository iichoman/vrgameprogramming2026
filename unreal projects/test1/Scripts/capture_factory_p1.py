"""Render the saved blockout in an unattended editor; never save or rebuild it."""
import json
import runpy
import time
import traceback
import uuid
from pathlib import Path
import unreal

project = Path(unreal.Paths.project_dir()).resolve()
assert '-FactoryP1VerifyOnly' in unreal.SystemLibrary.get_command_line()
runpy.run_path(str(project / 'Scripts/create_factory_p1.py'), run_name='__main__')
world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
for command in ('r.ScreenPercentage 100', 'r.AntiAliasingMethod 1'):
    unreal.SystemLibrary.execute_console_command(world, command)
out = project / 'Saved/FactoryP1Preview' / uuid.uuid4().hex[:10]
out.mkdir(parents=True, exist_ok=True)
unreal.EditorPythonScripting.set_keep_python_script_alive(True)
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
cameras = {a.get_actor_label(): a for a in actors.get_all_level_actors() if isinstance(a, unreal.CameraActor)}
jobs = [('FP1_View_Overview', 'Overview.png'), ('FP1_View_Plan', 'Plan.png')]
state = {'start': time.monotonic(), 'index': 0, 'task': None, 'results': [], 'ticks': 0}


def finish(error=None):
    unreal.unregister_slate_post_tick_callback(handle)
    report = {'success': error is None and len(state['results']) == len(jobs), 'captures': state['results'], 'error': error}
    (out / 'render_report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    (project / 'Saved/FactoryP1RenderReport.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    unreal.EditorPythonScripting.set_keep_python_script_alive(False)


def tick(delta):
    try:
        state['ticks'] += 1
        if time.monotonic() - state['start'] > 240:
            finish('Screenshot timeout')
            return
        if state['ticks'] < 45:
            return
        if state['task'] is not None:
            if not state['task'].is_task_done():
                return
            camera, filename = jobs[state['index']]
            path = out / filename
            if not path.is_file():
                return
            state['results'].append({'camera': camera, 'file': str(path), 'bytes': path.stat().st_size})
            state['task'] = None
            state['index'] += 1
        if state['index'] == len(jobs):
            finish()
            return
        unreal.AutomationLibrary.finish_loading_before_screenshot()
        camera, filename = jobs[state['index']]
        state['task'] = unreal.AutomationLibrary.take_high_res_screenshot(
            1600, 900, str(out / filename), camera=cameras[camera], delay=1.0, force_game_view=True)
    except Exception:
        finish(traceback.format_exc())


handle = unreal.register_slate_post_tick_callback(tick)
