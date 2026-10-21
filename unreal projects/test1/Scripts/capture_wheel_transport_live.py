"""Ticked editor capture: run with -ExecutePythonScript, not a commandlet."""
from pathlib import Path
import unreal
unreal.EditorPythonScripting.set_keep_python_script_alive(True)

p = Path(unreal.Paths.project_dir()).resolve()
assert unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).load_level('/Game/DigitalTwin/Maps/WheelAssemblyCell')
api = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
cell = next(a for a in api.get_all_level_actors() if a.get_class().get_name() == 'WheelAssemblyCell')
world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
camera = api.spawn_actor_from_class(unreal.CameraActor, unreal.Vector(900, -1350, 850))
rotation = unreal.MathLibrary.find_look_at_rotation(camera.get_actor_location(), unreal.Vector(-280, 0, 50))
camera.set_actor_rotation(rotation, True)
unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).set_level_viewport_camera_info(camera.get_actor_location(), rotation)
assert cell.call_method('ResetCycle')
frame = 0
busy = False
tasks = []
out = p/'Saved/TransportPreview'
out.mkdir(parents=True, exist_ok=True)
def step(dt):
    global frame, handle
    frame += 1
    if frame == 60:
        tasks.append(unreal.AutomationLibrary.take_high_res_screenshot(1440, 900, str(out/'01_waiting.png'), camera))
    elif frame == 80:
        assert cell.call_method('StartCycle')
        for _ in range(240):
            cell.call_method('AdvanceSimulation', (1/60,))
    elif frame == 100:
        tasks.append(unreal.AutomationLibrary.take_high_res_screenshot(1440, 900, str(out/'02_transport.png'), camera))
    elif frame == 120:
        for _ in range(300):
            cell.call_method('AdvanceSimulation', (1/60,))
    elif frame == 140:
        tasks.append(unreal.AutomationLibrary.take_high_res_screenshot(1440, 900, str(out/'03_docked.png'), camera))
    elif frame > 165:
        unreal.unregister_slate_post_tick_callback(handle)
        unreal.SystemLibrary.quit_editor()
def tick(dt):
    global busy
    if busy:
        return  # Loading a screenshot pumps Slate recursively.
    busy = True
    try:
        step(dt)
    finally:
        busy = False
handle = unreal.register_slate_post_tick_callback(tick)
