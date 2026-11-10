"""Add a calibrated opposite-side door robot to the saved factory_p1 map."""
import json
import sys
from pathlib import Path
import unreal

p = Path(unreal.Paths.project_dir()).resolve()
sys.path.insert(0, str(p/'Scripts'))
import setup_door_assembly_cell as door

levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
assert levels.load_level('/Game/DigitalTwin/Maps/factory_p1')
all_actors = actors.get_all_level_actors()
line = next(a for a in all_actors if isinstance(a, unreal.IntegratedAssemblyLine))
assert line.get_editor_property('door_cell')
robot = next((a for a in all_actors if a.get_actor_label() == 'DoorStation_RightRobot'), None)
if robot is None:
    robot = actors.spawn_actor_from_class(unreal.RobotArm, unreal.Vector(1040,420,0))
    robot.set_actor_label('DoorStation_RightRobot')
    robot.set_actor_rotation(unreal.Rotator(yaw=-90), True)
    calibration = json.loads((p/'Scripts/robot_calibration.json').read_text(encoding='utf-8'))
    door.configure_robot(robot, calibration)
line.set_editor_property('right_door_robot', robot)
assert levels.save_current_level()
(p/'Saved/FourDoorStationSetup.json').write_text(json.dumps({
    'map': '/Game/DigitalTwin/Maps/factory_p1',
    'robot': robot.get_actor_label(),
    'door_ids': ['Door_FL','Door_RL','Door_FR','Door_RR']}, indent=2), encoding='utf-8')
