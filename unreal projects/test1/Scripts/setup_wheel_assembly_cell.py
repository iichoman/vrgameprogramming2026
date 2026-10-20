"""Create an independent four-wheel cell map from the saved Camry preview.

Run in UnrealEditor-Cmd after the native module builds. Never changes the door map.
The cylindrical tire and hub visuals are replaceable placeholders.
"""
import importlib.util
import json
import math
import traceback
from pathlib import Path
import unreal

PROJECT = Path(unreal.Paths.project_dir()).resolve()
SOURCE = '/Game/DigitalTwin/Maps/CamryAssemblyPreview'
MAP = '/Game/DigitalTwin/Maps/WheelAssemblyCell'
REPORT = PROJECT / 'Saved/WheelAssemblySetup.json'


def load_door_setup():
    spec = importlib.util.spec_from_file_location('door_setup', PROJECT / 'Scripts/setup_door_assembly_cell.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def quaternion_for(side, twist):
    half = math.sqrt(0.5)
    roll = [half if side == 0 else -half, 0, 0, half]
    angle = math.radians(twist) / 2
    return door.qm([0, math.sin(angle), 0, math.cos(angle)], roll)


def configure_visual_materials(cell):
    # Existing engine material has a color parameter; persist dedicated instances
    # so the visible placeholders read as tire rubber and silver wheel hubs.
    base = unreal.load_asset('/Engine/BasicShapes/BasicShapeMaterial')
    if not base:
        return {'material': 'engine default'}
    assets = unreal.AssetToolsHelpers.get_asset_tools()
    result = {}
    for name, color in [('MI_WheelTire', unreal.LinearColor(0.035, 0.039, 0.045, 1)),
                        ('MI_WheelHub', unreal.LinearColor(0.52, 0.57, 0.61, 1))]:
        path = '/Game/DigitalTwin/Wheels/' + name
        material = unreal.load_asset(path)
        if not material:
            material = assets.create_asset(name, '/Game/DigitalTwin/Wheels', unreal.MaterialInstanceConstant,
                                           unreal.MaterialInstanceConstantFactoryNew())
        material.set_editor_property('parent', base)
        unreal.MaterialEditingLibrary.set_material_instance_vector_parameter_value(material, 'Color', color)
        unreal.EditorAssetLibrary.save_loaded_asset(material)
        result[name] = material
    for component in cell.get_components_by_class(unreal.StaticMeshComponent):
        name = component.get_name()
        if name.startswith('Tire'):
            component.set_material(0, result['MI_WheelTire'])
        elif name.startswith('WheelHub'):
            component.set_material(0, result['MI_WheelHub'])
    return {'material': 'dedicated engine material instances'}


def preflight(robot, cell, side):
    # Find one tool orientation valid at both stations. A joint-space transit
    # connects the two independently reachable seven-pose wheel operations.
    groups = []
    for index in [side * 2, side * 2 + 1]:
        pickup = cell.call_method('GetPickupWorld', (index,)).translation
        mount = cell.call_method('GetMountWorld', (index,)).translation
        direction = -1 if side == 0 else 1
        def shifted(point, outward=0, up=0):
            return [point.x, point.y + direction * outward, point.z + up]
        groups.append([
            shifted(pickup, 25 + 12), shifted(pickup, 12), shifted(pickup, 12, 25),
            shifted(mount, 55 + 12, 25), shifted(mount, 25 + 12), shifted(mount, 12), shifted(mount, 25 + 12)
        ])
    seed_sets = [[0, 40, -65, 0, 25, 0], [0, -40, 65, 0, -25, 0],
                 [0, 60, -40, 0, -20, 0], [0, 0, 0, 0, 0, 0]]
    failures = []
    for twist in [0, 45, -45, 90, -90, 180]:
        q = quaternion_for(side, twist)
        solved = []
        for station, targets in enumerate(groups):
            found = None
            for seed in seed_sets:
                robot.call_method('SetInitialAngles', (seed,))
                poses = []
                for position in targets:
                    target = door.transform({'translation_cm': position, 'quaternion_xyzw': q})
                    if not robot.call_method('CommandTCPWorld', (target,)):
                        failures.append({'twist': twist, 'station': station, 'seed': seed,
                                         'waypoint': len(poses), 'status': str(robot.get_editor_property('last_status'))})
                        break
                    solution = list(robot.get_editor_property('target_angles_degrees'))
                    poses.append(solution)
                    robot.call_method('SetInitialAngles', (solution,))
                if len(poses) == len(targets):
                    found = {'seed': seed, 'poses': poses}
                    break
            if not found:
                break
            solved.append(found)
        if len(solved) == 2:
            robot.call_method('SetInitialAngles', (solved[0]['poses'][0],))
            transform = door.transform({'translation_cm': [0, direction * 12, 0], 'quaternion_xyzw': q})
            cell.set_editor_property('left_tcp_in_wheel' if side == 0 else 'right_tcp_in_wheel', transform)
            cell.set_editor_property('left_rear_approach_angles_degrees' if side == 0 else 'right_rear_approach_angles_degrees',
                                     solved[1]['poses'][0])
            return {'twist_degrees': twist, 'front_seed': solved[0]['seed'], 'rear_seed': solved[1]['seed'],
                    'initial_angles_deg': solved[0]['poses'][0], 'rear_approach_angles_deg': solved[1]['poses'][0],
                    'waypoints_per_wheel': len(groups[0]), 'failures_before_solution': len(failures)}
    raise AssertionError(('No common six-axis IK orientation for wheel lane', side, failures[-12:]))


def main():
    global door
    door = load_door_setup()
    report = {'map': MAP, 'source': SOURCE, 'saved': False}
    try:
        levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
        if not unreal.EditorAssetLibrary.does_asset_exist(MAP):
            assert levels.new_level_from_template(MAP, SOURCE)
        else:
            assert levels.load_level(MAP)
        actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
        assembly_class = unreal.load_class(None, '/Script/test1.CamryAssembly')
        assembly = next(a for a in actors.get_all_level_actors() if a.get_class() == assembly_class)
        for actor in list(actors.get_all_level_actors()):
            if actor.get_class().get_name() in ('DoorFixture', 'DoorAssemblyCell', 'RobotArm'):
                actors.destroy_actor(actor)
        calibration = json.loads((PROJECT / 'Scripts/robot_calibration.json').read_text(encoding='utf-8'))
        robots = []
        for side, label in [(0, 'LeftWheelRobot'), (1, 'RightWheelRobot')]:
            cls = unreal.load_class(None, '/Script/test1.RobotArm')
            robot = actors.spawn_actor_from_class(cls, unreal.Vector(0, 0, 0))
            robot.set_actor_label(label)
            robot.set_actor_location(unreal.Vector(0, -405 if side == 0 else 405, 0), False, True)
            robot.set_actor_rotation(unreal.Rotator(pitch=0, yaw=90 if side == 0 else -90, roll=0), True)
            door.configure_robot(robot, calibration)
            robots.append(robot)
        cls = unreal.load_class(None, '/Script/test1.WheelAssemblyCell')
        assert cls, 'WheelAssemblyCell native reflection unavailable; build the editor module.'
        cells = [a for a in actors.get_all_level_actors() if a.get_class() == cls]
        cell = cells[0] if cells else actors.spawn_actor_from_class(cls, unreal.Vector(0, 0, 0))
        cell.set_actor_label('WheelAssemblyCell')
        cell.set_editor_property('assembly', assembly)
        cell.set_editor_property('left_robot', robots[0])
        cell.set_editor_property('right_robot', robots[1])
        report['visuals'] = configure_visual_materials(cell)
        report['ik'] = []
        for side, robot in enumerate(robots):
            report['ik'].append(preflight(robot, cell, side))
        assert cell.call_method('ResetCycle'), cell.get_editor_property('status')
        game_mode = unreal.load_class(None, '/Script/test1.WheelAssemblyGameMode')
        assert game_mode
        world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
        world.get_world_settings().set_editor_property('default_game_mode', game_mode)
        # Match the door-cell light fix for the preview template.
        directional = []
        for actor in actors.get_all_level_actors():
            directional.extend(actor.get_components_by_class(unreal.DirectionalLightComponent))
        for index, light in enumerate(sorted(directional, key=lambda c: c.get_path_name())):
            light.set_editor_property('forward_shading_priority', len(directional) - index)
        assert levels.save_current_level()
        report['saved'] = True
        report['success'] = True
        unreal.log('WHEEL_ASSEMBLY_SETUP_SUCCESS')
    except Exception:
        report['success'] = False
        report['failure'] = traceback.format_exc()
        unreal.log_error(report['failure'])
    finally:
        REPORT.parent.mkdir(parents=True, exist_ok=True)
        REPORT.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    assert report['success'], report.get('failure')


if __name__ == '__main__':
    main()
