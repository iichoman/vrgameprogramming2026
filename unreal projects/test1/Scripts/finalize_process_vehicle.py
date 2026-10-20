"""Improve the new test map's overview and resolve the directional-light priority warning."""
import unreal

levels=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
assert levels.load_level('/Game/DigitalTwin/Maps/CAR_MODULE_TEST')
all_actors=actors.get_all_level_actors()
assert sum(isinstance(a,unreal.VehicleProcessModule) for a in all_actors)==42
pos=unreal.Vector(340,-420,650)
rot=unreal.MathLibrary.find_look_at_rotation(pos,unreal.Vector(0,0,60))
for actor in all_actors:
    if isinstance(actor,unreal.DirectionalLight):
        actor.light_component.set_editor_property('forward_shading_priority',10-int(actor.get_actor_label().rsplit('_',1)[1]))
    if isinstance(actor,unreal.PlayerStart) or actor.get_actor_label()=='View_01_WholeVehicle':
        actor.set_actor_location_and_rotation(pos,rot,False,True)
        if isinstance(actor,unreal.CameraActor): actor.camera_component.set_field_of_view(70)
unreal.EditorLevelLibrary.set_level_viewport_camera_info(pos,rot)
assert levels.save_current_level()
unreal.log('PROCESS_MODULES_VIEW_FINALIZED')
