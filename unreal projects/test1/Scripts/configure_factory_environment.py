"""Restore factory_p1's original lightweight lighting and editor viewpoints."""
from pathlib import Path
import hashlib,json,unreal

def configure():
 actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
 for a in actors.get_all_level_actors():
  if isinstance(a,(unreal.DirectionalLight,unreal.CameraActor,unreal.PlayerStart)) or a.get_actor_label()=='FP1_Exposure_NoLumen':
   assert actors.destroy_actor(a)
 def spawn(cls,label,pos,folder,rot=unreal.Rotator()):
  a=actors.spawn_actor_from_class(cls,unreal.Vector(*pos),rot)
  a.set_actor_label(label);a.set_folder_path('FactoryP1/'+folder)
  a.set_editor_property('tags',[unreal.Name('FactoryP1'),unreal.Name(label)])
  return a
 for i,(yaw,intensity) in enumerate(((35,3.5),(210,2.0),(115,1.0))):
  a=spawn(unreal.DirectionalLight,'FP1_Light_'+str(i),(3600,0,1000),'90_Environment',unreal.Rotator(pitch=-50,yaw=yaw))
  a.light_component.set_intensity(intensity);a.light_component.set_cast_shadows(False)
  a.light_component.set_editor_property('forward_shading_priority',10-i)
 pp=spawn(unreal.PostProcessVolume,'FP1_Exposure_NoLumen',(0,0,0),'90_Environment')
 pp.set_editor_property('unbound',True)
 s=pp.get_editor_property('settings')
 for key,value in {
  'override_dynamic_global_illumination_method':True,'dynamic_global_illumination_method':unreal.DynamicGlobalIlluminationMethod.NONE,
  'override_reflection_method':True,'reflection_method':unreal.ReflectionMethod.NONE,
  'override_auto_exposure_method':True,'auto_exposure_method':unreal.AutoExposureMethod.AEM_MANUAL,
  'override_auto_exposure_apply_physical_camera_exposure':True,'auto_exposure_apply_physical_camera_exposure':False,
  'override_auto_exposure_bias':True,'auto_exposure_bias':0.0,'override_motion_blur_amount':True,'motion_blur_amount':0.0
 }.items():s.set_editor_property(key,value)
 pp.set_editor_property('settings',s)
 pos=unreal.Vector(3600,5700,6200)
 cam=spawn(unreal.CameraActor,'FP1_View_Overview',(pos.x,pos.y,pos.z),'80_Views',unreal.MathLibrary.find_look_at_rotation(pos,unreal.Vector(3600,0,0)))
 cam.camera_component.set_field_of_view(58)
 cam.camera_component.set_editor_property('aspect_ratio',16/9)
 top=spawn(unreal.CameraActor,'FP1_View_Plan',(3600,0,10000),'80_Views',unreal.Rotator(pitch=-90,yaw=-90))
 top.camera_component.set_editor_property('projection_mode',unreal.CameraProjectionMode.ORTHOGRAPHIC)
 top.camera_component.set_editor_property('ortho_width',8200)
 top.camera_component.set_editor_property('aspect_ratio',16/9)
 start=unreal.Vector(3400,1600,900)
 spawn(unreal.PlayerStart,'FP1_FreeFly_Start',(start.x,start.y,start.z),'80_Views',unreal.MathLibrary.find_look_at_rotation(start,unreal.Vector(4200,0,100)))

if __name__=='__main__':
 p=Path(unreal.Paths.project_dir()).resolve();target=p/'Content/DigitalTwin/Maps/factory_p1.umap'
 others={f:hashlib.sha256(f.read_bytes()).hexdigest() for f in (p/'Content').rglob('*.umap') if f!=target}
 levels=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
 assert levels.load_level('/Game/DigitalTwin/Maps/factory_p1')
 runtime=next(a for a in unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors() if isinstance(a,unreal.FactoryRuntime))
 assert runtime.get_editor_property('use_assembly_integration')
 configure()
 assert levels.save_current_level()
 assert all(hashlib.sha256(f.read_bytes()).hexdigest()==h for f,h in others.items())
 (p/'Saved/FactoryAssemblyEnvironmentReport.json').write_text(json.dumps({'success':True,'other_maps_unchanged':True}),encoding='utf8')
