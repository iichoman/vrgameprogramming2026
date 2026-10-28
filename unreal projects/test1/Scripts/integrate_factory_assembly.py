"""Merge factory settings into a copy of the complete team scene (all references retained)."""
from pathlib import Path
from datetime import datetime
import hashlib,json,shutil,sys,unreal
p=Path(unreal.Paths.project_dir()).resolve()
target=p/'Content/DigitalTwin/Maps/factory_p1.umap'
backup=p/'Saved/FactoryRuntimeBackups'/datetime.now().strftime('assembly_integration_%Y%m%d_%H%M%S')
backup.mkdir(parents=True,exist_ok=True)
shutil.copy2(target,backup/target.name)
other={f:hashlib.sha256(f.read_bytes()).hexdigest() for f in (p/'Content').rglob('*.umap') if f!=target}
levels=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
assert levels.load_level('/Game/DigitalTwin/Maps/factory_p1')
old=next(a for a in actors.get_all_level_actors() if isinstance(a,unreal.FactoryRuntime))
assert not old.get_editor_property('use_assembly_integration'),'Already integrated; preserve user changes.'
original=json.loads(old.get_editor_property('initial_layout_json'))
(backup/'previous_layout.json').write_text(json.dumps(original,ensure_ascii=False,indent=2),encoding='utf8')
mesh_settings={name:[(v.get_editor_property('mesh').get_path_name(),v.get_editor_property('preview_visible')) for v in old.get_editor_property(name)] for name in ('body_meshes','wheel_meshes')}
pivot=old.get_editor_property('wheel_pivot_cm');pivot_values=(pivot.x,pivot.y,pivot.z)
overrides=[(str(v.get_editor_property('equipment_kind')),v.get_editor_property('mesh').get_path_name() if v.get_editor_property('mesh') else None) for v in old.get_editor_property('visual_overrides')]
extras=[v.get_path_name() for v in old.get_editor_property('extra_equipment')]
old=None
assert levels.load_level('/Game/DigitalTwin/Maps/AutonomousAssemblyLine')
unreal.log('ASSEMBLY_INSTALL_SOURCE_LOADED')
world=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
source=actors.get_all_level_actors()
line=next(a for a in source if isinstance(a,unreal.AutonomousAssemblyLine))
wheel=line.get_editor_property('wheel_cell');door=line.get_editor_property('door_cell')
car=wheel.get_editor_property('assembly')
assert door.get_editor_property('assembly')==car
modules=[a for a in source if isinstance(a,unreal.VehicleProcessModule)]
assert len(modules)==42
assert all(a.get_editor_property('vehicle_frame')==car for a in modules)
assert all(a.get_attach_parent_actor() is not None for a in modules)
for a in source:
 if isinstance(a,unreal.StaticMeshActor) and 'floor' in a.get_actor_label().lower():
  assert actors.destroy_actor(a)
 elif a.get_class().get_path_name().startswith('/Script/test1.'):
  a.set_folder_path('통합 테스트 (Assembly Integration)')
runtime=actors.spawn_actor_from_class(unreal.FactoryRuntime,unreal.Vector())
runtime.set_actor_label('FactoryRuntime')
for name,entries in mesh_settings.items():
 parts=[]
 for path,visible in entries:
  part=unreal.VehicleModuleMesh()
  part.set_editor_property('mesh',unreal.load_asset(path));part.set_editor_property('preview_visible',visible)
  parts.append(part)
 runtime.set_editor_property(name,parts)
runtime.set_editor_property('wheel_pivot_cm',unreal.Vector(*pivot_values))
vs=[]
for kind,path in overrides:
 v=unreal.FactoryVisualOverride();v.set_editor_property('equipment_kind',unreal.Name(kind));v.set_editor_property('mesh',unreal.load_asset(path) if path else None);vs.append(v)
runtime.set_editor_property('visual_overrides',vs)
runtime.set_editor_property('extra_equipment',[unreal.load_asset(path) for path in extras])
unreal.log('ASSEMBLY_INSTALL_RUNTIME_CONFIGURED')
line.set_actor_tick_enabled(False)
runtime.set_editor_property('test_assembly_line',line)
runtime.set_editor_property('use_assembly_integration',True)
original['items']=[i for i in original['items'] if i['kind']=='zone']
original['job']={'station_id':'','supply_id':'','handler_id':''}
original['hall']['origin_cm']=[-1200,0]
original['hall']['length_cm']=8400
runtime.set_editor_property('initial_layout_json',json.dumps(original,ensure_ascii=False))
runtime.call_method('BuildPreview')
assert runtime.call_method('VerifySetup')
assert line.call_method('ResetLine'),line.get_editor_property('status')
world.get_world_settings().set_editor_property('default_game_mode',unreal.GameModeBase)
sys.path.insert(0,str(p/'Scripts'))
from configure_factory_environment import configure
configure()
eye=unreal.Vector(400,-2200,1600)
unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).set_level_viewport_camera_info(eye,unreal.MathLibrary.find_look_at_rotation(eye,unreal.Vector(400,0,70)))
# Save As copies the world and its internal references; never save the source package.
assert unreal.EditorLoadingAndSavingUtils.save_map(world,'/Game/DigitalTwin/Maps/factory_p1')
assert all(hashlib.sha256(f.read_bytes()).hexdigest()==h for f,h in other.items())
report={'success':True,'backup':str(backup),'modules':42,'source_maps_unchanged':True,'map':'/Game/DigitalTwin/Maps/factory_p1'}
(p/'Saved/FactoryAssemblyInstallReport.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
unreal.log('FACTORY_ASSEMBLY_INTEGRATED '+str(report))
