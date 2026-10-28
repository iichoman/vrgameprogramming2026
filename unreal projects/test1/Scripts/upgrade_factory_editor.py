import unreal, json, shutil, hashlib
from pathlib import Path
from datetime import datetime
project=Path(unreal.Paths.project_dir()).resolve()
mapfile=project/'Content/DigitalTwin/Maps/factory_p1.umap'
backup=project/'Saved/FactoryRuntimeBackups'/datetime.now().strftime('editor_v2_%Y%m%d_%H%M%S')
backup.mkdir(parents=True,exist_ok=True)
def hashes():
 return {str(p.relative_to(project)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (project/'Content').rglob('*.umap') if p!=mapfile}
before=hashes()
level=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
assert level.load_level('/Game/DigitalTwin/Maps/factory_p1')
actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
runtimes=[a for a in actors.get_all_level_actors() if isinstance(a,unreal.FactoryRuntime)]
assert len(runtimes)==1
runtime=runtimes[0]
assert len(runtime.get_editor_property('body_meshes'))==80
assert len(runtime.get_editor_property('wheel_meshes'))==2
shutil.copy2(mapfile,backup/mapfile.name)
old=runtime.get_editor_property('initial_layout_json')
(backup/'previous_layout.json').write_text(old,encoding='utf8')
layout=(project/'Scripts/factory_editor_default_v2.json').read_text(encoding='utf8')
assert json.loads(layout)['version']==2
runtime.set_editor_property('initial_layout_json',layout)
runtime.call_method('BuildPreview')
assert runtime.call_method('VerifySetup')
assert level.save_current_level()
assert hashes()==before
report={'success':True,'backup':str(backup),'map':'/Game/DigitalTwin/Maps/factory_p1','other_maps_unchanged':True,'other_maps_sha256':before}
(project/'Saved/FactoryEditorUpgradeReport.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
unreal.log('FACTORY_EDITOR_V2_MAP_UPDATED')
