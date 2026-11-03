"""Upgrade only factory_p1; keep the team's rail/transport hierarchy and source maps."""
import unreal,json,hashlib,shutil
from datetime import datetime
from pathlib import Path
p=Path(unreal.Paths.project_dir()).resolve()
levels=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
api=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
target=p/'Content/DigitalTwin/Maps/factory_p1.umap'
backup=p/'Saved/FactoryRuntimeBackups'/datetime.now().strftime('modular_editor_%Y%m%d_%H%M%S')
backup.mkdir(parents=True,exist_ok=True)
shutil.copy2(target,backup/target.name)
other={f:hashlib.sha256(f.read_bytes()).hexdigest() for f in (p/'Content').rglob('*.umap') if f!=target}
assert levels.load_level('/Game/DigitalTwin/Maps/AutonomousAssemblyLine')
def rail_signature():
 return sorted(a.static_mesh_component.static_mesh.get_path_name() for a in api.get_all_level_actors()
  if isinstance(a,unreal.StaticMeshActor) and a.static_mesh_component.static_mesh and unreal.Name('WheelTransport187') in a.tags)
source_rail=rail_signature()
assert len(source_rail)>20
assert levels.load_level('/Game/DigitalTwin/Maps/factory_p1')
r=next(a for a in api.get_all_level_actors() if isinstance(a,unreal.FactoryRuntime))
assert r.get_editor_property('use_assembly_integration')
# Remove old layout proxy conveyors only. CAD rails and moving carriage are retained.
layout=json.loads(r.get_editor_property('initial_layout_json'))
old_ids={i['id'] for i in layout['items'] if i['kind']=='conveyor'}
layout['items']=[i for i in layout['items'] if i['id'] not in old_ids]
layout['process_order']=[i for i in layout['process_order'] if i not in old_ids]
r.set_editor_property('initial_layout_json',json.dumps(layout,ensure_ascii=False))
r.call_method('BuildPreview')
new=json.loads(r.get_editor_property('initial_layout_json'))
assert new['version']==3 and len(new['assembly_bindings'])==3
assert all(any(i['id']==v for i in new['items']) for v in new['assembly_bindings'].values())
assert rail_signature()==source_rail,'Factory rail differs from team source; do not guess or delete CAD.'
assert not any(i['kind']=='conveyor' for i in new['items'])
assert levels.save_current_level()
assert all(hashlib.sha256(f.read_bytes()).hexdigest()==h for f,h in other.items())
report=dict(success=True,backup=str(backup),removed_proxy_ids=sorted(old_ids),rail_mesh_count=len(source_rail),rail_matches_team=True,source_maps_unchanged=True,assembly_bindings=new['assembly_bindings'])
(p/'Saved/FactoryModularInstallReport.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
