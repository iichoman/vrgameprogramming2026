import json
from pathlib import Path
import unreal

p=Path(unreal.Paths.project_dir()).resolve()
bp_path='/Game/DigitalTwin/Blueprints/BP_CamryAssembly'
map_path='/Game/DigitalTwin/Maps/CamryAssemblyPreview'
level=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
assert level.load_level(map_path)
actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
cls=unreal.EditorAssetLibrary.load_blueprint_class(bp_path)
old=[a for a in actors.get_all_level_actors() if isinstance(a,unreal.StaticMeshActor)
     and a.static_mesh_component.static_mesh and '/Camry/' in a.static_mesh_component.static_mesh.get_path_name()]
assert len(old)==11
assembly=actors.spawn_actor_from_class(cls,unreal.Vector(0,0,0))
assembly.set_actor_label('CamryAssembly')
components=assembly.get_components_by_class(unreal.StaticMeshComponent)
assert len(components)==11
def xyz(v): return [v.x,v.y,v.z]
def check(actual,expected,detail):
    assert all(abs(a-b)<.01 for a,b in zip(actual,expected)), (detail,actual,expected)
before={c.get_name():xyz(c.get_world_location()) for c in components}
for actor in old:
    c=next(c for c in components if c.static_mesh==actor.static_mesh_component.static_mesh)
    check(before[c.get_name()],xyz(actor.get_actor_location()),'Initial placement')
nodes={g:next(c for c in assembly.get_components_by_class(unreal.SceneComponent) if c.get_name()==g)
       for g in ('LeftDoor','RightDoor')}
pivots={g:xyz(node.get_editor_property('relative_location')) for g,node in nodes.items()}
for group,node in nodes.items():
    saved=pivots[group]
    node.set_relative_location(unreal.Vector(saved[0],saved[1]+10,saved[2]),False,True)
    for c in components:
        expected=before[c.get_name()].copy()
        if c.get_attach_parent().get_name()==group:
            expected[1]+=10
        check(xyz(c.get_world_location()),expected,group+' '+c.get_name())
    node.set_relative_location(unreal.Vector(*saved),False,True)
    for c in components:
        check(xyz(c.get_world_location()),before[c.get_name()],'Door restore')
assembly.set_actor_location(unreal.Vector(20,30,40),False,True)
for c in components:
    check(xyz(c.get_world_location()),[a+b for a,b in zip(before[c.get_name()],[20,30,40])],'Whole assembly')
assembly.set_actor_location(unreal.Vector(0,0,0),False,True)
assert actors.destroy_actors(old)
assert level.save_current_level()
assert level.load_level(map_path)
saved=[a for a in actors.get_all_level_actors() if a.get_class()==cls]
assert len(saved)==1
saved_components=saved[0].get_components_by_class(unreal.StaticMeshComponent)
assert len(saved_components)==11
for c in saved_components:
    check(xyz(c.get_world_location()),before[c.get_name()],'Reload')
report={'success':True,'blueprint':bp_path,'map':map_path,'body_meshes':5,'left_door_meshes':3,
        'right_door_meshes':3,'door_pivots_cm':pivots,
        'checks':['initial placement preserved','left door moves alone','right door moves alone',
                  'whole vehicle translates together','saved map reload preserves pose'],
        'door_pivot_note':'Door geometry centers for positioning, not physical hinge axes',
        'viewport_render_verified':False}
(p/'Saved/CamryAssemblyReport.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
unreal.log('CAMRY_ASSEMBLY_SUCCESS')
