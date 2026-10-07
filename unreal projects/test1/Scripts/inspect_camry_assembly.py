import json
from pathlib import Path
import unreal
p=Path(unreal.Paths.project_dir()).resolve()
level=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
assert level.load_level('/Game/DigitalTwin/Maps/CamryAssemblyPreview')
actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
cls=unreal.EditorAssetLibrary.load_blueprint_class('/Game/DigitalTwin/Blueprints/BP_CamryAssembly')
a=actors.spawn_actor_from_class(cls,unreal.Vector(0,0,0))
def snapshot():
    result=[]
    for c in a.get_components_by_class(unreal.SceneComponent):
        v=c.get_world_location()
        result.append({'name':c.get_name(),'parent':c.get_attach_parent().get_name() if c.get_attach_parent() else None,
                       'tags':[str(t) for t in c.component_tags],'location':[v.x,v.y,v.z], 'mobility':str(c.mobility)})
    return result
before=snapshot()
left=next(c for c in a.get_components_by_class(unreal.SceneComponent) if c.get_name()=='LeftDoor')
v=left.get_editor_property('relative_location')
left.set_relative_location(unreal.Vector(v.x,v.y+10,v.z),False,True)
(p/'Saved/CamryHierarchyDebug.json').write_text(json.dumps({'before':before,'after':snapshot()},indent=2),encoding='utf-8')
