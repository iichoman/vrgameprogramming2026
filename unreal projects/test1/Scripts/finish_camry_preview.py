import json
from pathlib import Path
import unreal

project = Path(unreal.Paths.project_dir()).resolve()
destination = '/Game/DigitalTwin/Camry/FrontDoorPreview'
map_path = '/Game/DigitalTwin/Maps/CamryDoorPreview'
level = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
assert level.load_level(map_path)
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
meshes = [unreal.load_asset(p) for p in unreal.EditorAssetLibrary.list_assets(destination + '/Parts')]
meshes = [m for m in meshes if isinstance(m, unreal.StaticMesh)]
assert len(meshes) == 6
assert not [a for a in actors.get_all_level_actors() if isinstance(a, unreal.StaticMeshActor)], 'Map is not empty'
info = []
for mesh in meshes:
    actor = actors.spawn_actor_from_class(unreal.StaticMeshActor, unreal.Vector(250, 0, 0))
    assert actor is not None, 'Class spawn failed'
    assert actor.static_mesh_component.set_static_mesh(mesh)
    actor.set_actor_label(mesh.get_name())
    bounds = mesh.get_bounds()
    info.append({'asset': mesh.get_path_name(), 'origin_cm': list((bounds.origin.x, bounds.origin.y, bounds.origin.z)),
                 'extent_cm': list((bounds.box_extent.x, bounds.box_extent.y, bounds.box_extent.z))})
floor = actors.spawn_actor_from_class(unreal.StaticMeshActor, unreal.Vector(0, 0, -5))
floor.static_mesh_component.set_static_mesh(unreal.load_asset('/Engine/BasicShapes/Plane.Plane'))
floor.set_actor_scale3d(unreal.Vector(8, 8, 1))
floor.set_actor_label('PreviewFloor')
for yaw in (30, 210):
    actors.spawn_actor_from_class(unreal.DirectionalLight, unreal.Vector(0, 0, 300), unreal.Rotator(-40, yaw, 0))
unreal.EditorLevelLibrary.set_level_viewport_camera_info(unreal.Vector(350, -380, 230), unreal.Rotator(-18, 140, 0))
assert level.save_current_level()
# Re-open saved map to verify assets persisted and actors reference all six meshes.
assert level.load_level(map_path)
loaded = [a for a in actors.get_all_level_actors() if isinstance(a, unreal.StaticMeshActor)
          and a.static_mesh_component.static_mesh in meshes]
assert len(loaded) == 6
dimensions = [max(i['origin_cm'][a]+i['extent_cm'][a] for i in info) -
              min(i['origin_cm'][a]-i['extent_cm'][a] for i in info) for a in range(3)]
assert all(abs(a-b) < .1 for a,b in zip(dimensions, [120.0527, 182.48858, 73.12665])), dimensions
unreal.EditorAssetLibrary.delete_asset(destination + '/SM_CamryFrontDoor')
report = {'success': True, 'map': map_path, 'meshes': info, 'saved_map_verified_mesh_actors': len(loaded),
          'dimensions_cm': dimensions, 'viewport_render_verified': False}
(project/'Saved/CamryImportReport.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
unreal.log('CAMRY_IMPORT_SUCCESS')
