"""Register six approximate inspection stations and verify independent pose-based measurements."""
import json
import math
import shutil
import traceback
from pathlib import Path
from datetime import datetime
import unreal

project=Path(unreal.Paths.project_dir()).resolve()
report_path=project/'Saved/DoorInspectionReport.json'
verify='-InspectionVerifyOnly' in unreal.SystemLibrary.get_command_line()

def xyz(v): return [v.x,v.y,v.z]
def close(a,b): assert abs(a-b)<1.e-5, (a,b)
def main():
    levels=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    assert levels.load_level('/Game/DigitalTwin/Maps/CamryAssemblyPreview')
    assembly=next(a for a in actors.get_all_level_actors() if a.get_class().get_path_name()=='/Script/test1.CamryAssembly')
    inspector=assembly.get_components_by_class(unreal.DoorInspectionComponent)[0]
    controls={c.get_name():c for c in assembly.get_components_by_class(unreal.DoorPoseComponent)}
    pose=controls['LeftDoorPose']
    assert pose.call_method('HasReferencePose')
    meshes=assembly.get_components_by_class(unreal.StaticMeshComponent)
    before={c.get_name():xyz(c.get_world_location()) for c in meshes}
    normal=pose.get_editor_property('reference_pose_in_body')
    scene={c.get_name():c for c in assembly.get_components_by_class(unreal.SceneComponent)}
    current=scene['LeftDoor'].get_world_transform()
    body=scene['Body'].get_world_transform()
    expectednormal=body.transform_location(normal.translation)
    for a,b in zip(xyz(current.translation),xyz(expectednormal)): close(a,b)
    # Do not overwrite a measurement definition on verify/repeated runs.
    if not inspector.get_editor_property('points'):
        assert not verify
        backup=project/'Saved/Backups'/('Inspection_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
        backup.mkdir(parents=True)
        shutil.copy2(project/'Content/DigitalTwin/Maps/CamryAssemblyPreview.umap',backup/'CamryAssemblyPreview.umap')
        stations=json.loads((project/'Scripts/inspection_points.json').read_text())['stations']
        configured=[]
        for s in stations:
            x,y,z=s['door_m']
            bodypoint=unreal.Vector(x*100+250,-y*100,z*100)
            local=normal.inverse_transform_location(bodypoint)
            gap=unreal.Vector(*s['gap_direction_body'])
            anchor=unreal.Vector(bodypoint.x+gap.x*.3,bodypoint.y,bodypoint.z)
            p=unreal.DoorInspectionPoint()
            p.set_editor_property('name',unreal.Name(s['name']))
            p.set_editor_property('door_point_local_cm',local)
            p.set_editor_property('body_point_local_cm',anchor)
            p.set_editor_property('reference_door_point_body_cm',bodypoint)
            p.set_editor_property('gap_direction_body',gap)
            p.set_editor_property('flush_direction_body',unreal.Vector(0,-1,0))
            configured.append(p)
        inspector.set_editor_property('points',configured)
    # Unreal array/struct wrappers can alias the live property: preserve independent copies.
    points=[]
    for source in inspector.get_editor_property('points'):
        p=unreal.DoorInspectionPoint()
        for field in ('name','door_point_local_cm','body_point_local_cm','reference_door_point_body_cm','gap_direction_body','flush_direction_body'):
            p.set_editor_property(field,source.get_editor_property(field))
        points.append(p)
    assert len(points)==6

    def results():
        assert inspector.call_method('RefreshMeasurements')
        return inspector.get_editor_property('measurements')
    def zero():
        for r in results(): close(r.get_editor_property('gap_change_mm'),0); close(r.get_editor_property('flush_change_mm'),0)
    def check_translation(offset):
        # Event subscription must update results immediately, without a Tick/manual refresh.
        assert pose.call_method('ApplyReferenceOffsetMm',(offset,))
        actual=inspector.get_editor_property('measurements')
        assert len(actual)==6 and inspector.get_editor_property('measurements_valid')
        for p,r in zip(points,actual):
            g=p.get_editor_property('gap_direction_body'); f=p.get_editor_property('flush_direction_body')
            close(r.get_editor_property('gap_change_mm'),-sum(a*b for a,b in zip(xyz(offset),xyz(g))))
            close(r.get_editor_property('flush_change_mm'),sum(a*b for a,b in zip(xyz(offset),xyz(f))))
    zero()
    check_translation(unreal.Vector(1,0,0))
    check_translation(unreal.Vector(1,0,0))
    check_translation(unreal.Vector(0,-1,0))
    assert pose.call_method('RestoreReferencePose'); zero()
    # Rotate around door center; different stations must report different changes.
    turned=scene['LeftDoor'].get_world_transform()
    turned.rotation=unreal.Rotator(pitch=0,yaw=.5,roll=0).quaternion()
    assert pose.call_method('SetDoorWorldPose',(turned,))
    varied=[]
    for p,r in zip(points,results()):
        local=p.get_editor_property('door_point_local_cm')
        # Independent analytic z-axis rotation; origin at the saved door center.
        angle=math.radians(.5)
        delta=[(math.cos(angle)-1)*local.x-math.sin(angle)*local.y,
               math.sin(angle)*local.x+(math.cos(angle)-1)*local.y,0]
        g=p.get_editor_property('gap_direction_body'); f=p.get_editor_property('flush_direction_body')
        close(r.get_editor_property('gap_change_mm'),-10*sum(a*b for a,b in zip(delta,xyz(g))))
        close(r.get_editor_property('flush_change_mm'),10*sum(a*b for a,b in zip(delta,xyz(f))))
        varied.append(r.get_editor_property('flush_change_mm'))
    assert max(varied)-min(varied)>.1
    assert pose.call_method('RestoreReferencePose'); zero()
    saved=assembly.get_actor_transform()
    assembly.set_actor_rotation(unreal.Rotator(pitch=0,yaw=90,roll=0),True)
    assembly.set_actor_location(unreal.Vector(200,300,100),False,True)
    zero(); check_translation(unreal.Vector(1,-1,0))
    assert pose.call_method('RestoreReferencePose'); zero()
    assembly.set_actor_transform(saved,False,True); zero()
    assert controls['RightDoorPose'].call_method('ApplyReferenceOffsetMm',(unreal.Vector(10,20,30),))
    zero(); assert controls['RightDoorPose'].call_method('RestoreReferencePose')
    # Invalid axes clear stale results; restoring definition recovers.
    bad=unreal.DoorInspectionPoint()
    bad.set_editor_property('gap_direction_body',unreal.Vector(0,0,0))
    inspector.set_editor_property('points',[bad])
    assert not inspector.call_method('RefreshMeasurements')
    assert not inspector.get_editor_property('measurements')
    inspector.set_editor_property('points',points); zero()
    for c in meshes:
        for a,b in zip(xyz(c.get_world_location()),before[c.get_name()]): close(a,b)
    assert len([c for c in assembly.get_components_by_class(unreal.TextRenderComponent) if c.get_name().startswith('InspectionLabel')])==6
    if not verify:
        assert levels.save_current_level()
    report={'success':True,'cold_load_verified':verify,
        'model':'Six shell-edge samples, virtual 3mm body anchors, fixed body axes; signed deviations only, no OEM tolerances.',
        'checks':['zero baseline','1mm translations and correct signs','immediate event refresh','no accumulation',
                  'analytic rotation with different station results','rotated/translated body frame',
                  'right door isolation','invalid axes reject and clear stale results','mesh poses restored'],
        'points':[{'name':str(p.get_editor_property('name')),'door_local_cm':xyz(p.get_editor_property('door_point_local_cm')),
                   'body_local_cm':xyz(p.get_editor_property('body_point_local_cm'))} for p in points],
        'viewport_render_verified':False}
    report_path.write_text(json.dumps(report,indent=2),encoding='utf-8')
    unreal.log('DOOR_INSPECTION_SUCCESS')
try: main()
except Exception:
    (project/'Saved/DoorInspectionFailure.txt').write_text(traceback.format_exc(),encoding='utf-8')
    raise
