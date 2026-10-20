"""Create CAR_MODULE_TEST; use -ProcessModulesVerifyOnly for a cold-load audit."""
import json
import traceback
from pathlib import Path
import sys
import unreal

PROJECT=Path(unreal.Paths.project_dir()).resolve()
sys.path.insert(0,str(PROJECT/'Scripts'))
import process_import_base as base

SOURCE=PROJECT.parent.parent/'car models/Camry_Converted/ProcessModules'
MANIFEST=json.loads((PROJECT/'Scripts/process_vehicle_manifest.json').read_text(encoding='utf-8'))
DEST='/Game/DigitalTwin/Camry/ProcessModules'
MAP='/Game/DigitalTwin/Maps/CAR_MODULE_TEST'
VERIFY='-ProcessModulesVerifyOnly' in unreal.SystemLibrary.get_command_line()
RESUME='-ProcessModulesResumeSetup' in unreal.SystemLibrary.get_command_line()
base.SOURCE,base.DEST,base.MAP,base.MANIFEST=SOURCE,DEST,MAP,MANIFEST
ACTORS,LEVELS=base.ACTORS,base.LEVELS


def main():
    if not VERIFY:
        exists=unreal.EditorAssetLibrary.does_asset_exist(MAP)
        if exists and not RESUME:
            raise RuntimeError('Map already exists; use verify-only. Existing maps are never replaced.')
        materials={name:unreal.load_asset('/Game/DigitalTwin/Camry/FullVehicleTest/Materials/M_FullVehicle_'+name)
                   for name in ('Body','Metal','Dark','Seat','Rubber','Glass')}
        assert all(materials.values())
        if RESUME:
            assert exists and LEVELS.load_level(MAP)
            assert not any(isinstance(a,(unreal.VehicleProcessModule,unreal.VehicleProcessTestDriver)) for a in ACTORS.get_all_level_actors()), 'Resume only accepts the empty setup from an interrupted initial creation.'
            meshes={m['name']:unreal.load_asset(DEST+'/Meshes/'+m['category']+'/'+m['name']) for m in MANIFEST['meshes']}
            assert all(meshes.values())
        else:
            meshes={m['name']:base.import_mesh(m,materials) for m in MANIFEST['meshes']}
        # Reuse the lightweight floor/lights/cameras setup, with no old vehicle actors.
        if not RESUME: base.create_map([])
        frame=base.place(unreal.VehicleProcessTestDriver,'Vehicle_ProcessFrame',(250,0,0),folder='VehicleModules')
        wheel_actors={}
        entries={m['name']:m for m in MANIFEST['meshes']}
        for m in MANIFEST['modules']:
            p=m['pivot_ue_cm']
            actor=base.place(unreal.VehicleProcessModule,m['id'],(250+p[0],p[1],p[2]),folder='VehicleModules/'+m['id'])
            actor.attach_to_actor(frame,unreal.Name('None'),unreal.AttachmentRule.KEEP_WORLD,unreal.AttachmentRule.KEEP_WORLD,unreal.AttachmentRule.KEEP_WORLD,False)
            actor.set_editor_property('module_id',unreal.Name(m['id']))
            actor.set_editor_property('vehicle_frame',frame)
            actor.set_editor_property('mesh_offset_cm',unreal.Vector(*[-v for v in p]))
            actor.set_editor_property('wheel_motion_enabled',m['motion_enabled'])
            parts=[]
            for name in m['meshes']:
                part=unreal.VehicleModuleMesh()
                part.set_editor_property('mesh',meshes[name])
                part.set_editor_property('preview_visible',entries[name]['visible'])
                parts.append(part)
            actor.set_editor_property('mesh_parts',parts)
            actor.call_method('RebuildVisuals')
            assert actor.call_method('CaptureReferencePose'),m['id']
            actor.set_editor_property('tags',[unreal.Name('ProcessModule'),unreal.Name(m['classification'])])
            if m['motion_enabled']: wheel_actors[m['id']]=actor
        frame.set_editor_property('wheels',[wheel_actors['Wheel_'+corner] for corner in ('FL','FR','RL','RR')])
        assert LEVELS.save_current_level()
    assert LEVELS.load_level(MAP)
    modules={str(a.get_editor_property('module_id')):a for a in ACTORS.get_all_level_actors() if isinstance(a,unreal.VehicleProcessModule)}
    assert len(modules)==42
    source_pids=[]
    actual_meshes=0
    actual_triangles=0
    for m in MANIFEST['modules']:
        actor=modules[m['id']]
        p=m['pivot_ue_cm']
        assert actor.get_editor_property('has_reference_pose')
        assert actor.get_editor_property('wheel_motion_enabled')==m['motion_enabled']
        assert actor.get_attach_parent_actor()==actor.get_editor_property('vehicle_frame')
        assert actor.call_method('GetVisualCount')==len(m['meshes'])
        assert actor.call_method('GetCurrentOffsetMm').is_nearly_zero(.001)
        comps=actor.get_components_by_class(unreal.StaticMeshComponent)
        assert len(comps)==len(m['meshes'])
        lookup={c.static_mesh.get_name():c for c in comps}
        for entry in (e for e in MANIFEST['meshes'] if e['category']==m['id']):
            comp=lookup[entry['name']]
            base.check_mesh(comp.static_mesh,entry)
            # Each component's world origin must equal CAR_TEST's common origin;
            # per-module pivots must cancel the retained source vertex coordinates.
            pos=comp.get_world_location()
            assert all(abs(a-b)<.001 for a,b in zip(base.xyz(pos),(250,0,0))),(m['id'],pos)
            assert comp.get_editor_property('visible')==entry['visible']
            assert comp.get_editor_property('hidden_in_game')==(not entry['visible'])
            assert str(comp.get_collision_profile_name())=='NoCollision'
            assert comp.get_editor_property('body_instance').get_editor_property('collision_enabled')==unreal.CollisionEnabled.NO_COLLISION
            assert not actor.get_actor_enable_collision()
            actual_meshes+=1
            actual_triangles+=comp.static_mesh.get_num_triangles(0)
        source_pids.extend(m['part_ids'])
    assert len(source_pids)==len(set(source_pids))==1068
    assert actual_meshes==len(MANIFEST['meshes'])
    frames=[a for a in ACTORS.get_all_level_actors() if isinstance(a,unreal.VehicleProcessTestDriver)]
    assert len(frames)==1
    assert [str(w.get_editor_property('module_id')) for w in frames[0].get_editor_property('wheels')]==['Wheel_FL','Wheel_FR','Wheel_RL','Wheel_RR']
    report={'success':True,'map':MAP,'cold_load':VERIFY,'module_count':42,'mesh_count':actual_meshes,
            'source_surface_parts':1068,'source_triangles':MANIFEST['totals']['triangles'],
            'imported_triangles':actual_triangles,'four_motion_enabled_wheels':True,
            'assembled_alignment_preserved':True,'collision_and_visibility_verified':True,
            'classification':'Candidate modules; nonwheel interfaces pending review. Not production/physical validation.'}
    path=PROJECT/'Saved'/('CAR_MODULE_TEST_ColdLoadReport.json' if VERIFY else 'CAR_MODULE_TEST_ImportReport.json')
    path.write_text(json.dumps(report,indent=2),encoding='utf-8')
    unreal.log('PROCESS_MODULES_SUCCESS '+str(path))


try:
    main()
except Exception:
    (PROJECT/'Saved/CAR_MODULE_TEST_Error.json').write_text(traceback.format_exc(),encoding='utf-8')
    raise
