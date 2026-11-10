"""Repair the saved factory rail without altering imported CAD or other maps.

The old adaptation scaled SA02 children by 1.35 and then scaled their parent
from 8 to 18.4 on the travel axis. This script is for that original map state;
it keeps the central span long and restores compact geometry at both CAD ends.
"""
import json
from pathlib import Path
import unreal

MAP = '/Game/DigitalTwin/Maps/factory_p1'
levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
assert levels.load_level(MAP)
all_actors = actors.get_all_level_actors()
fixed = next(a for a in all_actors if a.get_actor_label() == 'Traverse187_FixedRail')
children = [a for a in all_actors if a.get_attach_parent_actor() == fixed]
assert len(children) > 90, 'Expected the imported 187 rail parts.'
assert abs(fixed.get_actor_scale3d().y - 18.4) < 0.01, 'Unexpected rail layout; refusing an unsafe repeat.'

# Remove the extra child stretch before fitting the station span.
for part in children:
    s = part.get_actor_scale3d()
    if abs(s.y - 24.84) < 0.02:
        part.set_actor_scale3d(unreal.Vector(s.x, s.y / 1.35, s.z))

fixed.set_actor_scale3d(unreal.Vector(2, 10, 1))
fixed.set_actor_location(unreal.Vector(1330, 70, 2), False, True)
by_label={a.get_actor_label():a for a in children}
left=by_label['T187_1_2'].get_actor_location().x
right=by_label['T187_1_4'].get_actor_location().x
assert abs(left+515)<1 and abs(right-1375)<1
mid=(left+right)/2
span={'T187_0187_201_Plate_body1',
      'T187_0187_103_Timing_Belt_TypeMA8_Teeth382_body1',
      'T187_0187_103_Timing_Belt_TypeMA8_Teeth382_body1_2'}
for label in span:
    part=by_label[label]
    loc=part.get_actor_location()
    part.set_actor_location(unreal.Vector(mid,loc.y,loc.z),False,True)

# Restore the end assemblies about their drive centers. Only the continuous
# plate/belts keep the travel scale; the remaining CAD parts use scale 2.
old={a.get_actor_label():a.get_actor_transform() for a in children}
fixed.set_actor_scale3d(unreal.Vector(2,2,1))
counts={'left':0,'right':0,'middle':0,'span':0}
for part in children:
    loc=old[part.get_actor_label()].translation
    x=loc.x
    if part.get_actor_label() in span:
        part.set_actor_scale3d(unreal.Vector(2,10,1))
        counts['span']+=1
    else:
        part.set_actor_scale3d(unreal.Vector(2,2,1))
        if x < -400:
            x=left+(x-left)*.2
            counts['left']+=1
        elif x > 1200:
            x=right+(x-right)*.2
            counts['right']+=1
        else:
            counts['middle']+=1
    part.set_actor_location(unreal.Vector(x,loc.y,loc.z),False,True)
assert counts['left']>35 and counts['right']>35 and counts['span']==3
assert levels.save_current_level()
Path(unreal.Paths.project_saved_dir(), 'RailRepair.json').write_text(
    json.dumps({'map': MAP, 'parts': len(children), 'root_scale': [2,2,1],
                'root_location': [1330,70,2], 'end_centers_cm':[left,right],
                'centered_span_cm':mid, 'groups':counts}, indent=2), encoding='utf-8')
