"""Regroup the original Camry FE surfaces into candidate process modules.

Run with source KEY and output folder. Requires the bundled reviewed inventory.
No existing vehicle assets are overwritten. Pivots are geometric, not mass centers.
"""
from pathlib import Path
from collections import defaultdict
import argparse
import json
import camry_geometry as geometry
from module_classification import classify_part


def convert(source, output):
    inventory = json.loads(Path(__file__).with_name('process_part_inventory.json').read_text(encoding='utf-8'))
    records = {p['part_id']: p for p in inventory['parts']}
    original_classify = geometry.classify

    def classify(pid, name, kind):
        record = records[pid]
        assert record['name'] == name, (pid, name)
        module, confidence, note = classify_part(pid, name, record['bounds_m'])
        _, material, visible, _ = original_classify(pid, name, kind)
        # Keep hidden/visible laminates separate; combine seat materials within each seat.
        return module, material, visible, 'Hidden' if not visible else None

    geometry.classify = classify
    manifest = geometry.convert(source, output)
    geometry.classify = original_classify
    groups = defaultdict(list)
    for record in records.values():
        module, confidence, note = classify_part(record['part_id'], record['name'], record['bounds_m'])
        groups[module].append(record)
    modules = []
    for module, parts in sorted(groups.items()):
        bounds = [[min(p['bounds_m'][0][a] for p in parts) for a in range(3)],
                  [max(p['bounds_m'][1][a] for p in parts) for a in range(3)]]
        pivot_parts = [p for p in parts if 'rim' in p['name'].lower()] if module.startswith('Wheel_') else parts
        assert pivot_parts
        pivot_m = [(min(p['bounds_m'][0][a] for p in pivot_parts)+max(p['bounds_m'][1][a] for p in pivot_parts))*.5 for a in range(3)]
        modules.append({'id': module, 'part_ids': sorted(p['part_id'] for p in parts),
                        'bounds_m': bounds, 'pivot_ue_cm': [pivot_m[0]*100, -pivot_m[1]*100, pivot_m[2]*100],
                        'pivot_kind': 'rim_bounds_center' if module.startswith('Wheel_') else 'geometry_bounds_center_not_COM',
                        'motion_enabled': module.startswith('Wheel_'),
                        'classification': 'verified_wheel_geometry' if module.startswith('Wheel_') else 'candidate_boundary_review',
                        'meshes': [m['name'] for m in manifest['meshes'] if m['category']==module]})
    assert len(modules) == 42
    assert manifest['totals']['exported_parts']==1068
    assert manifest['totals']['faces']==2193002 and manifest['totals']['triangles']==4269686
    assert len({pid for m in modules for pid in m['part_ids']})==1068
    assert sum(len(m['part_ids']) for m in modules)==1068
    manifest['modules']=modules
    manifest['classification_inventory']='process_part_inventory.json'
    manifest['coordinate_conversion']='OBJ meters (x,y,z) -> UE cm (100*x,-100*y,100*z). Actor frame (250,0,0). Mesh relative translation = -module pivot.'
    manifest['limitations'][0]='42 candidate process groups, not a verified production BOM. Only four wheel tire/rim groups enabled for motion. Mount/hinge/review groups stay fixed.'
    (output/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    Path(__file__).with_name('process_vehicle_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    print('PROCESS_GROUPS_VERIFIED',len(modules),manifest['totals'],flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    if (args.output/'manifest.json').exists():
        parser.error('Output already exists; choose a new folder to preserve conversion provenance.')
    convert(args.source,args.output)
