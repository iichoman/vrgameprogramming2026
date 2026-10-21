"""Extract Camry shell midsurfaces and solid boundary skins for Unreal inspection.

Dependency-free. Source coordinates are preserved and mm is converted to meters.
This exports geometry, not LS-DYNA materials, joints, contacts or simulation.
"""
from array import array
from collections import Counter, defaultdict
from pathlib import Path
import argparse
import json
import math
import re


GLASS = {
    2000290: ("Windshield_Midlayer", False),
    2000647: ("Windshield_Top", True),
    2000648: ("Windshield_Bottom", False),
    2000291: ("RearWindow_Midlayer", False),
    2000977: ("RearWindow_Top", True),
    2000978: ("RearWindow_Bottom", False),
    2000364: ("RearQuarterWindow", True),
    2000365: ("RearDoorWindow", True),
    2000369: ("FrontDoorWindow", True),
    2000546: ("RearQuarterWindowR", True),
    2000547: ("RearDoorWindowR", True),
    2000554: ("FrontDoorWindowR", True),
}
SEAT_MAJOR = {2005049, 2005050, 2000919, 2000920, 2005052, 2001053,
              2001015, 2001024, 2001025, 2001026, 2001031, 2000980}
HEX_FACES = ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7))
MATERIAL_NAMES = {"Paint": "Body", "MetalDark": "Dark", "Interior": "Dark",
                  "Fabric": "Seat", "Glass": "Glass", "Metal": "Metal", "Rubber": "Rubber"}


def clean_name(name):
    return re.sub(r"[^A-Za-z0-9_]+", "_", name).strip("_")[:90]


def classify(pid, name, element_type):
    n = name.lower()
    visible, separate = True, None
    if pid in GLASS:
        pane, visible = GLASS[pid]
        return "Glass", "Glass", visible, f"{pane}_{pid}"
    if "seat" in n or "headrest" in n or "head_rest" in n:
        fabric = any(x in n for x in ("cushion", "foam", "headrest"))
        if "bar" in n or "post" in n or "sprt" in n:
            fabric = False
        if pid in SEAT_MAJOR:
            separate = f"{clean_name(name)}_{pid}"
        return "Seats", "Fabric" if fabric else "MetalDark", visible, separate
    if "tire" in n or "tyre" in n or "tirerim" in n:
        return "Wheels", "Metal" if "rim" in n else "Rubber", visible, None
    if "door" in n:
        interior = any(x in n for x in ("trim", "panel", "foam", "armrest", "speaker"))
        return "Doors", "Interior" if interior else "Paint", visible, None
    if "roof" in n:
        return "Roof", "Paint", visible, None
    if "hood" in n or "trunk" in n:
        return "Closures", "Paint", visible, None
    if "bumper" in n:
        return "Bumpers", "Rubber" if "foam" in n else "Paint", visible, None
    if any(x in n for x in ("engine", "transmission", "coolant", "radiator", "battery",
                           "fuel", "exhaust", "muffler", "waterpump", "alternator", "airfilter")):
        return "Powertrain", "MetalDark", visible, None
    if any(x in n for x in ("susp", "strut", "aarm", "axle", "steer", "shock", "spring",
                           "hub", "brake", "subframe", "spindle")):
        return "Chassis", "MetalDark", visible, None
    if any(x in n for x in ("dash", "instrument", "glove", "console", "carpet", "headlin",
                           "airbag", "duct", "vent", "cockpit")):
        return "Interior", "Interior", visible, None
    return "Body", "Paint", visible, None


def read_source(source):
    parts, node_offsets, xyz = {}, {}, array("d")
    shells, solids = defaultdict(lambda: array("I")), defaultdict(lambda: array("I"))
    counts, omitted = Counter(), defaultdict(Counter)
    state, title = "", None
    with source.open(encoding="utf-8", errors="strict") as stream:
        for line in stream:
            if line.startswith("$") or not line.strip():
                continue
            if line.startswith("*"):
                state, title = line.strip().upper(), None
                if state.startswith("*INCLUDE"):
                    raise ValueError("Use the self-contained vehicle KEY, not combine.key")
                continue
            if state == "*PART":
                if title is None:
                    title = line.strip()
                else:
                    pid = int(line[:10])
                    if pid in parts:
                        raise ValueError(f"Duplicate source part {pid}")
                    parts[pid] = title
                    state = ""
            elif state == "*NODE":
                nid = int(line[:8])
                coords = tuple(float(line[i:i + 16].replace("D", "E")) * .001
                               for i in (8, 24, 40))
                if nid in node_offsets or not all(math.isfinite(v) for v in coords):
                    raise ValueError(f"Duplicate or nonfinite node {nid}")
                node_offsets[nid] = len(xyz)
                xyz.extend(coords)
            elif state in ("*ELEMENT_SHELL", "*ELEMENT_SOLID"):
                v = [int(x) for x in line.split()]
                expected = 6 if state == "*ELEMENT_SHELL" else 10
                if len(v) != expected:
                    raise ValueError(f"Unexpected {state} record: {line[:120]}")
                target = shells if expected == 6 else solids
                target[v[1]].extend(v[2:])
                counts[state] += 1
            elif state in ("*ELEMENT_BEAM", "*ELEMENT_DISCRETE", "*ELEMENT_MASS"):
                v = line.split()
                counts[state] += 1
                if state != "*ELEMENT_MASS":
                    omitted[state][int(v[1])] += 1
    if not node_offsets or not shells or not parts:
        raise ValueError("Source did not contain expected Camry geometry")
    return parts, node_offsets, xyz, shells, solids, counts, omitted


def make_surfaces(shells, solids, node_offsets, xyz):
    surfaces, kinds, solid_stats, issues = {}, {}, {}, []

    def position(nid):
        start = node_offsets[nid]
        return xyz[start:start + 3]

    for pid, records in shells.items():
        output = array("I")
        for i in range(0, len(records), 4):
            face = list(dict.fromkeys(n for n in records[i:i + 4] if n))
            if len(face) < 3:
                issues.append({"kind": "degenerate_shell", "part_id": pid, "record": i // 4})
                continue
            for nid in face:
                if nid not in node_offsets:
                    raise ValueError(f"Shell {pid} references absent node {nid}")
            output.extend(face + [0] * (4 - len(face)))
        surfaces[pid], kinds[pid] = output, "shell_midsurface"
    for pid, records in solids.items():
        if pid in surfaces:
            raise ValueError(f"Part {pid} has both solid and shell elements")
        boundary, incidences = {}, Counter()
        shape_counts = Counter()
        for i in range(0, len(records), 8):
            corners = records[i:i + 8]
            unique = list(dict.fromkeys(corners))
            shape_counts[len(unique)] += 1
            if len(unique) not in (4, 6, 8) or 0 in unique:
                raise ValueError(f"Unsupported solid topology in {pid}: {corners}")
            points = [position(nid) for nid in unique]
            center = tuple(sum(p[axis] for p in points) / len(points) for axis in range(3))
            for template in HEX_FACES:
                face = list(dict.fromkeys(corners[j] for j in template))
                if len(face) < 3:
                    continue
                key = tuple(sorted(face))
                incidences[key] += 1
                if incidences[key] > 1:
                    boundary.pop(key, None)
                    continue
                vertices = [position(nid) for nid in face]
                normal = [0., 0., 0.]
                for a, b in zip(vertices, vertices[1:] + vertices[:1]):
                    normal[0] += (a[1] - b[1]) * (a[2] + b[2])
                    normal[1] += (a[2] - b[2]) * (a[0] + b[0])
                    normal[2] += (a[0] - b[0]) * (a[1] + b[1])
                face_center = tuple(sum(p[axis] for p in vertices) / len(vertices) for axis in range(3))
                if sum(normal[a] * (face_center[a] - center[a]) for a in range(3)) < 0:
                    face.reverse()
                boundary[key] = face
        output = array("I")
        for face in boundary.values():
            output.extend(face + [0] * (4 - len(face)))
        nonmanifold = [(key, n) for key, n in incidences.items() if n > 2]
        for key, n in nonmanifold:
            issues.append({"kind": "solid_face_incidence_gt_2", "part_id": pid,
                           "node_ids": key, "incidence": n, "action": "excluded_non_boundary"})
        surfaces[pid], kinds[pid] = output, "solid_boundary_skin"
        solid_stats[pid] = {"elements": len(records) // 8, "unique_corner_counts": dict(shape_counts),
                            "boundary_faces": len(output) // 4,
                            "internal_faces": sum(n == 2 for n in incidences.values()),
                            "nonmanifold_faces": len(nonmanifold)}
    return surfaces, kinds, solid_stats, issues


def convert(source, output, max_triangles=180000):
    output.mkdir(parents=True, exist_ok=True)
    parts, offsets, xyz, shells, solids, element_counts, omitted = read_source(source)
    print(f"Read {len(parts)} parts, {len(offsets):,} nodes", flush=True)
    surfaces, kinds, solid_stats, issues = make_surfaces(shells, solids, offsets, xyz)
    expected_parts = set(shells) | set(solids)
    if set(surfaces) != expected_parts or expected_parts - parts.keys():
        raise ValueError("Geometry source coverage failed")
    del shells, solids
    buckets = defaultdict(list)
    part_records = []
    for pid in sorted(surfaces):
        faces = surfaces[pid]
        tri_count = sum(2 if faces[i + 3] else 1 for i in range(0, len(faces), 4))
        category, material, visible, separate = classify(pid, parts[pid], kinds[pid])
        part_records.append({"part_id": pid, "name": parts[pid], "geometry": kinds[pid],
                             "faces": len(faces) // 4, "triangles": tri_count,
                             "category": category, "material": material, "preview_visible": visible,
                             "solid_details": solid_stats.get(pid)})
        start, count = 0, 0
        for i in range(0, len(faces), 4):
            triangles = 2 if faces[i + 3] else 1
            if count + triangles > max_triangles:
                buckets[(category, material, visible, separate)].append((pid, start, i, count))
                start, count = i, 0
            count += triangles
        if count:
            buckets[(category, material, visible, separate)].append((pid, start, len(faces), count))

    meshes, all_bounds = [], [[math.inf] * 3, [-math.inf] * 3]
    written_faces = Counter()

    def write_mesh(key, segments, sequence):
        category, material, visible, separate = key
        suffix = separate or material
        name = f"SM_CamryFull_{category}_{suffix}_{sequence:02d}"
        relative = Path(category) / f"{name}.obj"
        file_path = output / relative
        file_path.parent.mkdir(parents=True, exist_ok=True)
        node_map, bounds = {}, [[math.inf] * 3, [-math.inf] * 3]
        for pid, start, end, _ in segments:
            faces = surfaces[pid]
            for nid in faces[start:end]:
                if nid and nid not in node_map:
                    node_map[nid] = len(node_map) + 1
        counts = Counter()
        with file_path.open("w", encoding="utf-8", newline="\n") as stream:
            stream.write("# Camry FE geometry: meters; original axes/origin preserved\n")
            stream.write("# Shell midsurfaces and exterior solid skins; no imported physics\n")
            for nid in node_map:
                start = offsets[nid]
                coords = xyz[start:start + 3]
                stream.write("v " + " ".join(f"{v:.9g}" for v in coords) + "\n")
                for a, value in enumerate(coords):
                    bounds[0][a] = min(bounds[0][a], value)
                    bounds[1][a] = max(bounds[1][a], value)
                    all_bounds[0][a] = min(all_bounds[0][a], value)
                    all_bounds[1][a] = max(all_bounds[1][a], value)
            for pid, start, end, _ in segments:
                stream.write(f"o p{pid}_{clean_name(parts[pid])}\n")
                faces = surfaces[pid]
                for i in range(start, end, 4):
                    face = [node_map[n] for n in faces[i:i + 4] if n]
                    if len(face) not in (3, 4) or any(n < 1 or n > len(node_map) for n in face):
                        raise ValueError("Output face validation failed")
                    stream.write("f " + " ".join(str(n) for n in face) + "\n")
                    counts["faces"] += 1
                    counts["triangles"] += len(face) - 2
                    written_faces[pid] += 1
        if counts["triangles"] > max_triangles:
            raise ValueError("Output exceeded configured triangle limit")
        pids = sorted({segment[0] for segment in segments})
        meshes.append({"name": name, "filename": relative.as_posix(), "category": category,
                       "material": MATERIAL_NAMES[material], "visible": visible, "part_ids": pids,
                       "parts": [{"id": pid, "name": parts[pid], "kind": kinds[pid]}
                                 for pid in pids], "vertices": len(node_map), **counts,
                       "bounds_m": bounds, "bytes": file_path.stat().st_size})

    for key, segments in sorted(buckets.items(), key=lambda item: str(item[0])):
        batch, triangles, sequence = [], 0, 1
        for segment in segments:
            if batch and triangles + segment[3] > max_triangles:
                write_mesh(key, batch, sequence)
                batch, triangles, sequence = [], 0, sequence + 1
            batch.append(segment)
            triangles += segment[3]
        if batch:
            write_mesh(key, batch, sequence)
    for pid, faces in surfaces.items():
        if written_faces[pid] != len(faces) // 4:
            raise ValueError(f"Source face coverage failed for {pid}")
    no_geometry = sorted(set(parts) - set(surfaces))
    manifest = {
        "source": str(source.resolve()), "unit": "meter", "source_unit": "millimeter",
        "scale_from_source": .001, "source_axes_origin_preserved": True,
        "source_nodes": len(offsets), "source_part_count": len(parts),
        "source_element_counts": dict(element_counts), "max_triangles_per_mesh": max_triangles,
        "bounds_m": all_bounds, "meshes": meshes, "parts": part_records,
        "totals": {"meshes": len(meshes), "exported_parts": len(surfaces),
                   "shell_parts": sum(k == "shell_midsurface" for k in kinds.values()),
                   "solid_parts": sum(k == "solid_boundary_skin" for k in kinds.values()),
                   "vertices": sum(m["vertices"] for m in meshes),
                   "faces": sum(m["faces"] for m in meshes),
                   "triangles": sum(m["triangles"] for m in meshes),
                   "preview_visible_triangles": sum(m["triangles"] for m in meshes if m["visible"])},
        "omitted": {"parts": [{"part_id": pid, "name": parts[pid]} for pid in no_geometry],
                    "beam_discrete_elements_by_part": {k: dict(v) for k, v in omitted.items()},
                    "mass_elements": element_counts["*ELEMENT_MASS"]},
        "issues": issues,
        "validation": {"source_part_coverage": True, "source_face_coverage": True,
                       "finite_coordinates": True, "output_indices_in_range": True,
                       "per_mesh_triangle_limit": True},
        "limitations": [
            "Semantic grouping is a name-based convenience, not verified manufacturing assemblies.",
            "All shell layers are exported; inner windshield/rear-window layers are hidden only in preview.",
            "Shell geometry is the midsurface; thickness is not extruded.",
            "Solid internal faces are removed per part; no solid FEM physics is imported.",
            "Beam welds/seat springs, discrete elements and point masses have no exported geometry.",
            "No original rendering materials, UVs, mechanical joints, weld constraints or deformation behavior.",
            "This crash-analysis FE model is not validated manufacturing CAD or tolerance evidence.",
        ],
    }
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"totals": manifest["totals"], "bounds_m": all_bounds,
                      "issues": issues, "manifest": str(output / "manifest.json")}, indent=2), flush=True)
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--max-triangles", type=int, default=180000)
    args = parser.parse_args()
    if args.max_triangles < 2:
        parser.error("max-triangles must be at least 2")
    convert(args.source, args.output, args.max_triangles)
