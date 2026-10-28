"""Upgrade only factory_p1 from its initial blockout to native runtime editing.

Run once after compiling FactoryRuntime. -FactoryRuntimeVerifyOnly performs a
cold-load reference audit without saving the map. Existing runtime installations
are never overwritten. All other project maps and existing mesh assets are read
only. The previous factory_p1 is backed up before its blockout actors are removed.
"""
import hashlib
import json
import shutil
import traceback
from datetime import datetime, timezone
from pathlib import Path

import unreal


PROJECT = Path(unreal.Paths.project_dir()).resolve()
MAP = "/Game/DigitalTwin/Maps/factory_p1"
MAP_FILE = PROJECT / "Content/DigitalTwin/Maps/factory_p1.umap"
MESH_ROOT = "/Game/DigitalTwin/Camry/ProcessModules/Meshes"
TINT_PATH = "/Game/DigitalTwin/Factory/Runtime/M_FactoryTint"
VERIFY = "-FactoryRuntimeVerifyOnly" in unreal.SystemLibrary.get_command_line()
ACTORS = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
LEVELS = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
REPORT = PROJECT / "Saved" / (
    "FactoryRuntimeColdLoadReport.json" if VERIFY else "FactoryRuntimeInstallReport.json"
)
MANIFEST = json.loads(
    (PROJECT / "Scripts/process_vehicle_manifest.json").read_text(encoding="utf-8")
)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def other_map_hashes():
    return {
        str(path.relative_to(PROJECT)).replace("\\", "/"): digest(path)
        for path in sorted((PROJECT / "Content").rglob("*.umap"))
        if path.resolve() != MAP_FILE.resolve()
    }


def runtime_actors():
    return [
        actor for actor in ACTORS.get_all_level_actors()
        if actor.get_class().get_path_name() == "/Script/test1.FactoryRuntime"
    ]


def make_tint_material():
    if unreal.EditorAssetLibrary.does_asset_exist(TINT_PATH):
        material = unreal.load_asset(TINT_PATH)
        assert isinstance(material, unreal.Material), TINT_PATH
        return material
    material = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        "M_FactoryTint", "/Game/DigitalTwin/Factory/Runtime",
        unreal.Material, unreal.MaterialFactoryNew()
    )
    assert material
    lib = unreal.MaterialEditingLibrary
    tint = lib.create_material_expression(
        material, unreal.MaterialExpressionVectorParameter, -300, 0
    )
    tint.set_editor_property("parameter_name", unreal.Name("Tint"))
    tint.set_editor_property("default_value", unreal.LinearColor(1, 1, 1, 1))
    assert lib.connect_material_property(tint, "", unreal.MaterialProperty.MP_BASE_COLOR)
    roughness = lib.create_material_expression(
        material, unreal.MaterialExpressionConstant, -300, 140
    )
    roughness.set_editor_property("r", 0.9)
    assert lib.connect_material_property(
        roughness, "", unreal.MaterialProperty.MP_ROUGHNESS
    )
    lib.recompile_material(material)
    assert unreal.EditorAssetLibrary.save_loaded_asset(material)
    return material


def mesh_path(entry):
    return MESH_ROOT + "/" + entry["category"] + "/" + entry["name"]


def load_mesh_parts():
    body, wheel = [], []
    for entry in MANIFEST["meshes"]:
        mesh = unreal.load_asset(mesh_path(entry))
        assert isinstance(mesh, unreal.StaticMesh), mesh_path(entry)
        part = unreal.VehicleModuleMesh()
        part.set_editor_property("mesh", mesh)
        part.set_editor_property("preview_visible", bool(entry["visible"]))
        (wheel if entry["category"] == "Wheel_FL" else body).append(part)
    assert len(body) == 80 and len(wheel) == 2, (len(body), len(wheel))
    return body, wheel


def retained_blockout_actors():
    return {
        actor.get_actor_label(): actor.get_class().get_path_name()
        for actor in ACTORS.get_all_level_actors()
        if str(actor.get_folder_path()) in (
            "FactoryP1/90_Environment", "FactoryP1/80_Views"
        )
    }


def install():
    assert MAP_FILE.is_file(), "Existing factory_p1 is required."
    assert LEVELS.load_level(MAP), MAP
    assert not runtime_actors(), (
        "FactoryRuntime already exists. Use -FactoryRuntimeVerifyOnly; "
        "this installer never overwrites an edited runtime map."
    )
    runtime_class = unreal.load_class(None, "/Script/test1.FactoryRuntime")
    assert runtime_class, "Compile FactoryRuntime before installing the map."
    assert (PROJECT / "Content/DigitalTwin/Fonts/NanumGothic.ttf").is_file(), (
        "Copy the Korean font and accompanying notices before installation."
    )
    # Resolve every dependency before touching the existing blockout.
    body, wheel = load_mesh_parts()
    make_tint_material()
    before_other_maps = other_map_hashes()
    retained = retained_blockout_actors()
    assert "FP1_FreeFly_Start" in retained, "Expected original PlayerStart."
    assert "FP1_View_Overview" in retained and "FP1_View_Plan" in retained

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    backup_dir = PROJECT / "Saved/FactoryRuntimeBackups" / stamp
    backup_dir.mkdir(parents=True, exist_ok=False)
    original_hash = digest(MAP_FILE)
    backup_file = backup_dir / "factory_p1.umap"
    shutil.copy2(MAP_FILE, backup_file)
    assert digest(backup_file) == original_hash
    (backup_dir / "backup.json").write_text(json.dumps({
        "source": str(MAP_FILE), "sha256": original_hash,
        "other_maps_sha256": before_other_maps,
        "note": "Original blockout before native runtime installation."
    }, ensure_ascii=False, indent=2), encoding="utf-8")

    removed = []
    for actor in ACTORS.get_all_level_actors():
        tags = {str(tag) for tag in actor.get_editor_property("tags")}
        folder = str(actor.get_folder_path())
        if "FactoryP1" not in tags:
            continue
        if folder in ("FactoryP1/90_Environment", "FactoryP1/80_Views"):
            continue
        # Only our old shell, markings and text are replaced. Untagged user
        # additions and the lighting/viewpoint actors remain untouched.
        removed.append(actor.get_actor_label())
        assert ACTORS.destroy_actor(actor), removed[-1]

    actor = ACTORS.spawn_actor_from_class(runtime_class, unreal.Vector(0, 0, 0))
    assert actor
    actor.set_actor_label("공장 편집·실행 (Factory Runtime)")
    actor.set_folder_path("FactoryRuntime")
    actor.set_editor_property("tags", [unreal.Name("FactoryRuntime")])
    actor.set_editor_property("initial_layout_json", "")
    actor.set_editor_property("body_meshes", body)
    actor.set_editor_property("wheel_meshes", wheel)
    actor.set_editor_property("wheel_pivot_cm", unreal.Vector(-94.52773, -80.64976, 33.70584))
    actor.call_method("BuildPreview")
    assert actor.call_method("VerifySetup"), "Native setup check failed."
    world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    world.get_world_settings().set_editor_property("default_game_mode", unreal.GameModeBase)
    assert retained_blockout_actors() == retained, "Environment/viewpoints changed."
    assert LEVELS.save_current_level(), "Map save failed."
    assert other_map_hashes() == before_other_maps, "Another project map changed."
    return {
        "backup": str(backup_file), "backup_sha256": original_hash,
        "removed_blockout_actors": len(removed),
        "preserved_environment_and_views": retained,
        "other_maps_sha256": before_other_maps,
    }


def verify():
    hashes_before = other_map_hashes()
    map_hash_before = digest(MAP_FILE)
    assert LEVELS.load_level(MAP)
    actors = runtime_actors()
    assert len(actors) == 1, "Exactly one native FactoryRuntime is required."
    runtime = actors[0]
    expected = {
        entry["name"]: (entry["category"], bool(entry["visible"]))
        for entry in MANIFEST["meshes"]
    }
    seen = set()
    counts = {}
    for property_name, is_wheel in (("body_meshes", False), ("wheel_meshes", True)):
        parts = runtime.get_editor_property(property_name)
        counts[property_name] = len(parts)
        for part in parts:
            mesh = part.get_editor_property("mesh")
            assert isinstance(mesh, unreal.StaticMesh)
            name = mesh.get_name()
            assert name not in seen, "Duplicate mesh: " + name
            seen.add(name)
            category, visible = expected[name]
            assert (category == "Wheel_FL") == is_wheel, name
            assert part.get_editor_property("preview_visible") == visible, name
    assert seen == set(expected), "Source mesh membership changed."
    assert counts == {"body_meshes": 80, "wheel_meshes": 2}, counts
    pivot = runtime.get_editor_property("wheel_pivot_cm")
    for value, target in zip((pivot.x, pivot.y, pivot.z), (-94.52773, -80.64976, 33.70584)):
        assert abs(value - target) < 0.0001
    assert runtime.call_method("VerifySetup"), "Native setup check failed."
    assert unreal.EditorAssetLibrary.does_asset_exist(TINT_PATH)
    assert retained_blockout_actors().get("FP1_FreeFly_Start") == "/Script/Engine.PlayerStart"
    assert other_map_hashes() == hashes_before
    assert digest(MAP_FILE) == map_hash_before, "Verification modified the saved map."
    previous_report = PROJECT / "Saved/FactoryRuntimeInstallReport.json"
    if VERIFY and previous_report.is_file():
        previous = json.loads(previous_report.read_text(encoding="utf-8"))
        assert previous["other_maps_sha256"] == hashes_before, "An original map changed after installation."
    return {
        "success": True, "map": MAP, "cold_load": VERIFY,
        "runtime_class": runtime.get_class().get_path_name(),
        "body_meshes": counts["body_meshes"], "wheel_meshes": counts["wheel_meshes"],
        "source_visibility_preserved": True, "other_maps_unchanged": True,
        "runtime_simulation_tested": False,
        "note": "Reference/setup audit only; runtime behavior and screenshots are checked separately."
    }


try:
    details = {} if VERIFY else install()
    result = verify()
    result.update(details)
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    unreal.log("FACTORY_RUNTIME_SETUP_OK " + str(REPORT))
except Exception:
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps({"success": False, "error": traceback.format_exc()},
                                 ensure_ascii=False, indent=2), encoding="utf-8")
    raise
