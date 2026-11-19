"""Isolated R12-camera fixture for four original A1 points and bounded point deferral."""
import argparse
import json
from pathlib import Path
import tempfile
from xml.etree import ElementTree as XML

from common import APP, PKG, ROOT, canonical, dump, edition, read, sha
from geometry_r02 import write
from scene_spray_r10 import compose as formal_scene
from visual_r11 import apply, camera

SCRIPTS = PKG / "scripts"
BASE_MISSION = edition("R10") / "spray/mission.json"
RECORDER = edition("R12") / "runtime_spray_video.py"
EXPERIMENTS = APP / "results/experiments/spray_defer"
FOUR = ["point_11_20", "point_10_20", "point_9_20", "point_8_20"]
FIXTURE_SOURCES = [SCRIPTS / name for name in
                   ["spray_defer_fixture_r10.py", "spray_defer_policy_r10.py", "spray_defer_candidate_r10.py"]]
OBSTACLE_NAME = "temporary_equipment_crate"


def original_points():
    cfg = read(BASE_MISSION)
    cluster = next(c for c in cfg["clusters"] if c["id"] == "A1")
    ids = [p["id"] for p in cluster["points"]]
    i = ids.index(FOUR[0])
    if ids[i:i+4] != FOUR or len(ids) != 56 or len(set(ids)) != 56:
        raise ValueError("The four points must remain an original contiguous subset of 56 A1 obligations")
    return cfg, cluster["points"][i:i+4]


def fixture_scene(out, task, session, obstacle, experiment_version=None, **kwargs):
    if task != "spray" or kwargs.get("cluster", "A1") != "A1":
        raise ValueError("The deferral experiment uses original spray A1 only")
    cfg, points = original_points()
    config = formal_scene(out, task, session, **kwargs)
    version = experiment_version or canonical({"preview": True, "parent": cfg["version"], "crate": obstacle})
    tool = read(edition("R10") / "spray/tool.json")
    tool.pop("tool_revision")
    tool["plan_revision"] = version
    tool["tool_revision"] = canonical(tool)
    tool["session"] = session
    dump(out / "tool.json", tool)
    manager = read(edition("R10") / "spray/manager.json")
    manager["plan_revision"] = version
    dump(out / "manager.json", manager)
    source = out / "world.sdf"
    tree = XML.parse(source).getroot()
    plugin = tree.find("world/model[@name='vehicle']/plugin[@name='s10::ToolActuator']/config")
    if plugin is None:
        raise ValueError("The baseline physical tool feed is missing")
    plugin.text = json.dumps(tool)
    world = tree.find("world")
    vehicle = world.find("model[@name='vehicle']/pose")
    v = [float(x) for x in vehicle.text.split()]
    offset = cfg["map_offset"]
    first, target = points[:2]
    v[0], v[1] = [x+y for x, y in zip(first["target"], offset)]
    v[5] = first["yaw"]
    vehicle.text = " ".join(map(str, v))
    audit = dict(spawn=[v[0], v[1], v[5]], online_access=False,
                 coarse_operator_map_hint=[first["target"][0]+.5, first["target"][1]-.5, v[5]+.05],
                 selected_original_ids=FOUR, original_obligations=56, map_offset=offset,
                 candidate_version=version, candidate_tool_revision=tool["tool_revision"],
                 frozen_map_unchanged=True, obstacle_in_frozen_prior=False, effect_domain="Gazebo physics and rendering")
    if obstacle:
        if world.find(f"model[@name='{OBSTACLE_NAME}']") is not None:
            raise ValueError("Fixture equipment name collides with an existing model")
        x, y = [a+b for a, b in zip(target["target"], offset)]
        size = [.60, .60, .72]
        crate = XML.SubElement(world, "model", name=OBSTACLE_NAME)
        XML.SubElement(crate, "static").text = "true"
        XML.SubElement(crate, "pose").text = f"{x} {y} {.5*size[2]} 0 0 0"
        link = XML.SubElement(crate, "link", name="body")
        for kind in ("collision", "visual"):
            part = XML.SubElement(link, kind, name="crate_"+kind)
            shape = XML.SubElement(XML.SubElement(part, "geometry"), "box")
            XML.SubElement(shape, "size").text = " ".join(map(str, size))
            if kind == "visual":
                material = XML.SubElement(part, "material")
                XML.SubElement(material, "ambient").text = ".72 .28 .06 1"
                XML.SubElement(material, "diffuse").text = ".85 .35 .07 1"
        audit["physical_fixture"] = dict(name=OBSTACLE_NAME, world_xy=[x,y], map_xy=target["target"],
                                         size_xyz_m=size, static=True, overlay_only=True)
    else:
        audit["physical_fixture"] = None
    original = out / "untextured.sdf"
    write(original, tree)
    apply(original, source, task)
    camera(source, task)
    audit["world_sdf_sha256"] = sha(source)
    dump(out / "fixture.json", audit)
    dump(out / "recording_camera.json", dict(topic="/r12/observer/image", domain=120, autonomy_input=False,
                                            source="actual renderer", material_geometry_contract=str(out/"world.visual.json")))
    return config


def once(source, old, new):
    if source.count(old) != 1:
        raise ValueError("Unexpected R12 recording runtime structure at " + old[:64])
    return source.replace(old, new, 1)


def runtime_source():
    script = RECORDER.read_text()
    a, b = script.index("def compose("), script.index("def run(")
    script = script[:a] + script[b:]
    pairs = [
        ("APP/'results/R12'/(task+'_'+stamp())", "EXPERIMENTS/('spray_defer_'+stamp())"),
        ("dump(APP/'results/R12'/(task+'_'+case+'_latest.json')", "dump(EXPERIMENTS/latest_name"),
        ("spec['online_probe_sha256']=sha(PKG/'scripts/mission_spray_r10.py')",
         "spec.update(candidate_profile=case,formal_credit=False,candidate_selected_point_ids=selected_ids,"
         "original_obligations=56,online_probe_sha256=sha(worker))"),
        ("spec['implementation_bindings']=", "bound += [RECORDER,worker,*FIXTURE_SOURCES];spec['implementation_bindings']="),
        ("cfg=read((edition('R10')/'spray')/'mission.json')", "cfg=candidate_config()"),
        ("str(PKG/'scripts/mission_spray_r10.py'),'--out',str(out)])",
         "str(worker),'--out',str(out)],dict(env,PYTHONPATH=os.pathsep.join([str(PKG/'scripts'),env.get('PYTHONPATH','')])))"),
        ("str((edition('R10')/'spray')/'manager.json')", "str(out/'scene/manager.json')"),
        ("spec.update(rig_revision=cfg['rig']['revision']",
         "spec.update(candidate_scene_sha256=sha(out/'scene/world.sdf'),candidate_mission_version=cfg['version'],"
         "rig_revision=cfg['rig']['revision']")
    ]
    for before, after in pairs:
        script = once(script, before, after)
    return script


def build_runtime(candidate_mission: Path, *, obstacle=True):
    worker = candidate_mission.resolve()
    worker.relative_to(ROOT)
    if not all(p.is_file() for p in [worker, *FIXTURE_SOURCES]):
        raise ValueError("Build the candidate dispatcher and its policy sources before starting")
    profile = "bounded_defer" if obstacle else "bounded_defer_natural"
    implementation = {str(p.relative_to(ROOT)): sha(p) for p in [worker, RECORDER, *FIXTURE_SOURCES]}

    def candidate_config():
        cfg, _ = original_points()
        baseline = cfg.pop("version")
        cfg["bounded_defer_experiment"] = dict(parent_version=baseline, selected_ids=FOUR, formal_credit=False,
                                               fixture_profile=profile, bound_implementation=implementation)
        cfg["version"] = canonical(cfg)
        return cfg

    def scene(out, task, session, **kwargs):
        return fixture_scene(out, task, session, obstacle, candidate_config()["version"], **kwargs)

    scope = dict(__file__=__file__, __name__="spray_defer_runtime", compose=scene, candidate_config=candidate_config,
                 EXPERIMENTS=EXPERIMENTS, latest_name="latest.json" if obstacle else "latest_natural.json",
                 selected_ids=list(FOUR), worker=worker, RECORDER=RECORDER, FIXTURE_SOURCES=FIXTURE_SOURCES)
    exec(compile(runtime_source(), str(RECORDER), "exec"), scope)

    def run(*, seconds=300, rtf=.6):
        if not 60 <= seconds <= 420 or not 0 < rtf <= 1:
            raise ValueError("Expected 60–420 simulation seconds and a real-time factor in (0,1]")
        if any(sha(ROOT / name) != expected for name, expected in implementation.items()):
            raise ValueError("Candidate sources changed since binding")
        return scope["run"](task="spray", seconds=seconds, rtf=rtf, cluster="A1", point_limit=4, case=profile)

    return run


def offline_check():
    import copy
    import math
    import numpy as np
    import yaml
    from PIL import Image
    from scipy.ndimage import distance_transform_edt

    cfg, points = original_points()
    trees, audit = [], []
    with tempfile.TemporaryDirectory(prefix="spray-defer-check-") as temporary:
        for enabled in (False, True):
            out = Path(temporary) / str(enabled)
            fixture_scene(out, "spray", "offline_fixture_preview", enabled, rtf=.6, cluster="A1")
            tree = XML.parse(out/"world.sdf")
            tool = read(out/"tool.json")
            records = read(out/"fixture.json")
            plugged = json.loads(tree.find("world/model[@name='vehicle']/plugin[@name='s10::ToolActuator']/config").text)
            if not (plugged == tool and tool["plan_revision"] == read(out/"manager.json")["plan_revision"] == records["candidate_version"]):
                raise ValueError("Runtime manager, actuator and ledger version binding mismatch")
            check = copy.deepcopy(tool)
            check.pop("tool_revision")
            check.pop("session")
            if canonical(check) != tool["tool_revision"]:
                raise ValueError("Invalid version for physical work feedback")
            if len(tree.findall("world/model/link/sensor[@name='r12_observer']")) != 1:
                raise ValueError("The isolated R12 recording camera is missing or duplicated")
            crate = tree.find(f"world/model[@name='{OBSTACLE_NAME}']")
            if (crate is not None) != enabled:
                raise ValueError("Unexpected static fixture count")
            if crate is not None:
                if crate.find("static").text != "true" or crate.find("link/collision/geometry/box/size").text != crate.find("link/visual/geometry/box/size").text:
                    raise ValueError("Check visibility, size and physical contact geometry")
                tree.find("world").remove(crate)
            tree.find("world/model[@name='vehicle']/plugin[@name='s10::ToolActuator']/config").text = "variant-version"
            trees.append(XML.canonicalize(XML.tostring(tree.getroot(), encoding="unicode"), strip_text=True))
            audit.append(records)
    if trees[0] != trees[1] or audit[0]["candidate_version"] == audit[1]["candidate_version"]:
        raise ValueError("Natural and obstacle scenarios unexpectedly differ, or share a plan version")

    # Check a diagnostic detour geometrically in the declared prior. These points are
    # never used by the worker, controller, planner or obstacle sensors.
    prior_dir = edition("R07")
    grid = yaml.safe_load((prior_dir/"prior.yaml").read_text())
    step = grid["resolution"]
    pixels = np.asarray(Image.open(prior_dir/"prior.pgm"))
    clear = distance_transform_edt((255-pixels)/255 < grid["free_thresh"]) * step
    x0, y0, yaw = grid["origin"]
    if yaw != 0:
        raise ValueError("Expected the recorded unrotated A work-area raster")
    xmin, ymin = points[1]["target"]
    waypoints = [points[0]["target"], [points[0]["target"][0], ymin-1.5],
                 [points[2]["target"][0], ymin-1.5], points[2]["target"], points[3]["target"]]
    min_border, min_crate = math.inf, math.inf
    radius = max(math.hypot(*xy) for xy in cfg["vehicle"]["footprint"]) + cfg["footprint_margin_m"]
    for start, finish in zip(waypoints, waypoints[1:]):
        n = math.ceil(math.dist(start, finish)/.05)
        for i in range(n+1):
            x, y = [a+(b-a)*i/n for a,b in zip(start,finish)]
            row = pixels.shape[0]-1-math.floor((y-y0)/step)
            col = math.floor((x-x0)/step)
            if not (0<=row<pixels.shape[0] and 0<=col<pixels.shape[1]):
                raise ValueError("Offline diagnostic detour leaves the navigation map")
            min_border = min(min_border, clear[row,col]-math.sqrt(2)*step-.025)
            min_crate = min(min_crate, math.hypot(max(0, abs(x-xmin)-.3), max(0, abs(y-ymin)-.3))-.025)
    if min(min_border,min_crate) <= radius:
        raise ValueError("The original four-point scenario has no verified diagnostic side clearance")
    compile(runtime_source(), str(RECORDER), "exec")
    return dict(passed=True, scope="Offline SDF and static planning-grid check only; runtime navigation and work require independent evidence",
                formal_credit=False, gazebo_started=False, original_obligations=56, selected_original_ids=FOUR,
                source_r12_sha256=sha(RECORDER), source_mission_sha256=sha(BASE_MISSION), fixture_source_sha256=sha(Path(__file__)),
                natural_and_injected_other_geometry_identical=True, camera_present_both=True,
                tool_manager_plugin_version_consistent=True, profiles_have_separate_plan_versions=True,
                spawn_world=audit[0]["spawn"], coarse_map_hint=audit[0]["coarse_operator_map_hint"], physical_fixture=audit[1]["physical_fixture"],
                detour_witness_map_m=waypoints, footprint_circumradius_plus_margin_m=radius, grid_resolution_m=step,
                conservative_grid_border_clearance_m=float(min_border), conservative_crate_clearance_m=min_crate,
                online_has_fixture_waypoints=False, actual_nav2_path_unverified=True)



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    cmd = parser.add_subparsers(dest="operation", required=True)
    check = cmd.add_parser("inspect", help="Build and inspect the SDF only. No simulator or ROS processes.")
    check.add_argument("--natural", action="store_true")
    cmd.add_parser("check", help="Compare both generated worlds and check a static-grid detour. No simulator.")
    execute = cmd.add_parser("run")
    execute.add_argument("--candidate-mission", required=True, type=Path)
    execute.add_argument("--seconds", default=300, type=int)
    execute.add_argument("--rtf", default=.6, type=float)
    execute.add_argument("--natural", action="store_true")
    args = parser.parse_args()
    if args.operation == "inspect":
        with tempfile.TemporaryDirectory(prefix="spray-defer-fixture-") as temporary:
            out = Path(temporary)
            fixture_scene(out, "spray", "offline_fixture_preview", not args.natural, rtf=.6, cluster="A1")
            compile(runtime_source(), str(RECORDER), "exec")
            print(json.dumps(read(out/"fixture.json"), indent=2))
    elif args.operation == "check":
        print(json.dumps(offline_check(), indent=2))
    else:
        build_runtime(args.candidate_mission, obstacle=not args.natural)(seconds=args.seconds, rtf=args.rtf)
