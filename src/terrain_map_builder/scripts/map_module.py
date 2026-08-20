#!/usr/bin/env python3
"""Create, configure, validate and activate named terrain-map modules."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import sys


MODULE_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")
NAVIGATION_FILES = (
    "traversability_map.yaml",
    "traversability_map.pgm",
    "terrain_cost.bin",
    "terrain_cost_coarse.bin",
)
PLANNER_FILES = (
    "traversability_map.yaml",
    "traversability_map.pgm",
    "elevation_map.dem",
    "obstacle_map.pgm",
    "terrain_cost.bin",
    "terrain_cost_coarse.bin",
)


def validate_name(name):
    if not MODULE_RE.fullmatch(name or ""):
        raise ValueError(
            "map module must match [A-Za-z0-9][A-Za-z0-9_-]*: {!r}".format(
                name))
    return name


def find_project_root(explicit=""):
    if explicit:
        return Path(explicit).expanduser().resolve()
    configured = os.environ.get("DESERT_TRACKER_ROOT", "").strip()
    if configured:
        return Path(configured).expanduser().resolve()
    try:
        import rospkg
        package = Path(rospkg.RosPack().get_path("terrain_map_builder"))
        return package.parent.parent.resolve()
    except Exception:
        source = Path(__file__).resolve()
        for parent in source.parents:
            if (parent / "src" / "terrain_map_builder" / "package.xml").is_file():
                return parent
    raise RuntimeError(
        "cannot locate project root; pass --project-root or set "
        "DESERT_TRACKER_ROOT")


def module_dir(project_root, module):
    return Path(project_root) / "maps" / validate_name(module)


def module_config_path(project_root, module):
    return module_dir(project_root, module) / "module.yaml"


def planner_dir(project_root, module, requested=""):
    expected = (module_dir(project_root, module) / "planner").resolve()
    if not requested:
        return expected
    actual = Path(requested).expanduser().resolve()
    if actual != expected:
        raise RuntimeError(
            "map_dir must belong to map module {!r}: expected {}, got {}".format(
                module, expected, actual))
    return actual


def _unquote(value):
    value = value.strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
        return value[1:-1]
    return value


def read_config(project_root, module):
    path = module_config_path(project_root, module)
    if not path.is_file():
        raise RuntimeError("missing map module config: {}".format(path))
    values = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = _unquote(value)
    return values


def _yaml_scalar(value):
    if value is None or str(value).strip() == "":
        return '""'
    text = str(value).strip()
    try:
        float(text)
        return text
    except ValueError:
        return json.dumps(text, ensure_ascii=False)


def write_config(project_root, module, values):
    path = module_config_path(project_root, module)
    path.parent.mkdir(parents=True, exist_ok=True)
    content = (
        "# Coordinate contract for this reusable map module.\n"
        "map_module: {}\n"
        "origin_lat: {}\n"
        "origin_lon: {}\n"
        "origin_alt: {}\n"
        "yaw_enu_to_oxyz_deg: {}\n"
    ).format(
        module,
        _yaml_scalar(values.get("origin_lat", "")),
        _yaml_scalar(values.get("origin_lon", "")),
        _yaml_scalar(values.get("origin_alt", "0")),
        _yaml_scalar(values.get("yaw_enu_to_oxyz_deg", "0")),
    )
    temporary = path.with_suffix(".yaml.tmp")
    with temporary.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(content)
    os.replace(str(temporary), str(path))
    return path


def create_module(project_root, module, args):
    root = module_dir(project_root, module)
    config_path = root / "module.yaml"
    if config_path.exists() and not args.force:
        raise RuntimeError(
            "map module already exists: {}; use configure or --force".format(
                module))
    for stage in ("raw", "aligned", "planner"):
        directory = root / stage
        directory.mkdir(parents=True, exist_ok=True)
        (directory / ".gitkeep").touch(exist_ok=True)
    bag_directory = Path(project_root) / "bags" / module
    bag_directory.mkdir(parents=True, exist_ok=True)
    (bag_directory / ".gitkeep").touch(exist_ok=True)
    for mode in ("qianxun", "lio_sam", "slam"):
        result_directory = Path(project_root) / "results" / module / mode
        result_directory.mkdir(parents=True, exist_ok=True)
        (result_directory / ".gitkeep").touch(exist_ok=True)
    values = {
        "origin_lat": args.origin_lat,
        "origin_lon": args.origin_lon,
        "origin_alt": args.origin_alt,
        "yaw_enu_to_oxyz_deg": args.yaw_enu_to_oxyz_deg,
    }
    write_config(project_root, module, values)
    return root


def configure_module(project_root, module, args):
    values = read_config(project_root, module)
    for key in ("origin_lat", "origin_lon", "origin_alt",
                "yaw_enu_to_oxyz_deg"):
        value = getattr(args, key)
        if value is not None:
            values[key] = value
    return write_config(project_root, module, values)


def _require_numeric(config, key, allow_empty=False):
    value = str(config.get(key, "")).strip()
    if not value and allow_empty:
        return
    if not value:
        raise RuntimeError("module.yaml is missing {}".format(key))
    try:
        float(value)
    except ValueError:
        raise RuntimeError("module.yaml {} is not numeric: {!r}".format(
            key, value))


def validate_module(project_root, module, stage="planner", map_dir=""):
    root = module_dir(project_root, module)
    config = read_config(project_root, module)
    if config.get("map_module", module) != module:
        raise RuntimeError(
            "module.yaml map_module={!r} does not match directory {!r}".format(
                config.get("map_module"), module))
    for key in ("origin_alt", "yaw_enu_to_oxyz_deg"):
        _require_numeric(config, key)

    if stage in ("qianxun", "lio_sam"):
        _require_numeric(config, "origin_lat")
        _require_numeric(config, "origin_lon")

    planner = planner_dir(project_root, module, map_dir)
    required = []
    if stage == "raw":
        required.append(root / "raw" / "GlobalMap.pcd")
    elif stage == "aligned":
        required.append(root / "aligned" / "GlobalMap.pcd")
    elif stage == "planner":
        required.extend(planner / name for name in NAVIGATION_FILES)
    elif stage in ("qianxun", "lio_sam"):
        # Full-system navigation starts the evaluator by default, so require
        # its DEM and obstacle layers in addition to move_base inputs.
        required.extend(planner / name for name in PLANNER_FILES)
        if stage == "lio_sam":
            required.append(root / "aligned" / "GlobalMap.pcd")
    elif stage != "module":
        raise RuntimeError("unknown validation stage: {}".format(stage))

    missing = [str(path) for path in required
               if not path.is_file() or path.stat().st_size <= 0]
    if missing:
        raise RuntimeError("map module is incomplete; missing: {}".format(
            ", ".join(missing)))
    return root, planner, config


def _quick_file_digest(path):
    path = Path(path)
    if not path.is_file():
        return {"path": str(path), "missing": True}
    size = path.stat().st_size
    digest = hashlib.sha256()
    digest.update(str(size).encode("ascii"))
    with path.open("rb") as stream:
        if size <= 64 * 1024 * 1024:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
            method = "sha256-full"
        else:
            digest.update(stream.read(1024 * 1024))
            stream.seek(max(0, size - 1024 * 1024))
            digest.update(stream.read(1024 * 1024))
            method = "sha256-size-first-last-1MiB"
    return {
        "path": str(path), "size": size, "digest": digest.hexdigest(),
        "method": method,
    }


def map_identity(project_root, module, map_dir=""):
    root = module_dir(project_root, module)
    planner = planner_dir(project_root, module, map_dir)
    files = {"module_config": _quick_file_digest(root / "module.yaml")}
    for name in PLANNER_FILES:
        files[name] = _quick_file_digest(planner / name)
    files["aligned_pcd"] = _quick_file_digest(
        root / "aligned" / "GlobalMap.pcd")
    combined = hashlib.sha256()
    for key in sorted(files):
        record = files[key]
        combined.update(key.encode("utf-8"))
        combined.update(str(record.get("digest", "missing")).encode("ascii"))
    return {
        "map_module": module,
        "module_dir": str(root),
        "planner_dir": str(planner),
        "fingerprint": combined.hexdigest(),
        "files": files,
    }


def list_modules(project_root):
    maps_root = Path(project_root) / "maps"
    if not maps_root.is_dir():
        return []
    return sorted(path.name for path in maps_root.iterdir()
                  if path.is_dir() and (path / "module.yaml").is_file())


def guard_module(project_root, module, stage, map_dir):
    root, planner, _ = validate_module(
        project_root, module, stage=stage, map_dir=map_dir)
    identity = map_identity(project_root, module, map_dir=str(planner))
    import rospy
    from std_msgs.msg import String
    rospy.init_node("map_module_guard")
    publisher = rospy.Publisher(
        "/map_module/active", String, queue_size=1, latch=True)
    payload = json.dumps(identity, sort_keys=True)
    publisher.publish(String(data=payload))
    rospy.loginfo(
        "Active map module=%s stage=%s planner=%s fingerprint=%s",
        module, stage, planner, identity["fingerprint"][:16])
    rospy.spin()


def save_module_map(project_root, module, resolution, service_name):
    validate_module(project_root, module, stage="module")
    destination = module_dir(project_root, module) / "raw"
    destination.mkdir(parents=True, exist_ok=True)
    import rospy
    from lio_sam.srv import save_map
    rospy.init_node("save_map_module", anonymous=True)
    rospy.wait_for_service(service_name, timeout=30.0)
    response = rospy.ServiceProxy(service_name, save_map)(
        float(resolution), str(destination) + os.sep)
    if not response.success:
        raise RuntimeError("LIO-SAM save_map service returned success=false")
    print(destination / "GlobalMap.pcd")


def add_coordinate_args(parser, defaults=False):
    default = "" if defaults else None
    parser.add_argument("--origin-lat", default=default)
    parser.add_argument("--origin-lon", default=default)
    parser.add_argument("--origin-alt", default="0" if defaults else None)
    parser.add_argument(
        "--yaw-enu-to-oxyz-deg", default="0" if defaults else None)


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", default="")
    sub = parser.add_subparsers(dest="command", required=True)

    create = sub.add_parser("create")
    create.add_argument("module")
    create.add_argument("--force", action="store_true")
    add_coordinate_args(create, defaults=True)

    configure = sub.add_parser("configure")
    configure.add_argument("module")
    add_coordinate_args(configure)

    validate = sub.add_parser("validate")
    validate.add_argument("module")
    validate.add_argument(
        "--stage", choices=("module", "raw", "aligned", "planner",
                            "qianxun", "lio_sam"), default="planner")
    validate.add_argument("--map-dir", default="")

    guard = sub.add_parser("guard")
    guard.add_argument("module")
    guard.add_argument(
        "--stage", choices=("module", "raw", "aligned", "planner",
                            "qianxun", "lio_sam"), default="planner")
    guard.add_argument("--map-dir", default="")

    path = sub.add_parser("path")
    path.add_argument("module")
    path.add_argument(
        "stage", nargs="?", choices=("module", "raw", "aligned", "planner"),
        default="module")

    sub.add_parser("list")

    fingerprint = sub.add_parser("fingerprint")
    fingerprint.add_argument("module")
    fingerprint.add_argument("--map-dir", default="")

    save = sub.add_parser("save")
    save.add_argument("module")
    save.add_argument("--resolution", type=float, default=0.2)
    save.add_argument("--service", default="/lio_sam/save_map")
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    try:
        project_root = find_project_root(args.project_root)
        if args.command == "create":
            print(create_module(project_root, validate_name(args.module), args))
        elif args.command == "configure":
            print(configure_module(
                project_root, validate_name(args.module), args))
        elif args.command == "validate":
            root, planner, _ = validate_module(
                project_root, validate_name(args.module), args.stage,
                args.map_dir)
            print("VALID {} {} {}".format(args.module, args.stage, planner))
        elif args.command == "guard":
            guard_module(project_root, validate_name(args.module), args.stage,
                         args.map_dir)
        elif args.command == "path":
            root = module_dir(project_root, validate_name(args.module))
            print(root if args.stage == "module" else root / args.stage)
        elif args.command == "list":
            for module in list_modules(project_root):
                print(module)
        elif args.command == "fingerprint":
            print(json.dumps(map_identity(
                project_root, validate_name(args.module), args.map_dir),
                indent=2, sort_keys=True))
        elif args.command == "save":
            save_module_map(project_root, validate_name(args.module),
                            args.resolution, args.service)
        return 0
    except Exception as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
