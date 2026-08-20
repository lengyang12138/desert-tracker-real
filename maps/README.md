# Named map modules

`sample_desert/` is copied from the simulation project only for build and
interface checks. It is not geometrically aligned with Qianxun coordinates and
must not be used for real driving.

A reusable map is one self-contained module:

```text
maps/<map_module>/
  module.yaml
  raw/GlobalMap.pcd
  aligned/GlobalMap.pcd
  planner/traversability_map.yaml
  planner/traversability_map.pgm
  planner/elevation_map.dem
  planner/obstacle_map.pgm
  planner/terrain_cost.bin
  planner/terrain_cost_coarse.bin
```

`site_a` and `site_b` are ready-to-fill sibling modules. Each owns its own
`raw/`, `aligned/` and `planner/` stages; select one with the `map_module`
launch argument.

`module.yaml` stores the fixed Qianxun origin and O-XYZ axis rotation used by
that map.  A planner directory must contain:

- `traversability_map.yaml` and its referenced PGM;
- `elevation_map.dem` and `obstacle_map.pgm` for evaluation;
- `terrain_cost.bin` (TCM1 fine terrain layer);
- `terrain_cost_coarse.bin` (TCM1 coarse terrain layer).

The map origin and axes must use the same O-XYZ frame configured by
`QX_ORIGIN_LAT/LON/ALT` and `QX_YAW_ENU_TO_OXYZ_DEG`.

The intended LIO-SAM pipeline is deliberately two-stage so that alignment can
be reviewed before irreversible vehicle testing:

1. `align_lio_map.launch`: raw/saved LIO-SAM PCD -> O-XYZ aligned PCD;
2. `build_planner_map.launch`: aligned PCD -> occupancy, elevation and TCM1
   layers consumed by Hybrid A* and MPC.

Create and switch modules with:

```bash
rosrun terrain_map_builder map_module.py create dune_a --origin-lat 22.0 --origin-lon 114.0 --origin-alt 0.0 --yaw-enu-to-oxyz-deg 0.0
rosrun terrain_map_builder map_module.py list
rosrun terrain_map_builder map_module.py validate dune_a --stage qianxun
roslaunch vehicle_bringup full_system.launch map_module:=dune_a localization_mode:=qianxun
```

Mapping bags are isolated under `bags/<map_module>/`. Navigation results are
isolated under `results/<map_module>/<localization_mode>/`; each run also
contains a map fingerprint in its JSON/CSV metadata. The runtime guard rejects
a `map_dir` from a different module, so a run cannot silently use another
map's files under the selected module name.
