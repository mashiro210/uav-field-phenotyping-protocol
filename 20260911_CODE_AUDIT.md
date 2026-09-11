# Code audit — 2026-09-11

This audit covered every Python generator in the repository and checked the
implementation against the repository Wiki and DJI WPML 1.0.6 documentation.
The original dated files remain unchanged. Corrected copies use the
`20260911_` prefix.

## Corrected issues

- Added a shared, verified WGS84 ellipsoidal-height to EGM96 orthometric-height
  transformation. Ballpark and silent no-op transformations are rejected.
- Kept the required altitude split: `template.kml` writes WGS84 to
  `ellipsoidHeight` and EGM96 to `height`; `wayline.wpml` writes WGS84 to
  `executeHeight` with `executeHeightMode=WGS84`.
- Reprojected DSM access through `WarpedVRT` before sampling at WGS84
  longitude/latitude positions.
- Rejected nodata, non-finite, and placeholder DSM elevations instead of
  silently producing zero-height waypoints.
- Corrected the two area-mapping generators, which sampled projected rasters
  using untransformed longitude/latitude, wrote the same altitude under both
  vertical references, and contained a runtime `PM_t` name error in the M3M
  version.
- Changed metric route construction from Web Mercator to a locally estimated
  UTM CRS.
- Corrected the two line-route generators so each input LineString owns a
  separate interval-shooting action group. Photos no longer continue during
  transfers between independent lines.
- Preserved floating-point interval values instead of truncating values such
  as `0.7` seconds to integer zero.
- Added line labels to image suffixes and explicit local line headings using
  DJI's north-zero, east-positive, west-negative `[-180, 180]` convention.
- Completed missing wayline mission configuration, payload, route ID, turn,
  and M3M `useStraightLine` elements where required.
- Corrected the P1 dome generator's undefined `coord_height_mode` reference.
- Set POI altitude components to zero because DJI currently ignores vertical
  POI orientation.
- Replaced underscores in default mission names and mission IDs with hyphens.

## Files

- `20260911_generate_CCO_mission_terrain_follow_M3M.py`
- `20260911_generate_CCO_mission_terrain_follow_P1.py`
- `20260911_generate_custom_waypoint_mission_M3M.py`
- `20260911_generate_custom_waypoint_mission_P1.py`
- `20260911_generate_dome_mission_terrain_follow_M3M.py`
- `20260911_generate_dome_mission_terrain_follow_P1.py`
- `20260911_generate_flight_mission_terrain_follow_area_mapping_M3M.py`
- `20260911_generate_flight_mission_terrain_follow_area_mapping_P1.py`
- `20260911_generate_flight_mission_terrain_follow_line_route_M3M.py`
- `20260911_generate_flight_mission_terrain_follow_line_route_P1.py`
- `wpml_safety_20260911.py`
- `requirements-20260911.txt`

## Verification

- All revised files pass Python bytecode compilation.
- All revised files pass `pyflakes` with no findings.
- All ten generators completed a smoke test using generated Point,
  LineString, Polygon, and relative-height inputs.
- All ten resulting KMZ files contained `wpmz/template.kml` and
  `wpmz/waylines.wpml`, matching sequential waypoint indices and required
  mission configuration fields.
- The line-route smoke test produced two independent capture ranges (`0–7`
  and `8–15`) with labels `row-A` and `row-B`, rather than one capture range
  spanning the transfer.

Generated missions must still be inspected in DJI Pilot 2 before flight.
Low-altitude obstacle clearance, takeoff-point elevation, GNSS/RTK status, and
the actual vertical datum of every input DSM remain operational
responsibilities.
