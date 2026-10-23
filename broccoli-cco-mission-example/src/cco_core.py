"""Shared CCO geometry, terrain sampling and DJI WPML export routines.

Use generate_mission.py as the entry point for the four-plant example.
"""
#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import time
import zipfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import geopandas as gpd
import rasterio
from pyproj import CRS, Transformer
from pyproj import network as proj_network
from pyproj.transformer import TransformerGroup
from shapely.geometry import Point, Polygon, box, mapping


KML_NS = "http://www.opengis.net/kml/2.2"
WPML_NS = "http://www.dji.com/wpmz/1.0.6"
ET.register_namespace("", KML_NS)
ET.register_namespace("wpml", WPML_NS)

WORKING_EPSG = 32654
TARGET_OVERLAP = 0.80
COVERAGE_MARGIN_M = 0.10
MIN_TOP_RAY_DEPRESSION_DEG = 5.0
PHOTO_SPEED_MPS = 1.0
TRANSIT_SPEED_MPS = 3.0
TRANSIT_AGL_M = 8.0
TAKEOFF_SECURITY_HEIGHT_M = 20.0
SHOT_MARGIN_SEC = 0.10

# DJI official camera specifications.
P1_SENSOR_WIDTH_MM = 35.9
P1_SENSOR_HEIGHT_MM = 24.0
P1_FOCAL_LENGTH_MM = 35.0
P1_MIN_INTERVAL_SEC = 0.7
M3M_RGB_DIAGONAL_FOV_DEG = 84.0
M3M_RGB_WIDTH_PX = 5280
M3M_RGB_HEIGHT_PX = 3956
M3M_MS_HORIZONTAL_FOV_DEG = 61.2
M3M_MS_VERTICAL_FOV_DEG = 48.10
M3M_MS_MIN_INTERVAL_SEC = 2.0

# Radius lower bounds agreed for this task.  Radius is increased, never
# decreased, until every four-plant target has the requested footprint margin.
M400_MIN_RADIUS_M = 2.0
M3M_MIN_RADIUS_M = 1.2
RADIUS_INCREMENT_M = 0.05
MAX_RADIUS_M = 8.0


XY = Tuple[float, float]
XYZ = Tuple[float, float, float]


@dataclass(frozen=True)
class Sensor:
    name: str
    horizontal_fov_deg: float
    vertical_fov_deg: float


@dataclass
class TargetGroup:
    group_id: str
    area_id: str
    center_xy: XY
    plant_ids: List[str]
    plant_points: List[Point]
    target_dsm_wgs84_m: float = 0.0


@dataclass(frozen=True)
class Scenario:
    key: str
    mission_name: str
    filename_stem: str
    aircraft: str
    payload: str
    drone_enum: str
    drone_sub_enum: str
    payload_enum: str
    payload_sub_enum: Optional[str]
    image_format: str
    agl_m: float
    minimum_radius_m: float
    minimum_interval_sec: float
    sensors: Tuple[Sensor, ...]
    area_id: Optional[str] = None
    fixed_gimbal_pitch_deg: Optional[float] = None


@dataclass
class PhotoGeometry:
    group_id: str
    sequence: int
    angle_deg: float
    camera_xy: XY
    camera_ground_wgs84_m: float
    camera_wgs84_m: float
    target_wgs84_m: float
    heading_deg: float
    pitch_deg: float
    footprints: Dict[str, Polygon]
    overlaps_to_next: Dict[str, float] = field(default_factory=dict)
    plant_margin_m: Dict[str, float] = field(default_factory=dict)


@dataclass
class MissionWaypoint:
    index: int
    kind: str
    group_id: str
    xy: XY
    lonlat: XY
    dsm_wgs84_m: float
    wgs84_height_m: float
    egm96_height_m: float
    speed_mps: float
    heading_deg: float
    pitch_deg: float
    photo_sequence: Optional[int] = None
    first_photo_in_group: bool = False
    file_suffix: Optional[str] = None


class DsmSampler:
    def __init__(self, path: Path, working_epsg: int):
        self.path = path
        self.dataset = rasterio.open(path)
        if self.dataset.count != 1:
            raise ValueError("DSM must have exactly one raster band.")
        if self.dataset.crs is None:
            raise ValueError("DSM has no horizontal CRS.")
        self.nodata = self.dataset.nodata
        source = CRS.from_epsg(working_epsg)
        target = CRS.from_user_input(self.dataset.crs)
        self.transformer = None if source == target else Transformer.from_crs(
            source, target, always_xy=True
        )
        self.cache: Dict[Tuple[int, int], float] = {}

    def close(self) -> None:
        self.dataset.close()

    def sample(self, x: float, y: float) -> float:
        if self.transformer is not None:
            x, y = self.transformer.transform(x, y)
        row, col = self.dataset.index(x, y)
        key = (row, col)
        if key not in self.cache:
            if not (0 <= row < self.dataset.height and 0 <= col < self.dataset.width):
                raise ValueError("Coordinate lies outside DSM: %.3f, %.3f" % (x, y))
            value = float(self.dataset.read(1, window=((row, row + 1), (col, col + 1)))[0, 0])
            if not math.isfinite(value) or (
                self.nodata is not None and math.isclose(value, float(self.nodata))
            ):
                raise ValueError("Invalid DSM value at %.3f, %.3f" % (x, y))
            self.cache[key] = value
        return self.cache[key]




def p1_fovs() -> Tuple[float, float]:
    return (
        math.degrees(2.0 * math.atan(P1_SENSOR_WIDTH_MM / (2.0 * P1_FOCAL_LENGTH_MM))),
        math.degrees(2.0 * math.atan(P1_SENSOR_HEIGHT_MM / (2.0 * P1_FOCAL_LENGTH_MM))),
    )


def rgb_fovs_from_diagonal() -> Tuple[float, float]:
    tangent = math.tan(math.radians(M3M_RGB_DIAGONAL_FOV_DEG / 2.0))
    diagonal = math.hypot(M3M_RGB_WIDTH_PX, M3M_RGB_HEIGHT_PX)
    return (
        math.degrees(2.0 * math.atan(tangent * M3M_RGB_WIDTH_PX / diagonal)),
        math.degrees(2.0 * math.atan(tangent * M3M_RGB_HEIGHT_PX / diagonal)),
    )


def scenarios() -> Tuple[Scenario, ...]:
    p1_h, p1_v = p1_fovs()
    rgb_h, rgb_v = rgb_fovs_from_diagonal()
    p1 = (Sensor("P1-RGB", p1_h, p1_v),)
    m3m = (
        Sensor("M3M-RGB", rgb_h, rgb_v),
        Sensor("M3M-MS", M3M_MS_HORIZONTAL_FOV_DEG, M3M_MS_VERTICAL_FOV_DEG),
    )
    return (
        Scenario(
            "m400-p1-1m",
            "2026Broccoli-M400-P1-35mm-CCO-1m-80pct",
            "2026Broccoli-M400-P1-35mm-CCO-1m-80pct",
            "DJI Matrice 400",
            "Zenmuse P1 35 mm",
            "103", "0", "50", "1", "wide",
            1.0, M400_MIN_RADIUS_M, P1_MIN_INTERVAL_SEC, p1,
        ),
        Scenario(
            "m400-p1-3m",
            "2026Broccoli-M400-P1-35mm-CCO-3m-80pct",
            "2026Broccoli-M400-P1-35mm-CCO-3m-80pct",
            "DJI Matrice 400",
            "Zenmuse P1 35 mm",
            "103", "0", "50", "1", "wide",
            3.0, M400_MIN_RADIUS_M, P1_MIN_INTERVAL_SEC, p1,
        ),
        Scenario(
            "m400-p1-5m",
            "2026Broccoli-M400-P1-35mm-CCO-5m-80pct",
            "2026Broccoli-M400-P1-35mm-CCO-5m-80pct",
            "DJI Matrice 400",
            "Zenmuse P1 35 mm",
            "103", "0", "50", "1", "wide",
            5.0, M400_MIN_RADIUS_M, P1_MIN_INTERVAL_SEC, p1,
        ),
        Scenario(
            "m3m-rgbms-1m-jk",
            "2026Broccoli-M3M-RGBMS-JK-CCO-1m-80pct",
            "2026Broccoli-M3M-RGBMS-JK-CCO-1m-80pct",
            "DJI Mavic 3 Multispectral",
            "Integrated RGB + G/R/RE/NIR",
            "77", "2", "68", None, "wide,narrow_band",
            1.0, M3M_MIN_RADIUS_M, M3M_MS_MIN_INTERVAL_SEC, m3m,
            "JK_block2_4rows", -45.0,
        ),
        Scenario(
            "m3m-rgbms-1m-pq",
            "2026Broccoli-M3M-RGBMS-PQ-CCO-1m-80pct",
            "2026Broccoli-M3M-RGBMS-PQ-CCO-1m-80pct",
            "DJI Mavic 3 Multispectral",
            "Integrated RGB + G/R/RE/NIR",
            "77", "2", "68", None, "wide,narrow_band",
            1.0, M3M_MIN_RADIUS_M, M3M_MS_MIN_INTERVAL_SEC, m3m,
            "PQ_block2_4rows", -45.0,
        ),
    )


def normalize_heading(angle: float) -> float:
    return ((angle + 180.0) % 360.0) - 180.0


def fmt(value: float) -> str:
    return ("%.8f" % value).rstrip("0").rstrip(".")


def qname(namespace: str, tag: str) -> str:
    return "{%s}%s" % (namespace, tag)


def kml(parent: ET.Element, tag: str, text: Any = None) -> ET.Element:
    element = ET.SubElement(parent, qname(KML_NS, tag))
    if text is not None:
        element.text = str(text)
    return element


def wpml(parent: ET.Element, tag: str, text: Any = None) -> ET.Element:
    element = ET.SubElement(parent, qname(WPML_NS, tag))
    if text is not None:
        element.text = str(text)
    return element


def load_groups(center_path: Path, plant_path: Path) -> List[TargetGroup]:
    centers = gpd.read_file(center_path)
    plants = gpd.read_file(plant_path)
    if centers.crs is None or plants.crs is None:
        raise ValueError("Center and plant GeoJSON files must have CRS metadata.")
    centers = centers.to_crs(epsg=WORKING_EPSG)
    plants = plants.to_crs(epsg=WORKING_EPSG)
    if "plant_id" not in plants.columns:
        raise ValueError("Plant GeoJSON has no plant_id field.")
    plant_by_id = plants.set_index("plant_id", drop=False)
    groups: List[TargetGroup] = []
    for _, row in centers.iterrows():
        ids = [item.strip() for item in str(row["plant_ids"]).split(",") if item.strip()]
        if len(ids) != 4 or len(set(ids)) != 4:
            raise ValueError("%s does not contain four unique plant IDs." % row["group_id"])
        missing = [plant_id for plant_id in ids if plant_id not in plant_by_id.index]
        if missing:
            raise ValueError("Missing plant IDs for %s: %s" % (row["group_id"], missing))
        points = [plant_by_id.loc[plant_id].geometry for plant_id in ids]
        if not all(isinstance(point, Point) for point in points):
            raise ValueError("All plant geometries must be points.")
        groups.append(
            TargetGroup(
                str(row["group_id"]),
                str(row["area_id"]),
                (float(row.geometry.x), float(row.geometry.y)),
                ids,
                points,
            )
        )
    if len(groups) != 24:
        raise ValueError("Expected 24 target groups, found %d." % len(groups))
    return order_groups(groups)


def order_groups(groups: Sequence[TargetGroup]) -> List[TargetGroup]:
    """Greedy short path within JK then PQ, starting from the south."""
    result: List[TargetGroup] = []
    area_order = ["JK_block2_4rows", "PQ_block2_4rows"]
    leftovers = list(groups)
    for area in area_order:
        remaining = [group for group in leftovers if group.area_id == area]
        if not remaining:
            continue
        current = min(remaining, key=lambda group: (group.center_xy[1], group.center_xy[0]))
        while remaining:
            if current in remaining:
                remaining.remove(current)
                result.append(current)
            if not remaining:
                break
            current = min(
                remaining,
                key=lambda group: math.dist(result[-1].center_xy, group.center_xy),
            )
        leftovers = [group for group in leftovers if group.area_id != area]
    result.extend(sorted(leftovers, key=lambda group: (group.center_xy[1], group.center_xy[0])))
    return result


def vector_cross(a: XYZ, b: XYZ) -> XYZ:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def vector_unit(vector: XYZ) -> XYZ:
    length = math.sqrt(sum(value * value for value in vector))
    if length <= 0.0:
        raise ValueError("Cannot normalize a zero vector.")
    return tuple(value / length for value in vector)  # type: ignore[return-value]


def project_footprint(
    camera_xyz: XYZ,
    target_xyz: XYZ,
    sensor: Sensor,
    heading_deg: Optional[float] = None,
    pitch_deg: Optional[float] = None,
) -> Polygon:
    """Project a roll-zero landscape image onto the target horizontal plane."""
    if (heading_deg is None) != (pitch_deg is None):
        raise ValueError("heading_deg and pitch_deg must be supplied together.")
    if heading_deg is None:
        forward = vector_unit(tuple(target_xyz[i] - camera_xyz[i] for i in range(3)))
    else:
        heading = math.radians(float(heading_deg))
        pitch = math.radians(float(pitch_deg))
        forward = vector_unit(
            (
                math.sin(heading) * math.cos(pitch),
                math.cos(heading) * math.cos(pitch),
                math.sin(pitch),
            )
        )
    right = vector_unit(vector_cross(forward, (0.0, 0.0, 1.0)))
    image_up = vector_unit(vector_cross(right, forward))
    tan_h = math.tan(math.radians(sensor.horizontal_fov_deg / 2.0))
    tan_v = math.tan(math.radians(sensor.vertical_fov_deg / 2.0))
    corners: List[XY] = []
    # top-left, top-right, bottom-right, bottom-left
    for vertical_sign, horizontal_sign in ((1, -1), (1, 1), (-1, 1), (-1, -1)):
        ray = tuple(
            forward[i]
            + horizontal_sign * tan_h * right[i]
            + vertical_sign * tan_v * image_up[i]
            for i in range(3)
        )
        if ray[2] >= -1e-10:
            raise ValueError("Camera footprint includes or crosses the horizon.")
        scale = (target_xyz[2] - camera_xyz[2]) / ray[2]
        if scale <= 0.0:
            raise ValueError("Camera ray does not reach the target plane.")
        corners.append(
            (
                camera_xyz[0] + scale * ray[0],
                camera_xyz[1] + scale * ray[1],
            )
        )
    polygon = Polygon(corners)
    if not polygon.is_valid or polygon.area <= 0.0:
        raise ValueError("Invalid projected footprint polygon.")
    return polygon


def photo_geometry(
    group: TargetGroup,
    scenario: Scenario,
    radius_m: float,
    sequence: int,
    angle_deg: float,
    sampler: DsmSampler,
) -> PhotoGeometry:
    angle = math.radians(angle_deg)
    cx, cy = group.center_xy
    camera_xy = (cx + radius_m * math.cos(angle), cy + radius_m * math.sin(angle))
    camera_ground = sampler.sample(*camera_xy)
    camera_height = camera_ground + scenario.agl_m
    target_height = group.target_dsm_wgs84_m
    camera_xyz = (camera_xy[0], camera_xy[1], camera_height)
    target_xyz = (cx, cy, target_height)
    dz = camera_height - target_height
    if dz <= 0.0:
        raise ValueError("Camera is not above target plane for %s." % group.group_id)
    heading = normalize_heading(
        math.degrees(math.atan2(cx - camera_xy[0], cy - camera_xy[1]))
    )
    geometric_pitch = -math.degrees(math.atan2(dz, radius_m))
    pitch = (
        scenario.fixed_gimbal_pitch_deg
        if scenario.fixed_gimbal_pitch_deg is not None
        else geometric_pitch
    )
    for sensor in scenario.sensors:
        top_ray_depression = -pitch - sensor.vertical_fov_deg / 2.0
        if top_ray_depression + 1e-9 < MIN_TOP_RAY_DEPRESSION_DEG:
            raise ValueError(
                "%s top image ray is too close to the horizon (%.3f deg)."
                % (sensor.name, top_ray_depression)
            )
    footprints = {
        sensor.name: project_footprint(
            camera_xyz, target_xyz, sensor, heading_deg=heading, pitch_deg=pitch
        )
        for sensor in scenario.sensors
    }
    return PhotoGeometry(
        group.group_id,
        sequence,
        angle_deg,
        camera_xy,
        camera_ground,
        camera_height,
        target_height,
        heading,
        pitch,
        footprints,
    )


def minimum_plant_margin(polygon: Polygon, plants: Sequence[Point]) -> float:
    if not all(polygon.covers(plant) for plant in plants):
        return -1.0
    return min(plant.distance(polygon.exterior) for plant in plants)


def radius_is_valid(
    groups: Sequence[TargetGroup],
    scenario: Scenario,
    radius_m: float,
    coverage_margin_m: float,
    sampler: DsmSampler,
) -> Tuple[bool, float]:
    minimum_margin = math.inf
    # One-degree validation prevents a favorable discrete photo phase from
    # hiding a direction in which one of the four plants leaves the image.
    for group in groups:
        for angle_deg in range(360):
            try:
                geometry = photo_geometry(
                    group, scenario, radius_m, angle_deg + 1, float(angle_deg), sampler
                )
            except ValueError:
                return False, -1.0
            for polygon in geometry.footprints.values():
                margin = minimum_plant_margin(polygon, group.plant_points)
                minimum_margin = min(minimum_margin, margin)
                if margin + 1e-9 < coverage_margin_m:
                    return False, minimum_margin
    return True, minimum_margin


def choose_radius(
    groups: Sequence[TargetGroup],
    scenario: Scenario,
    coverage_margin_m: float,
    sampler: DsmSampler,
) -> Tuple[float, float]:
    steps = int(round((MAX_RADIUS_M - scenario.minimum_radius_m) / RADIUS_INCREMENT_M))
    for step in range(steps + 1):
        radius = round(scenario.minimum_radius_m + step * RADIUS_INCREMENT_M, 2)
        valid, margin = radius_is_valid(
            groups, scenario, radius, coverage_margin_m, sampler
        )
        if valid:
            # Operational radii are rounded upward to a clear 0.1 m setting.
            radius = math.ceil((radius - 1e-9) * 10.0) / 10.0
            valid, margin = radius_is_valid(
                groups, scenario, radius, coverage_margin_m, sampler
            )
            if valid:
                return radius, margin
    raise ValueError("No valid radius found for %s." % scenario.key)


def build_group_photo_geometries(
    groups: Sequence[TargetGroup],
    scenario: Scenario,
    radius_m: float,
    photos_per_orbit: int,
    sampler: DsmSampler,
) -> Dict[str, List[PhotoGeometry]]:
    by_group: Dict[str, List[PhotoGeometry]] = {}
    step = 360.0 / photos_per_orbit
    for group in groups:
        geometries = [
            photo_geometry(group, scenario, radius_m, sequence + 1, sequence * step, sampler)
            for sequence in range(photos_per_orbit)
        ]
        for index, geometry in enumerate(geometries):
            next_geometry = geometries[(index + 1) % len(geometries)]
            for sensor in scenario.sensors:
                polygon = geometry.footprints[sensor.name]
                next_polygon = next_geometry.footprints[sensor.name]
                geometry.overlaps_to_next[sensor.name] = (
                    polygon.intersection(next_polygon).area
                    / min(polygon.area, next_polygon.area)
                )
                geometry.plant_margin_m[sensor.name] = minimum_plant_margin(
                    polygon, group.plant_points
                )
        by_group[group.group_id] = geometries
    return by_group


def choose_photos_per_orbit(
    groups: Sequence[TargetGroup],
    scenario: Scenario,
    radius_m: float,
    target_overlap: float,
    coverage_margin_m: float,
    sampler: DsmSampler,
) -> Tuple[int, Dict[str, List[PhotoGeometry]]]:
    for count in range(4, 181):
        by_group = build_group_photo_geometries(
            groups, scenario, radius_m, count, sampler
        )
        minimum_overlap = min(
            overlap
            for geometries in by_group.values()
            for geometry in geometries
            for overlap in geometry.overlaps_to_next.values()
        )
        minimum_margin = min(
            margin
            for geometries in by_group.values()
            for geometry in geometries
            for margin in geometry.plant_margin_m.values()
        )
        if (
            minimum_overlap + 1e-9 >= target_overlap
            and minimum_margin + 1e-9 >= coverage_margin_m
        ):
            return count, by_group
    raise ValueError("Could not satisfy overlap with at most 180 photos per orbit.")


def build_vertical_transformer() -> Tuple[Any, str, float]:
    proj_network.set_network_enabled(True)
    group = TransformerGroup(
        CRS.from_epsg(4979),
        CRS.from_user_input("EPSG:4326+5773"),
        always_xy=True,
        allow_ballpark=False,
    )
    if not group.transformers or not group.best_available:
        raise ValueError(
            "A non-ballpark WGS84 ellipsoid to EGM96 transformation is unavailable."
        )
    transformer = group.transformers[0]
    return transformer, transformer.description, float(transformer.accuracy)


def add_waypoint(
    waypoints: List[MissionWaypoint],
    group_id: str,
    kind: str,
    xy: XY,
    dsm_m: float,
    agl_m: float,
    speed_mps: float,
    heading_deg: float,
    pitch_deg: float,
    to_wgs84: Transformer,
    photo_sequence: Optional[int] = None,
    first_photo: bool = False,
    file_suffix: Optional[str] = None,
) -> None:
    lon, lat = to_wgs84.transform(*xy)
    waypoints.append(
        MissionWaypoint(
            len(waypoints), kind, group_id, xy, (lon, lat), dsm_m,
            dsm_m + agl_m, 0.0, speed_mps, heading_deg, pitch_deg,
            photo_sequence, first_photo, file_suffix,
        )
    )


def build_mission_waypoints(
    groups: Sequence[TargetGroup],
    geometries: Dict[str, List[PhotoGeometry]],
    scenario: Scenario,
    transit_agl_m: float,
    photo_speed_mps: float,
    transit_speed_mps: float,
    sampler: DsmSampler,
) -> List[MissionWaypoint]:
    to_wgs84 = Transformer.from_crs(
        CRS.from_epsg(WORKING_EPSG), CRS.from_epsg(4326), always_xy=True
    )
    waypoints: List[MissionWaypoint] = []
    previous_closure: Optional[PhotoGeometry] = None
    for group_index, group in enumerate(groups):
        photos = geometries[group.group_id]
        first = photos[0]
        if previous_closure is not None:
            # The previous group's final waypoint is already at transit height.
            add_waypoint(
                waypoints, group.group_id, "transit-arrive-high", first.camera_xy,
                first.camera_ground_wgs84_m, transit_agl_m, transit_speed_mps,
                first.heading_deg, first.pitch_deg, to_wgs84,
            )
        for photo_index, photo in enumerate(photos):
            add_waypoint(
                waypoints, group.group_id, "photo", photo.camera_xy,
                photo.camera_ground_wgs84_m, scenario.agl_m, photo_speed_mps,
                photo.heading_deg, photo.pitch_deg, to_wgs84,
                photo.sequence, photo_index == 0,
                "%s-%03d" % (group.group_id, photo.sequence),
            )
        # Close the physical 360-degree orbit without duplicating the first image.
        add_waypoint(
            waypoints, group.group_id, "orbit-closure", first.camera_xy,
            first.camera_ground_wgs84_m, scenario.agl_m, photo_speed_mps,
            first.heading_deg, first.pitch_deg, to_wgs84,
        )
        # Climb vertically before any inter-group horizontal movement.
        add_waypoint(
            waypoints, group.group_id, "transit-depart-high", first.camera_xy,
            first.camera_ground_wgs84_m, transit_agl_m, transit_speed_mps,
            first.heading_deg, first.pitch_deg, to_wgs84,
        )
        previous_closure = first

    transformer, _, _ = build_vertical_transformer()
    lons = [waypoint.lonlat[0] for waypoint in waypoints]
    lats = [waypoint.lonlat[1] for waypoint in waypoints]
    heights = [waypoint.wgs84_height_m for waypoint in waypoints]
    _, _, orthometric = transformer.transform(lons, lats, heights)
    for waypoint, egm96 in zip(waypoints, orthometric):
        if not math.isfinite(float(egm96)):
            raise ValueError("EGM96 conversion failed at waypoint %d." % waypoint.index)
        waypoint.egm96_height_m = float(egm96)
        if abs(waypoint.wgs84_height_m - waypoint.egm96_height_m) < 0.01:
            raise ValueError("WGS84 and EGM96 heights unexpectedly match.")
    return waypoints


def add_mission_config(
    document: ET.Element,
    scenario: Scenario,
    transit_speed_mps: float,
) -> None:
    config = wpml(document, "missionConfig")
    wpml(config, "flyToWaylineMode", "safely")
    wpml(config, "finishAction", "goHome")
    wpml(config, "exitOnRCLost", "executeLostAction")
    wpml(config, "executeRCLostAction", "goBack")
    wpml(config, "takeOffSecurityHeight", fmt(TAKEOFF_SECURITY_HEIGHT_M))
    wpml(config, "globalTransitionalSpeed", fmt(transit_speed_mps))
    drone = wpml(config, "droneInfo")
    wpml(drone, "droneEnumValue", scenario.drone_enum)
    wpml(drone, "droneSubEnumValue", scenario.drone_sub_enum)
    payload = wpml(config, "payloadInfo")
    wpml(payload, "payloadEnumValue", scenario.payload_enum)
    if scenario.payload_sub_enum is not None:
        wpml(payload, "payloadSubEnumValue", scenario.payload_sub_enum)
    wpml(payload, "payloadPositionIndex", "0")


def add_payload_param(folder: ET.Element, scenario: Scenario) -> None:
    payload = wpml(folder, "payloadParam")
    wpml(payload, "payloadPositionIndex", "0")
    wpml(payload, "imageFormat", scenario.image_format)


def add_photo_action(
    placemark: ET.Element,
    waypoint: MissionWaypoint,
    settle_time_s: float,
) -> None:
    group = wpml(placemark, "actionGroup")
    wpml(group, "actionGroupId", waypoint.index)
    wpml(group, "actionGroupStartIndex", waypoint.index)
    wpml(group, "actionGroupEndIndex", waypoint.index)
    wpml(group, "actionGroupMode", "sequence")
    trigger = wpml(group, "actionTrigger")
    wpml(trigger, "actionTriggerType", "reachPoint")
    action_id = 0

    action = wpml(group, "action")
    wpml(action, "actionId", action_id)
    wpml(action, "actionActuatorFunc", "gimbalRotate")
    params = wpml(action, "actionActuatorFuncParam")
    wpml(params, "gimbalRotateMode", "absoluteAngle")
    wpml(params, "gimbalPitchRotateEnable", "1")
    wpml(params, "gimbalPitchRotateAngle", fmt(waypoint.pitch_deg))
    wpml(params, "gimbalRollRotateEnable", "0")
    wpml(params, "gimbalRollRotateAngle", "0")
    wpml(params, "gimbalYawRotateEnable", "0")
    wpml(params, "gimbalYawRotateAngle", "0")
    wpml(params, "gimbalRotateTimeEnable", "0")
    wpml(params, "gimbalRotateTime", "0")
    wpml(params, "payloadPositionIndex", "0")
    action_id += 1

    action = wpml(group, "action")
    wpml(action, "actionId", action_id)
    wpml(action, "actionActuatorFunc", "hover")
    params = wpml(action, "actionActuatorFuncParam")
    wpml(params, "hoverTime", fmt(settle_time_s))
    action_id += 1

    if waypoint.first_photo_in_group:
        action = wpml(group, "action")
        wpml(action, "actionId", action_id)
        wpml(action, "actionActuatorFunc", "focus")
        params = wpml(action, "actionActuatorFuncParam")
        wpml(params, "payloadPositionIndex", "0")
        wpml(params, "isPointFocus", "1")
        wpml(params, "focusX", "0.5")
        wpml(params, "focusY", "0.5")
        wpml(params, "isInfiniteFocus", "0")
        action_id += 1

        action = wpml(group, "action")
        wpml(action, "actionId", action_id)
        wpml(action, "actionActuatorFunc", "hover")
        params = wpml(action, "actionActuatorFuncParam")
        wpml(params, "hoverTime", "0.3")
        action_id += 1

    action = wpml(group, "action")
    wpml(action, "actionId", action_id)
    wpml(action, "actionActuatorFunc", "takePhoto")
    params = wpml(action, "actionActuatorFuncParam")
    wpml(params, "fileSuffix", waypoint.file_suffix)
    wpml(params, "payloadPositionIndex", "0")
    wpml(params, "useGlobalPayloadLensIndex", "1")


def add_waypoint_common(
    placemark: ET.Element,
    waypoint: MissionWaypoint,
    template: bool,
) -> None:
    point = kml(placemark, "Point")
    kml(point, "coordinates", "%.10f,%.10f" % waypoint.lonlat)
    wpml(placemark, "index", waypoint.index)
    if template:
        wpml(placemark, "ellipsoidHeight", fmt(waypoint.wgs84_height_m))
        wpml(placemark, "height", fmt(waypoint.egm96_height_m))
        wpml(placemark, "useGlobalHeight", "0")
        wpml(placemark, "useGlobalSpeed", "0")
    else:
        wpml(placemark, "executeHeight", fmt(waypoint.wgs84_height_m))
    wpml(placemark, "waypointSpeed", fmt(waypoint.speed_mps))
    if template:
        wpml(placemark, "useGlobalHeadingParam", "0")
        wpml(placemark, "useGlobalTurnParam", "0")
        wpml(placemark, "gimbalPitchAngle", fmt(waypoint.pitch_deg))
        wpml(placemark, "useStraightLine", "1")
    heading = wpml(placemark, "waypointHeadingParam")
    wpml(heading, "waypointHeadingMode", "smoothTransition")
    wpml(heading, "waypointHeadingAngle", fmt(waypoint.heading_deg))
    wpml(heading, "waypointHeadingPathMode", "followBadArc")
    if not template:
        wpml(heading, "waypointHeadingAngleEnable", "1")
    turn = wpml(placemark, "waypointTurnParam")
    wpml(turn, "waypointTurnMode", "toPointAndStopWithDiscontinuityCurvature")
    wpml(turn, "waypointTurnDampingDist", "0")
    if not template:
        wpml(placemark, "useStraightLine", "1")


def build_wpml_xml(
    waypoints: Sequence[MissionWaypoint],
    scenario: Scenario,
    settle_time_s: float,
    photo_speed_mps: float,
    transit_speed_mps: float,
    template: bool,
) -> ET.Element:
    root = ET.Element(qname(KML_NS, "kml"))
    document = kml(root, "Document")
    kml(document, "name", scenario.mission_name)
    if template:
        wpml(document, "author", "Codex / uav-field-phenotyping-protocol")
        timestamp = str(int(time.time() * 1000))
        wpml(document, "createTime", timestamp)
        wpml(document, "updateTime", timestamp)
    add_mission_config(document, scenario, transit_speed_mps)
    folder = kml(document, "Folder")
    if template:
        wpml(folder, "templateType", "waypoint")
    wpml(folder, "templateId", "0")
    if template:
        params = wpml(folder, "waylineCoordinateSysParam")
        wpml(params, "coordinateMode", "WGS84")
        wpml(params, "heightMode", "EGM96")
        wpml(params, "positioningType", "GPS")
    else:
        wpml(folder, "executeHeightMode", "WGS84")
        wpml(folder, "waylineId", "0")
    wpml(folder, "autoFlightSpeed", fmt(photo_speed_mps))
    if template:
        wpml(folder, "gimbalPitchMode", "usePointSetting")
        wpml(folder, "globalWaypointTurnMode", "toPointAndStopWithDiscontinuityCurvature")
    add_payload_param(folder, scenario)
    for waypoint in waypoints:
        placemark = kml(folder, "Placemark")
        add_waypoint_common(placemark, waypoint, template)
        if waypoint.kind == "photo":
            add_photo_action(placemark, waypoint, settle_time_s)
    return root


def xml_bytes(root: ET.Element) -> bytes:
    if hasattr(ET, "indent"):
        ET.indent(root, space="  ")
    return ET.tostring(root, encoding="utf-8", xml_declaration=True)


def export_kmz(path: Path, template: ET.Element, waylines: ET.Element) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("wpmz/template.kml", xml_bytes(template))
        archive.writestr("wpmz/waylines.wpml", xml_bytes(waylines))


def export_footprints(
    path: Path,
    groups: Sequence[TargetGroup],
    scenario: Scenario,
    radius_m: float,
    geometries: Dict[str, List[PhotoGeometry]],
) -> None:
    group_by_id = {group.group_id: group for group in groups}
    features: List[Dict[str, Any]] = []
    waypoint_no = 0
    for group in groups:
        for geometry in geometries[group.group_id]:
            waypoint_no += 1
            for sensor in scenario.sensors:
                polygon = geometry.footprints[sensor.name]
                features.append(
                    {
                        "type": "Feature",
                        "properties": {
                            "group_id": group.group_id,
                            "area_id": group.area_id,
                            "plant_ids": ",".join(group.plant_ids),
                            "photo_sequence": geometry.sequence,
                            "file_suffix": "%s-%03d" % (group.group_id, geometry.sequence),
                            "sensor": sensor.name,
                            "aircraft": scenario.aircraft,
                            "payload": scenario.payload,
                            "target_agl_m": scenario.agl_m,
                            "orbit_radius_m": radius_m,
                            "orbit_angle_deg": geometry.angle_deg,
                            "heading_deg": geometry.heading_deg,
                            "gimbal_pitch_deg": geometry.pitch_deg,
                            "camera_ground_dsm_wgs84_m": geometry.camera_ground_wgs84_m,
                            "camera_wgs84_height_m": geometry.camera_wgs84_m,
                            "target_dsm_wgs84_m": geometry.target_wgs84_m,
                            "horizontal_fov_deg": sensor.horizontal_fov_deg,
                            "vertical_fov_deg": sensor.vertical_fov_deg,
                            "footprint_area_m2": polygon.area,
                            "overlap_to_next_pct": geometry.overlaps_to_next[sensor.name] * 100.0,
                            "all_four_plants_covered": all(
                                polygon.covers(plant)
                                for plant in group_by_id[group.group_id].plant_points
                            ),
                            "minimum_plant_edge_margin_m": geometry.plant_margin_m[sensor.name],
                        },
                        "geometry": mapping(polygon),
                    }
                )
    collection = {
        "type": "FeatureCollection",
        "name": path.stem,
        "crs": {
            "type": "name",
            "properties": {"name": "urn:ogc:def:crs:EPSG::%d" % WORKING_EPSG},
        },
        "features": features,
    }
    with path.open("w", encoding="utf-8") as destination:
        json.dump(collection, destination, ensure_ascii=False, indent=2)
        destination.write("\n")


def export_route(path: Path, waypoints: Sequence[MissionWaypoint], scenario: Scenario) -> None:
    features = []
    for waypoint in waypoints:
        features.append(
            {
                "type": "Feature",
                "properties": {
                    "waypoint_index": waypoint.index,
                    "waypoint_no": waypoint.index + 1,
                    "kind": waypoint.kind,
                    "group_id": waypoint.group_id,
                    "photo_sequence": waypoint.photo_sequence,
                    "file_suffix": waypoint.file_suffix,
                    "speed_mps": waypoint.speed_mps,
                    "heading_deg": waypoint.heading_deg,
                    "gimbal_pitch_deg": waypoint.pitch_deg,
                    "dsm_wgs84_m": waypoint.dsm_wgs84_m,
                    "flight_wgs84_m": waypoint.wgs84_height_m,
                    "flight_egm96_m": waypoint.egm96_height_m,
                    "wgs84_minus_egm96_m": waypoint.wgs84_height_m - waypoint.egm96_height_m,
                    "photo_agl_m": scenario.agl_m if waypoint.kind == "photo" else None,
                },
                "geometry": {"type": "Point", "coordinates": list(waypoint.xy)},
            }
        )
    collection = {
        "type": "FeatureCollection",
        "name": path.stem,
        "crs": {
            "type": "name",
            "properties": {"name": "urn:ogc:def:crs:EPSG::%d" % WORKING_EPSG},
        },
        "features": features,
    }
    with path.open("w", encoding="utf-8") as destination:
        json.dump(collection, destination, ensure_ascii=False, indent=2)
        destination.write("\n")


def export_preview(
    path: Path,
    groups: Sequence[TargetGroup],
    scenario: Scenario,
    radius_m: float,
    geometries: Dict[str, List[PhotoGeometry]],
) -> None:
    """Render the worst-overlap consecutive pair for each sensor."""
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError as exc:
        raise ValueError("Preview rendering requires Pillow.") from exc

    panel_size = 900
    image = Image.new("RGB", (panel_size * len(scenario.sensors), panel_size), "white")
    try:
        font = ImageFont.truetype("/System/Library/Fonts/Supplemental/Arial.ttf", 24)
        small = ImageFont.truetype("/System/Library/Fonts/Supplemental/Arial.ttf", 19)
    except OSError:
        font = ImageFont.load_default()
        small = font

    def polygons(geometry: Any) -> List[Polygon]:
        if geometry.is_empty:
            return []
        if geometry.geom_type == "Polygon":
            return [geometry]
        return [item for item in geometry.geoms if item.geom_type == "Polygon"]

    for panel_index, sensor in enumerate(scenario.sensors):
        worst_group: Optional[TargetGroup] = None
        worst_index = 0
        worst_overlap = math.inf
        for group in groups:
            for index, geometry in enumerate(geometries[group.group_id]):
                overlap = geometry.overlaps_to_next[sensor.name]
                if overlap < worst_overlap:
                    worst_group = group
                    worst_index = index
                    worst_overlap = overlap
        if worst_group is None:
            raise ValueError("No preview geometry is available.")
        current = geometries[worst_group.group_id][worst_index]
        following = geometries[worst_group.group_id][
            (worst_index + 1) % len(geometries[worst_group.group_id])
        ]
        cx, cy = worst_group.center_xy
        half = max(3.0, radius_m + 1.0)
        clip = box(cx - half, cy - half, cx + half, cy + half)
        x0 = panel_index * panel_size

        def pixel(point: XY) -> Tuple[int, int]:
            px = x0 + int((point[0] - (cx - half)) / (2 * half) * (panel_size - 80)) + 40
            py = int(((cy + half) - point[1]) / (2 * half) * (panel_size - 150)) + 105
            return px, py

        overlay = Image.new("RGBA", image.size, (255, 255, 255, 0))
        draw_overlay = ImageDraw.Draw(overlay)
        for polygon in polygons(current.footprints[sensor.name].intersection(clip)):
            draw_overlay.polygon([pixel(pt) for pt in polygon.exterior.coords], fill=(33, 150, 243, 70), outline=(15, 95, 170, 255), width=4)
        for polygon in polygons(following.footprints[sensor.name].intersection(clip)):
            draw_overlay.polygon([pixel(pt) for pt in polygon.exterior.coords], fill=(255, 152, 0, 70), outline=(220, 95, 0, 255), width=4)
        image = Image.alpha_composite(image.convert("RGBA"), overlay).convert("RGB")
        draw = ImageDraw.Draw(image)
        orbit_points = [
            pixel((cx + radius_m * math.cos(math.radians(deg)), cy + radius_m * math.sin(math.radians(deg))))
            for deg in range(361)
        ]
        draw.line(orbit_points, fill=(110, 110, 110), width=3)
        for plant in worst_group.plant_points:
            px, py = pixel((plant.x, plant.y))
            draw.ellipse((px - 8, py - 8, px + 8, py + 8), fill=(20, 120, 30), outline="black", width=2)
        px, py = pixel(worst_group.center_xy)
        draw.ellipse((px - 7, py - 7, px + 7, py + 7), fill=(220, 0, 150), outline="black", width=2)
        for geometry, color in ((current, (15, 95, 170)), (following, (220, 95, 0))):
            px, py = pixel(geometry.camera_xy)
            draw.ellipse((px - 9, py - 9, px + 9, py + 9), fill=color, outline="black", width=2)
        draw.text((x0 + 35, 18), "%s / %s" % (scenario.key, sensor.name), fill="black", font=font)
        draw.text(
            (x0 + 35, 52),
            "%s  angles %.3f -> %.3f deg" % (
                worst_group.group_id, current.angle_deg, following.angle_deg
            ),
            fill="black", font=small,
        )
        draw.text(
            (x0 + 35, 77),
            "overlap %.3f%%  radius %.2f m  plant margin %.3f m" % (
                worst_overlap * 100.0,
                radius_m,
                min(current.plant_margin_m[sensor.name], following.plant_margin_m[sensor.name]),
            ),
            fill="black", font=small,
        )
        bar_start = pixel((cx - half + 0.3, cy - half + 0.3))
        bar_end = pixel((cx - half + 1.3, cy - half + 0.3))
        draw.line((bar_start, bar_end), fill="black", width=5)
        draw.text((bar_start[0], bar_start[1] - 28), "1 m", fill="black", font=small)
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)


def validate_kmz(
    path: Path,
    scenario: Scenario,
    expected_waypoints: int,
    expected_photos: int,
) -> None:
    with zipfile.ZipFile(path) as archive:
        members = set(archive.namelist())
        if members != {"wpmz/template.kml", "wpmz/waylines.wpml"}:
            raise ValueError("Unexpected KMZ members: %s" % sorted(members))
        template = ET.fromstring(archive.read("wpmz/template.kml"))
        waylines = ET.fromstring(archive.read("wpmz/waylines.wpml"))
        for root in (template, waylines):
            payload_enums = [
                item.text for item in root.iter(qname(WPML_NS, "payloadEnumValue"))
            ]
            if payload_enums != [scenario.payload_enum]:
                raise ValueError("Incorrect payloadEnumValue in %s." % path.name)
            payload_sub_enums = [
                item.text for item in root.iter(qname(WPML_NS, "payloadSubEnumValue"))
            ]
            expected_sub_enums = (
                [scenario.payload_sub_enum]
                if scenario.payload_sub_enum is not None
                else []
            )
            if payload_sub_enums != expected_sub_enums:
                raise ValueError("Incorrect payloadSubEnumValue in %s." % path.name)
            formats = [item.text for item in root.iter(qname(WPML_NS, "imageFormat"))]
            if formats != [scenario.image_format]:
                raise ValueError("Incorrect imageFormat in %s." % path.name)
            suffixes = [item.text for item in root.iter(qname(WPML_NS, "fileSuffix"))]
            if len(suffixes) != expected_photos or len(set(suffixes)) != expected_photos:
                raise ValueError("Photo suffix count/uniqueness validation failed.")
            actions = [
                item for item in root.iter(qname(WPML_NS, "actionActuatorFunc"))
                if item.text == "takePhoto"
            ]
            if len(actions) != expected_photos:
                raise ValueError("takePhoto count validation failed.")
            photo_action_groups = []
            for action_group in root.iter(qname(WPML_NS, "actionGroup")):
                action_names = [
                    item.text
                    for item in action_group.iter(
                        qname(WPML_NS, "actionActuatorFunc")
                    )
                ]
                if "takePhoto" in action_names:
                    photo_action_groups.append((action_group, action_names))
            if len(photo_action_groups) != expected_photos:
                raise ValueError("Photo action-group count validation failed.")
            for action_group, action_names in photo_action_groups:
                if action_names.count("takePhoto") != 1:
                    raise ValueError("Each photo action group must take one photo.")
                if action_names.count("gimbalRotate") != 1:
                    raise ValueError(
                        "Each photo action group must explicitly rotate the gimbal."
                    )
                if action_names.index("gimbalRotate") > action_names.index("takePhoto"):
                    raise ValueError("gimbalRotate must precede takePhoto.")
                if scenario.fixed_gimbal_pitch_deg is not None:
                    angles = [
                        float(item.text)
                        for item in action_group.iter(
                            qname(WPML_NS, "gimbalPitchRotateAngle")
                        )
                    ]
                    if angles != [scenario.fixed_gimbal_pitch_deg]:
                        raise ValueError(
                            "Fixed gimbal pitch validation failed for %s."
                            % path.name
                        )
        placemarks = list(waylines.iter(qname(KML_NS, "Placemark")))
        if len(placemarks) != expected_waypoints:
            raise ValueError("Waypoint count validation failed.")
        height_modes = [item.text for item in template.iter(qname(WPML_NS, "heightMode"))]
        execute_modes = [item.text for item in waylines.iter(qname(WPML_NS, "executeHeightMode"))]
        if height_modes != ["EGM96"] or execute_modes != ["WGS84"]:
            raise ValueError("WGS84/EGM96 height mode validation failed.")
        wgs = [float(item.text) for item in template.iter(qname(WPML_NS, "ellipsoidHeight"))]
        egm = [float(item.text) for item in template.iter(qname(WPML_NS, "height"))]
        execute = [float(item.text) for item in waylines.iter(qname(WPML_NS, "executeHeight"))]
        if len(wgs) != expected_waypoints or len(egm) != expected_waypoints:
            raise ValueError("Template height count validation failed.")
        if any(abs(a - b) < 0.01 for a, b in zip(wgs, egm)):
            raise ValueError("WGS84 and EGM96 heights must not match.")
        if any(abs(a - b) > 1e-6 for a, b in zip(wgs, execute)):
            raise ValueError("Template and waylines WGS84 heights differ.")
        if "_" in scenario.mission_name:
            raise ValueError("Mission name contains an underscore.")
        if scenario.fixed_gimbal_pitch_deg is not None:
            template_pitches = [
                float(item.text)
                for item in template.iter(qname(WPML_NS, "gimbalPitchAngle"))
            ]
            if not template_pitches or any(
                abs(value - scenario.fixed_gimbal_pitch_deg) > 1e-9
                for value in template_pitches
            ):
                raise ValueError("Template fixed gimbal pitch validation failed.")


def summarize_scenario(
    scenario: Scenario,
    radius_m: float,
    photos_per_orbit: int,
    geometries: Dict[str, List[PhotoGeometry]],
    waypoints: Sequence[MissionWaypoint],
    settle_time_s: float,
    photo_speed_mps: float,
    transit_agl_m: float,
    vertical_description: str,
    vertical_accuracy: float,
    paths: Dict[str, Path],
) -> Dict[str, Any]:
    overlap_by_sensor = {
        sensor.name: min(
            geometry.overlaps_to_next[sensor.name]
            for group_geometries in geometries.values()
            for geometry in group_geometries
        )
        for sensor in scenario.sensors
    }
    margin_by_sensor = {
        sensor.name: min(
            geometry.plant_margin_m[sensor.name]
            for group_geometries in geometries.values()
            for geometry in group_geometries
        )
        for sensor in scenario.sensors
    }
    footprint_area_by_sensor = {
        sensor.name: (
            min(
                geometry.footprints[sensor.name].area
                for group_geometries in geometries.values()
                for geometry in group_geometries
            ),
            max(
                geometry.footprints[sensor.name].area
                for group_geometries in geometries.values()
                for geometry in group_geometries
            ),
        )
        for sensor in scenario.sensors
    }
    pitches = [
        geometry.pitch_deg
        for group_geometries in geometries.values()
        for geometry in group_geometries
    ]
    photo_count = sum(1 for waypoint in waypoints if waypoint.kind == "photo")
    photo_waypoints = [waypoint for waypoint in waypoints if waypoint.kind == "photo"]
    orbit_distance = len(geometries) * 2.0 * math.pi * radius_m
    capture_seconds = orbit_distance / photo_speed_mps + photo_count * settle_time_s
    if photo_count:
        capture_seconds += len(geometries) * 0.3  # one focus-settle action per group
    route_distance_3d = 0.0
    route_travel_seconds = 0.0
    for start, end in zip(waypoints[:-1], waypoints[1:]):
        distance = math.sqrt(
            (end.xy[0] - start.xy[0]) ** 2
            + (end.xy[1] - start.xy[1]) ** 2
            + (end.wgs84_height_m - start.wgs84_height_m) ** 2
        )
        route_distance_3d += distance
        route_travel_seconds += distance / start.speed_mps
    total_seconds = (
        route_travel_seconds
        + photo_count * settle_time_s
        + len(geometries) * 0.3
    )
    return {
        "key": scenario.key,
        "mission_name": scenario.mission_name,
        "aircraft": scenario.aircraft,
        "payload": scenario.payload,
        "image_format": scenario.image_format,
        "area_id": scenario.area_id,
        "fixed_gimbal_pitch_deg": scenario.fixed_gimbal_pitch_deg,
        "photo_agl_m": scenario.agl_m,
        "transit_agl_m": transit_agl_m,
        "radius_m": radius_m,
        "photos_per_orbit": photos_per_orbit,
        "angle_step_deg": 360.0 / photos_per_orbit,
        "group_count": len(geometries),
        "photo_count": photo_count,
        "waypoint_count": len(waypoints),
        "settle_time_per_photo_s": settle_time_s,
        "minimum_overlap_pct": {
            name: value * 100.0 for name, value in overlap_by_sensor.items()
        },
        "minimum_plant_edge_margin_m": margin_by_sensor,
        "footprint_area_range_m2": {
            name: [bounds[0], bounds[1]]
            for name, bounds in footprint_area_by_sensor.items()
        },
        "minimum_top_ray_depression_deg": {
            sensor.name: min(-pitch - sensor.vertical_fov_deg / 2.0 for pitch in pitches)
            for sensor in scenario.sensors
        },
        "gimbal_pitch_range_deg": [min(pitches), max(pitches)],
        "dsm_range_m": [
            min(waypoint.dsm_wgs84_m for waypoint in waypoints),
            max(waypoint.dsm_wgs84_m for waypoint in waypoints),
        ],
        "flight_wgs84_range_m": [
            min(waypoint.wgs84_height_m for waypoint in waypoints),
            max(waypoint.wgs84_height_m for waypoint in waypoints),
        ],
        "flight_egm96_range_m": [
            min(waypoint.egm96_height_m for waypoint in waypoints),
            max(waypoint.egm96_height_m for waypoint in waypoints),
        ],
        "photo_flight_wgs84_range_m": [
            min(waypoint.wgs84_height_m for waypoint in photo_waypoints),
            max(waypoint.wgs84_height_m for waypoint in photo_waypoints),
        ],
        "photo_flight_egm96_range_m": [
            min(waypoint.egm96_height_m for waypoint in photo_waypoints),
            max(waypoint.egm96_height_m for waypoint in photo_waypoints),
        ],
        "vertical_transform": vertical_description,
        "vertical_transform_accuracy_m": vertical_accuracy,
        "estimated_capture_operation_seconds_excluding_transits": capture_seconds,
        "estimated_route_distance_3d_m": route_distance_3d,
        "estimated_total_seconds_excluding_takeoff_and_rth": total_seconds,
        "outputs": {name: str(path) for name, path in paths.items()},
    }


def write_report(path: Path, report: Dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as destination:
        json.dump(report, destination, ensure_ascii=False, indent=2)
        destination.write("\n")




