"""Mavic 3M configuration and CCO waypoint/action assembly helpers."""

def mission_scenarios(module):
    rgb_h, rgb_v = module.rgb_fovs_from_diagonal()
    sensors = (
        module.Sensor("M3M-RGB", rgb_h, rgb_v),
        module.Sensor(
            "M3M-MS",
            module.M3M_MS_HORIZONTAL_FOV_DEG,
            module.M3M_MS_VERTICAL_FOV_DEG,
        ),
    )
    return (
        module.Scenario(
            "m3m-rgbms-1p5m",
            "M3M-CCO-150cm",
            "M3M-CCO-150cm",
            "DJI Mavic 3 Multispectral",
            "Integrated RGB + G/R/RE/NIR",
            "77", "2", "68", None, "wide,narrow_band",
            1.5, 1.5, module.M3M_MS_MIN_INTERVAL_SEC, sensors,
            None, -45.0,
        ),
    )


def waypoint_name(waypoint):
    return "%03d" % (waypoint.index + 1)


def add_waypoint_names(module, root, waypoints):
    placemarks = list(root.iter(module.qname(module.KML_NS, "Placemark")))
    if len(placemarks) != len(waypoints):
        raise ValueError("Waypoint and Placemark counts differ.")
    for placemark, waypoint in zip(placemarks, waypoints):
        name = module.ET.Element(module.qname(module.KML_NS, "name"))
        name.text = waypoint_name(waypoint)
        placemark.insert(0, name)


def build_same_agl_no_climb_waypoints(
    module,
    groups,
    geometries,
    scenario,
    photo_speed_mps,
    transit_speed_mps,
):
    to_wgs84 = module.Transformer.from_crs(
        module.CRS.from_epsg(module.WORKING_EPSG),
        module.CRS.from_epsg(4326),
        always_xy=True,
    )
    waypoints = []

    def append_waypoint(
        group_id, kind, geometry, speed_mps, photo_sequence=None,
        first_photo=False, file_suffix=None,
    ):
        lon, lat = to_wgs84.transform(*geometry.camera_xy)
        waypoints.append(
            module.MissionWaypoint(
                len(waypoints),
                kind,
                group_id,
                geometry.camera_xy,
                (lon, lat),
                geometry.camera_ground_wgs84_m,
                geometry.camera_ground_wgs84_m + scenario.agl_m,
                0.0,
                speed_mps,
                geometry.heading_deg,
                geometry.pitch_deg,
                photo_sequence,
                first_photo,
                file_suffix,
            )
        )

    for group_index, group in enumerate(groups):
        photos = geometries[group.group_id]
        first = photos[0]
        for photo_index, photo in enumerate(photos):
            append_waypoint(
                group.group_id,
                "photo",
                photo,
                photo_speed_mps,
                photo.sequence,
                photo_index == 0,
                "%s-%03d" % (group.group_id, photo.sequence),
            )
        append_waypoint(
            group.group_id, "orbit-closure", first, photo_speed_mps
        )
        if group_index < len(groups) - 1:
            # Same coordinate and height; only the horizontal travel speed changes.
            append_waypoint(
                group.group_id, "transit-depart-level", first, transit_speed_mps
            )

    transformer, _, _ = module.build_vertical_transformer()
    lons = [waypoint.lonlat[0] for waypoint in waypoints]
    lats = [waypoint.lonlat[1] for waypoint in waypoints]
    heights = [waypoint.wgs84_height_m for waypoint in waypoints]
    _, _, orthometric = transformer.transform(lons, lats, heights)
    for waypoint, egm96 in zip(waypoints, orthometric):
        waypoint.egm96_height_m = float(egm96)
        if abs(waypoint.wgs84_height_m - waypoint.egm96_height_m) < 0.01:
            raise ValueError("WGS84 and EGM96 heights unexpectedly match.")
    if any(
        abs(
            waypoint.wgs84_height_m
            - waypoint.dsm_wgs84_m
            - scenario.agl_m
        ) > 1e-9
        for waypoint in waypoints
    ):
        raise ValueError("A waypoint does not maintain the requested AGL.")
    return waypoints
