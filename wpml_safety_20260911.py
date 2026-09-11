"""Shared validation helpers for the 2026-09-11 WPML generators."""

import math


def build_egm96_transformer():
    """Return a verified EPSG:4979 -> EPSG:4326+5773 transformer.

    Network access is enabled so PROJ can download the EGM96 grid once. A
    ballpark transformation is deliberately forbidden because it can silently
    leave the altitude unchanged.
    """
    try:
        from pyproj import CRS, network
        from pyproj.transformer import TransformerGroup
    except ImportError as exc:
        raise RuntimeError("pyproj is required: python -m pip install pyproj") from exc

    network.set_network_enabled(True)
    group = TransformerGroup(
        CRS.from_epsg(4979),
        CRS.from_user_input("EPSG:4326+5773"),
        always_xy=True,
        allow_ballpark=False,
    )
    if not group.transformers or not group.best_available:
        raise RuntimeError(
            "A real WGS84-to-EGM96 grid transformation is unavailable. "
            "Check network access or the local PROJ grid cache."
        )
    return group.transformers[0]


def to_egm96_height(transformer, lon, lat, ellipsoid_height):
    """Convert and verify one WGS84 ellipsoidal height."""
    _, _, orthometric_height = transformer.transform(lon, lat, ellipsoid_height)
    if not math.isfinite(orthometric_height):
        raise RuntimeError("The WGS84-to-EGM96 transformation returned a non-finite height.")
    if abs(float(ellipsoid_height) - float(orthometric_height)) < 0.01:
        raise RuntimeError(
            "WGS84 and EGM96 heights are unchanged; the EGM96 grid was not applied."
        )
    return float(orthometric_height)


def checked_dem_value(value, nodata=None):
    """Reject NaN, infinity, nodata and placeholder raster elevations."""
    elevation = float(value)
    if not math.isfinite(elevation):
        raise RuntimeError("DEM returned a non-finite elevation.")
    if nodata is not None and math.isfinite(float(nodata)) and elevation == float(nodata):
        raise RuntimeError("DEM returned its nodata value.")
    if elevation <= -1000:
        raise RuntimeError("DEM returned an invalid placeholder elevation.")
    return elevation


def validate_mission_name(name):
    """Use the naming rule required by the target DJI Pilot 2 workflow."""
    if not name or "_" in name:
        raise ValueError("MISSION_NAME must be non-empty and must not contain underscores (_).")
    return name
