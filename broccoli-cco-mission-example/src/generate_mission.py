"""Regenerate G1 only using the original planning implementation.

The authoritative delivered KMZ is an exact extraction of the saved mission.
This driver regenerates the same 21-position plan from GIS + the original DSM;
it does not overwrite the published example. See README.md for prerequisites.
"""
import argparse
from dataclasses import replace
import importlib.util
import json
import math
import sys
from pathlib import Path
from pyproj import datadir


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def main():
    sys.dont_write_bytecode = True
    here = Path(__file__).resolve().parent
    package = here.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dem", type=Path, required=True, help="External WGS84-ellipsoid-height DSM raster; not included in this repository")
    parser.add_argument("--geoid-grid", type=Path, help="External us_nga_egm96_15.tif; optional if already installed in PROJ")
    parser.add_argument("--output-dir", type=Path, required=True, help="New directory; never overwrite delivered extraction")
    args = parser.parse_args()
    if not args.dem.is_file():
        parser.error("--dem must point to an existing external DSM raster")
    if args.geoid_grid is not None and (not args.geoid_grid.is_file() or args.geoid_grid.name!="us_nga_egm96_15.tif"):
        parser.error("--geoid-grid must point to an existing us_nga_egm96_15.tif")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    mission_dir=args.output_dir
    gis_dir=args.output_dir/"gis"
    input_dir=args.output_dir/"inputs"
    for directory in (gis_dir,input_dir):
        directory.mkdir()
    module = load("cco_core", here / "cco_core.py")
    helper = load("waypoint_builder", here / "waypoint_builder.py")
    # Resolve an external or already-installed EGM96 grid locally. Do not allow online
    # availability metadata to hide a failed range request or yield infinity.
    if args.geoid_grid is not None:
        datadir.append_data_dir(str(args.geoid_grid.resolve().parent))
    def local_vertical_transformer():
        module.proj_network.set_network_enabled(False)
        transforms = module.TransformerGroup(module.CRS.from_epsg(4979),
            module.CRS.from_user_input("EPSG:4326+5773"), always_xy=True, allow_ballpark=False)
        if not transforms.best_available or not transforms.transformers:
            raise ValueError("Local non-ballpark EGM96 conversion grid unavailable; provide --geoid-grid")
        transformer = transforms.transformers[0]
        return transformer, transformer.description, float(transformer.accuracy)
    module.build_vertical_transformer = local_vertical_transformer
    plants_path = package / "data" / "plant-locations.geojson"
    center_path = package / "data" / "observation-center.geojson"
    points = module.gpd.read_file(plants_path).to_crs(module.WORKING_EPSG)
    centers = module.gpd.read_file(center_path).to_crs(module.WORKING_EPSG)
    assert len(points) == 4 and len(centers) == 1
    row = centers.iloc[0]
    group = module.TargetGroup("G1", "ind-pos", (row.geometry.x,row.geometry.y),
        points["plant_id"].tolist(), list(points.geometry))
    scenario = replace(helper.mission_scenarios(module)[0],
        mission_name="M3M-CCO-150cm", filename_stem="M3M-CCO-150cm")
    sampler = module.DsmSampler(args.dem, module.WORKING_EPSG)
    try:
        group.target_dsm_wgs84_m = sampler.sample(*group.center_xy)
        # Preserve the historical source's photo phase/count rather than
        # optimizing a new count for just this smaller selection.
        count, radius = 21, 1.5
        geometries = module.build_group_photo_geometries([group],scenario,radius,count,sampler)
        for geometry in geometries["G1"]:
            assert min(geometry.overlaps_to_next.values()) >= 0.80
            assert min(geometry.plant_margin_m.values()) >= 0.10
        chord = 2*radius*math.sin(math.pi/count)
        settle = max(0.20,scenario.minimum_interval_sec+module.SHOT_MARGIN_SEC-chord)
        waypoints = helper.build_same_agl_no_climb_waypoints(module,[group],geometries,scenario,1.0,3.0)
        template = module.build_wpml_xml(waypoints,scenario,settle,1.0,3.0,template=True)
        waylines = module.build_wpml_xml(waypoints,scenario,settle,1.0,3.0,template=False)
        helper.add_waypoint_names(module,template,waypoints)
        helper.add_waypoint_names(module,waylines,waypoints)
        kmz = mission_dir/(scenario.filename_stem+".kmz")
        if not all(math.isfinite(w.egm96_height_m) for w in waypoints):
            raise ValueError("Nonfinite vertical transform result")
        module.export_kmz(kmz,template,waylines)
        # Exactly the four confirmed GeoJSON layers; no route or union layer.
        for path in (plants_path,center_path):
            (input_dir/path.name).write_text(path.read_text(),encoding="utf-8")
        photos = []
        for w in waypoints:
            if w.kind != "photo":
                continue
            photos.append({"type":"Feature","properties":{
                "group_id":w.group_id,"waypoint_index":w.index,
                "waypoint_name":"%03d"%(w.index+1),
                "photo_sequence":w.photo_sequence,"file_suffix":w.file_suffix,
                "speed_mps":w.speed_mps,"heading_deg":w.heading_deg,
                "gimbal_pitch_deg":w.pitch_deg,"dsm_wgs84_m":w.dsm_wgs84_m,
                "flight_wgs84_m":w.wgs84_height_m,"flight_egm96_m":w.egm96_height_m},
                "geometry":{"type":"Point","coordinates":list(w.lonlat)}})
        photo_collection = {"type":"FeatureCollection","name":"photo-positions","features":photos}
        (gis_dir/"photo-positions.geojson").write_text(json.dumps(photo_collection,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
        footprints_path = gis_dir/"estimated-footprints-rgb-ms.geojson"
        module.export_footprints(footprints_path,[group],scenario,radius,geometries)
        footprints = json.loads(footprints_path.read_text())
        to_wgs84 = module.Transformer.from_crs(module.WORKING_EPSG,4326,always_xy=True)
        for feature in footprints["features"]:
            feature["geometry"]["coordinates"] = [
                [list(to_wgs84.transform(*coordinate)) for coordinate in ring]
                for ring in feature["geometry"]["coordinates"]]
        footprints.pop("crs",None)
        footprints_path.write_text(json.dumps(footprints,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
        module.validate_kmz(kmz,scenario,22,21)
        kml_exporter = load("input_kml",here/"export_kml.py")
        kml_exporter.export_inputs(input_dir,input_dir)
        print("Generated:",kmz)
        print("22 mission waypoints; four GeoJSON layers and two input-point KML layers (WGS84). Not flight-verified.")
    finally:
        sampler.close()


if __name__ == "__main__":
    main()
