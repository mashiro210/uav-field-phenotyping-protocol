"""Verify the published example and regenerate it without changing fixtures."""
from pathlib import Path
from collections import Counter
import hashlib
import json
import math
import os
import re
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import zipfile

from pyproj import Transformer
from shapely.geometry import shape
from shapely.ops import transform

ROOT=Path(__file__).resolve().parent
NS={"k":"http://www.opengis.net/kml/2.2","w":"http://www.dji.com/wpmz/1.0.6"}
MISSION=ROOT/"results/M3M-CCO-150cm.kmz"
LAYERS={"data/plant-locations.geojson":(4,"Point"),
    "data/observation-center.geojson":(1,"Point"),
    "results/gis/photo-positions.geojson":(21,"Point"),
    "results/gis/estimated-footprints-rgb-ms.geojson":(42,"Polygon")}


def read(path):
    return json.loads(path.read_text())


def normalized(element,ignore_name=False):
    return (element.tag,(element.text or "").strip(),tuple(normalized(child,ignore_name)
        for child in element if not(ignore_name and child.tag=="{%s}name"%NS["k"])))


def control_digest(root):
    content=[normalized(pm,True) for pm in root.findall(".//k:Placemark",NS)]
    return hashlib.sha256(json.dumps(content,ensure_ascii=False,separators=(",",":")).encode()).hexdigest()


class MissionExampleTests(unittest.TestCase):
    def test_four_geojson_layers(self):
        self.assertEqual({str(p.relative_to(ROOT)) for directory in (ROOT/"data",ROOT/"results") for p in directory.rglob("*.geojson")},set(LAYERS))
        self.assertFalse(list(ROOT.rglob("*.tif")))
        for relative,(count,kind) in LAYERS.items():
            data=read(ROOT/relative)
            self.assertNotIn("crs",data)
            self.assertEqual(len(data["features"]),count)
            for feature in data["features"]:
                self.assertEqual(feature["geometry"]["type"],kind)
                geometry=shape(feature["geometry"])
                self.assertTrue(geometry.is_valid)
                self.assertFalse(geometry.is_empty)
                self.assertTrue(-180<=geometry.bounds[0]<=180 and -90<=geometry.bounds[1]<=90)

    def test_input_kml_matches_wgs84_geojson(self):
        for stem,count in (("plant-locations",4),("observation-center",1)):
            root=ET.parse(ROOT/"data"/(stem+".kml")).getroot()
            features=read(ROOT/"data"/(stem+".geojson"))["features"]
            pms=root.findall(".//k:Placemark",NS)
            self.assertEqual(len(pms),count)
            for feature,pm in zip(features,pms):
                coords=list(map(float,pm.findtext("k:Point/k:coordinates",namespaces=NS).split(",")))
                self.assertEqual(coords[:2],feature["geometry"]["coordinates"][:2])
                self.assertEqual(pm.findtext("k:Point/k:altitudeMode",namespaces=NS),"clampToGround")

    def test_mission_names_and_reference_controls(self):
        reference=read(ROOT/"validation/mission-control-checksums.json")
        with zipfile.ZipFile(MISSION) as archive:
            self.assertIsNone(archive.testzip())
            self.assertEqual(set(archive.namelist()),{"wpmz/template.kml","wpmz/waylines.wpml"})
            for name in archive.namelist():
                root=ET.fromstring(archive.read(name))
                self.assertEqual(root.findtext("k:Document/k:name",namespaces=NS),"M3M-CCO-150cm")
                pms=root.findall(".//k:Placemark",NS)
                self.assertEqual(len(pms),22)
                self.assertEqual([p.findtext("k:name",namespaces=NS) for p in pms],["%03d"%i for i in range(1,23)])
                self.assertEqual([p.text for p in root.findall(".//w:fileSuffix",NS)],["G1-%03d"%i for i in range(1,22)])
                self.assertEqual(control_digest(root),reference[name])

    def test_height_radius_and_photo_identifiers(self):
        projection=Transformer.from_crs(4326,32654,always_xy=True)
        center=read(ROOT/"data/observation-center.geojson")["features"][0]
        cx,cy=projection.transform(*center["geometry"]["coordinates"])
        photos=read(ROOT/"results/gis/photo-positions.geojson")["features"]
        for index,photo in enumerate(photos,1):
            p=photo["properties"]
            self.assertEqual(p["waypoint_name"],"%03d"%index)
            self.assertEqual(p["file_suffix"],"G1-%03d"%index)
            self.assertAlmostEqual(p["flight_wgs84_m"]-p["dsm_wgs84_m"],1.5,places=7)
            self.assertEqual(p["gimbal_pitch_deg"],-45)
            x,y=projection.transform(*photo["geometry"]["coordinates"])
            self.assertAlmostEqual(math.hypot(x-cx,y-cy),1.5,places=5)

    def test_footprint_coverage_and_overlap(self):
        projection=Transformer.from_crs(4326,32654,always_xy=True)
        plants=[transform(projection.transform,shape(f["geometry"])) for f in read(ROOT/"data/plant-locations.geojson")["features"]]
        features=read(ROOT/"results/gis/estimated-footprints-rgb-ms.geojson")["features"]
        self.assertEqual(Counter(f["properties"]["sensor"] for f in features),{"M3M-RGB":21,"M3M-MS":21})
        for sensor in ("M3M-RGB","M3M-MS"):
            ordered=sorted((f for f in features if f["properties"]["sensor"]==sensor),key=lambda f:f["properties"]["photo_sequence"])
            polygons=[transform(projection.transform,shape(f["geometry"])) for f in ordered]
            self.assertTrue(all(p.covers(t) for p in polygons for t in plants))
            self.assertGreaterEqual(min(t.distance(p.exterior) for p in polygons for t in plants),0.1)
            self.assertGreaterEqual(min(a.intersection(b).area/min(a.area,b.area) for a,b in zip(polygons,polygons[1:]+polygons[:1])),0.8)

    def test_readme_links_and_private_path_scan(self):
        for md in ROOT.rglob("*.md"):
            for target in re.findall(r"\]\(([^)]+)\)",md.read_text()):
                if "://" not in target and not target.startswith("#"):
                    self.assertTrue((md.parent/target).exists(),str(md.relative_to(ROOT))+" -> "+target)
        markers=[b"/"+b"Users"+b"/",b"/"+b"private"+b"/",b"C:"+bytes([92])+b"Users"+bytes([92])]
        for path in ROOT.rglob("*"):
            if path.is_file() and "__pycache__" not in path.parts:
                payload=path.read_bytes()
                self.assertFalse(any(marker in payload for marker in markers),str(path.relative_to(ROOT)))

    def test_external_dsm_is_required(self):
        with tempfile.TemporaryDirectory(prefix="broccoli-cco-cli-") as temporary:
            output=Path(temporary)/"generated"
            result=subprocess.run([sys.executable,str(ROOT/"src/generate_mission.py"),"--output-dir",str(output)],capture_output=True,text=True)
            self.assertNotEqual(result.returncode,0)
            self.assertIn("--dem",result.stderr)
            self.assertFalse(output.exists())

    @unittest.skipUnless(os.environ.get("BROCCOLI_DSM") and os.environ.get("BROCCOLI_EGM96_GRID"),"External DSM and EGM96 grid not provided; example-data checks still run")
    def test_offline_regeneration(self):
        with tempfile.TemporaryDirectory(prefix="broccoli-cco-test-") as temporary:
            output=Path(temporary)/"generated"
            result=subprocess.run([sys.executable,str(ROOT/"src/generate_mission.py"),"--dem",os.environ["BROCCOLI_DSM"],"--geoid-grid",os.environ["BROCCOLI_EGM96_GRID"],"--output-dir",str(output)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            with zipfile.ZipFile(MISSION) as published,zipfile.ZipFile(output/"M3M-CCO-150cm.kmz") as generated:
                for name in published.namelist():
                    a,b=ET.fromstring(published.read(name)),ET.fromstring(generated.read(name))
                    self.assertEqual([normalized(p) for p in a.findall(".//k:Placemark",NS)],[normalized(p) for p in b.findall(".//k:Placemark",NS)])
                    self.assertEqual(normalized(a.find("k:Document/w:missionConfig",NS)),normalized(b.find("k:Document/w:missionConfig",NS)))
            for source_relative,generated_relative in (("data/plant-locations.geojson","inputs/plant-locations.geojson"),
                ("data/observation-center.geojson","inputs/observation-center.geojson"),
                ("results/gis/photo-positions.geojson","gis/photo-positions.geojson"),
                ("results/gis/estimated-footprints-rgb-ms.geojson","gis/estimated-footprints-rgb-ms.geojson")):
                a,b=read(ROOT/source_relative)["features"],read(output/generated_relative)["features"]
                self.assertEqual(len(a),len(b))
                for x,y in zip(a,b):
                    self.assertLess(shape(x["geometry"]).hausdorff_distance(shape(y["geometry"])),1e-12)
            for name in ("plant-locations.kml","observation-center.kml"):
                self.assertEqual((ROOT/"data"/name).read_bytes(),(output/"inputs"/name).read_bytes())


if __name__=="__main__":
    unittest.main()
