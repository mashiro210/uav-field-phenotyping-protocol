import os
import sys
import math
import time
import zipfile
import xml.etree.ElementTree as ET
import numpy as np
import geopandas as gpd
from shapely.geometry import Point
import rasterio
from rasterio.vrt import WarpedVRT
from pyproj import Transformer
from pyproj.network import set_network_enabled

# ==========================================================
# 1. ユーザー設定 (ファイルパスとフライトオプション)
# ==========================================================
SHP_PATH     = "center_point.shp"       # 中心点(POI)のPointデータ
DEM_PATH     = "your_dem_data.tif"      # DEMデータ
OUTPUT_DIR   = "./missions"
MISSION_NAME = "dome_safe_v3"
OUTPUT_KMZ   = os.path.join(OUTPUT_DIR, f"{MISSION_NAME}.kmz")

# --- 機体設定 ---
DRONE_TYPE   = "M350"           # "M400", "M350", "M300"
PAYLOAD_ENUM = "50"             # Zenmuse P1

# ==========================================
# ★ 安全・高度設定 (今回のアップデート)
# ==========================================
BASE_CENTER_HEIGHT = 3.5        # ★ドーム中心の最低高度 (m) 
                                # これによりドームが地面から浮き、低高度での衝突を防ぎます

TAKEOFF_SEC_HEIGHT = 30.0       # ★最初のWPへ向かう際の安全移動高度 (m)
                                # 現場の最も高い障害物より上に設定してください

DOME_RADIUS_M = 40.0            # ドームの半径 (m)
ANGLE_STEP_DEG  = 20.0          # 各レイヤーの円周上で何度ごとに撮影するか

LOCAL_EPSG = 6680

# --- 仰角の計算設定 ---
AUTO_ELEVATION_MODE = True      
AUTO_TOTAL_LAYERS   = 4         # 直上(Nadir)を含む総レイヤー数
MANUAL_ELEVATION_ANGLES = [30.0, 50.0]
MANUAL_INCLUDE_NADIR_TOP = True

FLIGHT_SPEED    = 5.0           
USE_INFINITY_FOCUS = False      # False: 最初のWPで一度だけAF実行
RC_LOST_ACTION  = "goBack"      

# --- WPML設定 ---
KML_NS  = "http://www.opengis.net/kml/2.2"
WPML_NS = "http://www.dji.com/wpmz/1.0.6"

if DRONE_TYPE == "M400":
    DRONE_ENUM, DRONE_SUB_ENUM = "103", "0"
elif DRONE_TYPE == "M350":
    DRONE_ENUM, DRONE_SUB_ENUM = "89", "0"
elif DRONE_TYPE == "M300":
    DRONE_ENUM, DRONE_SUB_ENUM = "60", "0"
else:
    raise ValueError("DRONE_TYPE Error")

IMAGE_FORMAT_STR = "wide"

# ==========================================================
# 2. ロジック分岐処理 (仰角の決定)
# ==========================================================
if AUTO_ELEVATION_MODE:
    elev_step = 90.0 / AUTO_TOTAL_LAYERS
    target_elev_angles = [elev_step * i for i in range(1, AUTO_TOTAL_LAYERS)]
    target_include_top = True
else:
    target_elev_angles = MANUAL_ELEVATION_ANGLES
    target_include_top = MANUAL_INCLUDE_NADIR_TOP

# ==========================================================
# 3. コアロジック (ドームルート計算)
# ==========================================================
def process_dome_waypoints(shp_path, dem_path, dome_radius, base_h, elev_angles, angle_step, add_top, local_epsg):
    gdf = gpd.read_file(shp_path)
    if gdf.crs is None or gdf.crs.to_epsg() != 4326:
        gdf = gdf.to_crs(epsg=4326)
    
    poi_wgs84 = gdf.geometry.iloc[0]
    gdf_proj = gdf.to_crs(epsg=local_epsg)
    poi_proj = gdf_proj.geometry.iloc[0]

    poi_ground_alt_wgs84 = 0.0
    execute_height_mode = "WGS84"
    coord_height_mode = "EGM96"
    has_dem = os.path.exists(dem_path)

    if has_dem:
        try:
            with rasterio.open(dem_path) as src:
                with WarpedVRT(src, crs="EPSG:4326") as vrt:
                    val = list(vrt.sample([(poi_wgs84.x, poi_wgs84.y)]))[0][0]
                    poi_ground_alt_wgs84 = float(val) if val > -1000 else 0.0
        except Exception:
            has_dem = False

    if not has_dem:
        execute_height_mode = "relativeToStartPoint"
        coord_height_mode = "relativeToStartPoint"

    wp_data = []
    wp_index = 0
    set_network_enabled(True)
    transformer = Transformer.from_crs("EPSG:4979", "EPSG:4326+5773", always_xy=True)

    # カメラが向くべき中心点の高度（地面 + BASE_OFFSET）
    poi_target_alt_w = poi_ground_alt_wgs84 + base_h

    # 各レイヤー(仰角)
    for l_idx, elev_deg in enumerate(sorted(elev_angles)):
        elev_rad = math.radians(elev_deg)
        # 高度 = Base + R*sin(θ) / 半径 = R*cos(θ)
        layer_alt_above_base = dome_radius * math.sin(elev_rad)
        layer_radius = dome_radius * math.cos(elev_rad)
        
        angles = np.arange(0, 360, angle_step)
        for a_idx, angle in enumerate(angles):
            rad = math.radians(angle)
            x = poi_proj.x + layer_radius * math.cos(rad)
            y = poi_proj.y + layer_radius * math.sin(rad)
            wp_pt = gpd.GeoSeries([Point(x, y)], crs=f"EPSG:{local_epsg}").to_crs(epsg=4326).iloc[0]
            
            # WGS84高度計算 (地面高度 + BASE + レイヤー高さ)
            target_alt_w = poi_ground_alt_wgs84 + base_h + layer_alt_above_base
            alt_e = target_alt_w
            if has_dem:
                _, _, alt_e = transformer.transform(wp_pt.x, wp_pt.y, target_alt_w)

            wp_data.append({
                'id': wp_index, 'lon': wp_pt.x, 'lat': wp_pt.y,
                'alt_w': target_alt_w, 'alt_e': alt_e, 'pitch': -elev_deg,
                'is_layer_start': (a_idx == 0) # レイヤーの最初の点か判定
            })
            wp_index += 1

    # 頂点
    if add_top:
        target_alt_w = poi_target_alt_w + dome_radius
        alt_e = target_alt_w
        if has_dem:
            _, _, alt_e = transformer.transform(poi_wgs84.x, poi_wgs84.y, target_alt_w)
        wp_data.append({
            'id': wp_index, 'lon': poi_wgs84.x, 'lat': poi_wgs84.y,
            'alt_w': target_alt_w, 'alt_e': alt_e, 'pitch': -90.0,
            'is_layer_start': True
        })

    poi_data = {'lon': poi_wgs84.x, 'lat': poi_wgs84.y, 'alt_w': poi_target_alt_w}
    return wp_data, poi_data, execute_height_mode, coord_height_mode

# ==========================================================
# 4. XML構築 (先行ジンバル制御アクション実装)
# ==========================================================
def add_elem(parent, tag, text=None):
    if tag.startswith("wpml:"):
        elem = ET.SubElement(parent, f"{{{WPML_NS}}}{tag[5:]}")
    else:
        elem = ET.SubElement(parent, f"{{{KML_NS}}}{tag}")
    if text is not None: elem.text = str(text)
    return elem

def build_dome_actions(pm, wp, is_first_wp):
    ag = add_elem(pm, "wpml:actionGroup")
    add_elem(ag, "wpml:actionGroupId", str(wp['id']))
    add_elem(ag, "wpml:actionGroupStartIndex", str(wp['id']))
    add_elem(ag, "wpml:actionGroupEndIndex", str(wp['id']))
    add_elem(ag, "wpml:actionGroupMode", "sequence")
    trig = add_elem(ag, "wpml:actionTrigger")
    add_elem(trig, "wpml:actionTriggerType", "reachPoint")
    
    act_idx = 0
    # ★ポイント1: 新レイヤーの最初、またはミッション開始時にジンバルを回転
    if wp['is_layer_start']:
        act_g = add_elem(ag, "wpml:action")
        add_elem(act_g, "wpml:actionId", str(act_idx)); act_idx += 1
        add_elem(act_g, "wpml:actionActuatorFunc", "gimbalRotate")
        act_g_p = add_elem(act_g, "wpml:actionActuatorFuncParam")
        add_elem(act_g_p, "wpml:gimbalRotateMode", "absoluteAngle")
        add_elem(act_g_p, "wpml:gimbalPitchRotateEnable", "1")
        add_elem(act_g_p, "wpml:gimbalPitchRotateAngle", str(round(wp['pitch'], 1)))
        add_elem(act_g_p, "wpml:payloadPositionIndex", "0")

    # ★ポイント2: 最初のウェイポイントのみフォーカスを実行
    if is_first_wp and not USE_INFINITY_FOCUS:
        act_f = add_elem(ag, "wpml:action")
        add_elem(act_f, "wpml:actionId", str(act_idx)); act_idx += 1
        add_elem(act_f, "wpml:actionActuatorFunc", "focus")
        act_f_p = add_elem(act_f, "wpml:actionActuatorFuncParam")
        add_elem(act_f_p, "wpml:payloadPositionIndex", "0")
        add_elem(act_f_p, "wpml:isPointFocus", "1")
        add_elem(act_f_p, "wpml:focusX", "0.5"); add_elem(act_f_p, "wpml:focusY", "0.5")

    # 撮影
    act_p = add_elem(ag, "wpml:action")
    add_elem(act_p, "wpml:actionId", str(act_idx))
    add_elem(act_p, "wpml:actionActuatorFunc", "takePhoto")
    act_p_p = add_elem(act_p, "wpml:actionActuatorFuncParam")
    add_elem(act_p_p, "wpml:fileSuffix", f"Dome_{wp['id']}")
    add_elem(act_p_p, "wpml:payloadPositionIndex", "0")
    add_elem(act_p_p, "wpml:useGlobalPayloadLensIndex", "1")

def generate_dji_xml(wp_data, poi_data, exec_h_mode, coord_h_mode):
    timestamp = str(int(time.time() * 1000))
    kml_t = ET.Element(f"{{{KML_NS}}}kml")
    doc_t = add_elem(kml_t, "Document")
    
    mc_t = add_elem(doc_t, "wpml:missionConfig")
    add_elem(mc_t, "wpml:flyToWaylineMode", "safely")
    add_elem(mc_t, "wpml:finishAction", "goHome")
    add_elem(mc_t, "wpml:exitOnRCLost", "executeLostAction")
    add_elem(mc_t, "wpml:executeRCLostAction", RC_LOST_ACTION)
    add_elem(mc_t, "wpml:takeOffSecurityHeight", str(TAKEOFF_SEC_HEIGHT)) # ★安全移動高度
    
    di_t = add_elem(mc_t, "wpml:droneInfo")
    add_elem(di_t, "wpml:droneEnumValue", DRONE_ENUM)
    add_elem(di_t, "wpml:droneSubEnumValue", DRONE_SUB_ENUM)
    pi_t = add_elem(mc_t, "wpml:payloadInfo")
    add_elem(pi_t, "wpml:payloadEnumValue", PAYLOAD_ENUM)
    add_elem(pi_t, "wpml:payloadPositionIndex", "0")

    folder_t = add_elem(doc_t, "Folder")
    add_elem(folder_t, "wpml:templateType", "waypoint")
    add_elem(folder_t, "wpml:templateId", "0")
    
    sys_param_t = add_elem(folder_t, "wpml:waylineCoordinateSysParam")
    add_elem(sys_param_t, "wpml:coordinateMode", "WGS84")
    add_elem(sys_param_t, "wpml:heightMode", coord_height_mode)
    add_elem(sys_param_t, "wpml:positioningType", "GPS")
    
    add_elem(folder_t, "wpml:autoFlightSpeed", str(FLIGHT_SPEED))
    add_elem(folder_t, "wpml:gimbalPitchMode", "usePointSetting")
    add_elem(folder_t, "wpml:globalWaypointTurnMode", "toPointAndPassWithContinuityCurvature")

    gwh_t = add_elem(folder_t, "wpml:globalWaypointHeadingParam")
    add_elem(gwh_t, "wpml:waypointHeadingMode", "towardPOI")
    add_elem(gwh_t, "wpml:waypointPoiPoint", f"{poi_data['lat']},{poi_data['lon']},{poi_data['alt_w']}")

    pp_t = add_elem(folder_t, "wpml:payloadParam")
    add_elem(pp_t, "wpml:payloadPositionIndex", "0")
    add_elem(pp_t, "wpml:imageFormat", IMAGE_FORMAT_STR)

    # --- waylines.wpml も同様の構成 ---
    kml_w = ET.Element(f"{{{KML_NS}}}kml")
    doc_w = add_elem(kml_w, "Document")
    mc_w = add_elem(doc_w, "wpml:missionConfig")
    add_elem(mc_w, "wpml:missionID", f"{MISSION_NAME}_ID")
    di_w = add_elem(mc_w, "wpml:droneInfo")
    add_elem(di_w, "wpml:droneEnumValue", DRONE_ENUM)
    add_elem(di_w, "wpml:droneSubEnumValue", DRONE_SUB_ENUM)

    folder_w = add_elem(doc_w, "Folder")
    add_elem(folder_w, "wpml:templateId", "0")
    add_elem(folder_w, "wpml:executeHeightMode", exec_h_mode)
    add_elem(folder_w, "wpml:autoFlightSpeed", str(FLIGHT_SPEED))

    for idx, wp in enumerate(wp_data):
        # Template
        pm_t = add_elem(folder_t, "Placemark")
        pt_t = add_elem(pm_t, "Point")
        add_elem(pt_t, "coordinates", f"{wp['lon']},{wp['lat']}")
        add_elem(pm_t, "wpml:index", str(wp['id']))
        add_elem(pm_t, "wpml:ellipsoidHeight", str(round(wp['alt_w'], 3)))
        add_elem(pm_t, "wpml:height", str(round(wp['alt_e'], 3)))
        add_elem(pm_t, "wpml:useGlobalHeight", "0")
        add_elem(pm_t, "wpml:useGlobalSpeed", "1")
        add_elem(pm_t, "wpml:useGlobalHeadingParam", "1")
        add_elem(pm_t, "wpml:useGlobalTurnParam", "1")
        add_elem(pm_t, "wpml:gimbalPitchAngle", str(round(wp['pitch'], 1)))
        build_dome_actions(pm_t, wp, idx == 0)

        # Waylines
        pm_w = add_elem(folder_w, "Placemark")
        pt_w = add_elem(pm_w, "Point")
        add_elem(pt_w, "coordinates", f"{wp['lon']},{wp['lat']}")
        add_elem(pm_w, "wpml:index", str(wp['id']))
        add_elem(pm_w, "wpml:executeHeight", str(round(wp['alt_w'], 3)))
        hp_w = add_elem(pm_w, "wpml:waypointHeadingParam")
        add_elem(hp_w, "wpml:waypointHeadingMode", "towardPOI")
        add_elem(hp_w, "wpml:waypointPoiPoint", f"{poi_data['lat']},{poi_data['lon']},{poi_data['alt_w']}")
        build_dome_actions(pm_w, wp, idx == 0)

    return kml_t, kml_w

# -------------------------
# 5. 出力・メイン実行 (前回と同様のため省略可、以下完成形)
# -------------------------
def export_kmz(kml_t_tree, kml_w_tree, output_kmz_path):
    temp_dir = os.path.join(os.path.dirname(output_kmz_path), "wpmz_temp")
    os.makedirs(temp_dir, exist_ok=True)
    if hasattr(ET, 'indent'): 
        ET.indent(kml_t_tree, space="  ")
        ET.indent(kml_w_tree, space="  ")
    tmp_t = os.path.join(temp_dir, "template.kml")
    tmp_w = os.path.join(temp_dir, "waylines.wpml")
    with open(tmp_t, 'w', encoding='utf-8') as f: 
        f.write(ET.tostring(kml_t_tree, encoding="utf-8", xml_declaration=True).decode('utf-8'))
    with open(tmp_w, 'w', encoding='utf-8') as f: 
        f.write(ET.tostring(kml_w_tree, encoding="utf-8", xml_declaration=True).decode('utf-8'))
    with zipfile.ZipFile(output_kmz_path, 'w', zipfile.ZIP_DEFLATED) as kmz:
        kmz.write(tmp_t, arcname="wpmz/template.kml")
        kmz.write(tmp_w, arcname="wpmz/waylines.wpml")
    import shutil; shutil.rmtree(temp_dir)
    print(f"✅ KMZ出力完了: {output_kmz_path}")

print(f"--- ドームフライト生成 (Base: {BASE_CENTER_HEIGHT}m) ---")
wp_data, poi_data, exec_h, coord_h = process_dome_waypoints(
    SHP_PATH, DEM_PATH, DOME_RADIUS_M, BASE_CENTER_HEIGHT, 
    target_elev_angles, ANGLE_STEP_DEG, target_include_top, LOCAL_EPSG
)
t_tree, w_tree = generate_dji_xml(wp_data, poi_data, exec_h, coord_h)
export_kmz(t_tree, w_tree, OUTPUT_KMZ)