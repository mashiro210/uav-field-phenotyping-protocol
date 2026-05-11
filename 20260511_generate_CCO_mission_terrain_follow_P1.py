# Author: Mashiro
# Last update: 2026/5/11
# Descripion: generate CCO mission with terrain follow line route; P1

# load modules
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
# --- 入力ファイル ---
SHP_PATH     = "center_point.shp"       # 中心点(POI)となるPointデータのShapefile
DEM_PATH     = "your_dem_data.tif"      # DEMデータ

# --- 出力設定 ---
OUTPUT_DIR   = "./missions"
MISSION_NAME = "cco_orbit_mission"
OUTPUT_KMZ   = os.path.join(OUTPUT_DIR, f"{MISSION_NAME}.kmz")

# --- 機体・ペイロード(カメラ) 設定 ---
DRONE_TYPE   = "M350"           # "M400", "M350", "M300" のいずれかを指定
PAYLOAD_ENUM = "50"             # Zenmuse P1 (50)

# ==========================================
# ★ CCO (円周斜め撮影) 設定
# ==========================================
LOCAL_EPSG_CODE = 6680          # 距離計算用ローカルEPSG (例: 北海道12系=6680)

CIRCLE_RADIUS_M = 50.0          # 円周の半径 (m)
ANGLE_STEP_DEG  = 10.0          # 何度ごとに撮影するか (10度なら1周で36枚)
TARGET_ALTITUDE_M = 40.0        # 対地目標高度 (m)

FLIGHT_SPEED    = 5.0           # 飛行速度 (m/s)
USE_INFINITY_FOCUS = False      # True: 無限遠 / False: 最初のWPで中心に向けてAFを1回実行し固定

RC_LOST_ACTION  = "goBack"      # 信号切断時: goBack(RTH)

# --- XMLネームスペース ---
KML_NS  = "http://www.opengis.net/kml/2.2"
WPML_NS = "http://www.dji.com/wpmz/1.0.6"

if DRONE_TYPE == "M400":
    DRONE_ENUM, DRONE_SUB_ENUM = "103", "0"
elif DRONE_TYPE == "M350":
    DRONE_ENUM, DRONE_SUB_ENUM = "89", "0"
elif DRONE_TYPE == "M300":
    DRONE_ENUM, DRONE_SUB_ENUM = "60", "0"
else:
    raise ValueError("DRONE_TYPE は 'M400', 'M350', 'M300' のいずれかをご指定ください。")

IMAGE_FORMAT_STR = "wide"

# ==========================================================
# 2. コアロジック (ルート計算)
# ==========================================================
def process_orbit_waypoints(shp_path, dem_path, radius, angle_step, alt_m, local_epsg):
    gdf = gpd.read_file(shp_path)
    if gdf.crs is None or gdf.crs.to_epsg() != 4326:
        gdf = gdf.to_crs(epsg=4326)
    
    poi_wgs84 = gdf.geometry.iloc[0]
    gdf_proj = gdf.to_crs(epsg=local_epsg)
    poi_proj = gdf_proj.geometry.iloc[0]

    # 中心点(POI)のDEM標高を取得
    poi_alt_wgs84 = 0.0
    execute_height_mode = "WGS84"
    coord_height_mode = "EGM96"
    has_dem = os.path.exists(dem_path)

    if has_dem:
        try:
            with rasterio.open(dem_path) as src:
                with WarpedVRT(src, crs="EPSG:4326") as vrt:
                    val = list(vrt.sample([(poi_wgs84.x, poi_wgs84.y)]))[0][0]
                    poi_alt_wgs84 = float(val) if val > -1000 else 0.0
        except Exception as e:
            print(f"⚠️ DEM処理エラー: {e}")
            has_dem = False

    if not has_dem:
        print("⚠️ DEM未検出: 離陸地点相対高度モードを適用します。")
        execute_height_mode = "relativeToStartPoint"
        coord_height_mode = "relativeToStartPoint"

    # ★ ジンバルピッチ角の計算 ★
    # 常にPOIをカメラの中心に捉えるための角度を自動計算します。
    # （※もし「-45度固定」にしたい場合は、下の行を `gimbal_pitch = -45.0` に書き換えてください）
    pitch_rad = math.atan2(alt_m, radius)
    gimbal_pitch = -math.degrees(pitch_rad)
    print(f"[計算] 高度 {alt_m}m / 半径 {radius}m -> ジンバルピッチ角: {gimbal_pitch:.1f}度 に設定します")

    # 円周上のウェイポイントを生成
    angles = np.arange(0, 360, angle_step)
    wp_data = []
    
    set_network_enabled(True)
    transformer = Transformer.from_crs("EPSG:4979", "EPSG:4326+5773", always_xy=True)

    for i, angle in enumerate(angles):
        rad = math.radians(angle)
        x = poi_proj.x + radius * math.cos(rad)
        y = poi_proj.y + radius * math.sin(rad)
        
        wp_pt = gpd.GeoSeries([Point(x, y)], crs=f"EPSG:{local_epsg}").to_crs(epsg=4326).iloc[0]
        
        alt_w, alt_e = alt_m, alt_m
        if has_dem:
            with rasterio.open(dem_path) as src:
                with WarpedVRT(src, crs="EPSG:4326") as vrt:
                    val = list(vrt.sample([(wp_pt.x, wp_pt.y)]))[0][0]
                    wp_ground_wgs84 = float(val) if val > -1000 else 0.0
                    alt_w = wp_ground_wgs84 + alt_m
                    _, _, alt_e = transformer.transform(wp_pt.x, wp_pt.y, alt_w)

        wp_data.append({
            'id': i,
            'lon': wp_pt.x,
            'lat': wp_pt.y,
            'alt_w': alt_w,
            'alt_e': alt_e
        })

    poi_data = {'lon': poi_wgs84.x, 'lat': poi_wgs84.y, 'alt_w': poi_alt_wgs84}
    return wp_data, poi_data, gimbal_pitch, execute_height_mode, coord_height_mode

# ==========================================================
# 3. XML構築モジュール
# ==========================================================
def add_elem(parent, tag, text=None):
    if tag.startswith("wpml:"):
        elem = ET.SubElement(parent, f"{{{WPML_NS}}}{tag[5:]}")
    else:
        elem = ET.SubElement(parent, f"{{{KML_NS}}}{tag}")
    if text is not None: 
        elem.text = str(text)
    return elem

def generate_dji_xml(wp_data, poi_data, pitch_deg, exec_h_mode, coord_h_mode):
    ET.register_namespace('', KML_NS)
    ET.register_namespace('wpml', WPML_NS)
    timestamp = str(int(time.time() * 1000))
    total_len = len(wp_data)

    # --- template.kml ---
    kml_t = ET.Element(f"{{{KML_NS}}}kml")
    doc_t = add_elem(kml_t, "Document")
    add_elem(doc_t, "wpml:createTime", timestamp)
    add_elem(doc_t, "wpml:updateTime", timestamp)

    mc_t = add_elem(doc_t, "wpml:missionConfig")
    add_elem(mc_t, "wpml:flyToWaylineMode", "safely")
    add_elem(mc_t, "wpml:finishAction", "goHome")
    add_elem(mc_t, "wpml:exitOnRCLost", "executeLostAction")
    add_elem(mc_t, "wpml:executeRCLostAction", RC_LOST_ACTION)
    add_elem(mc_t, "wpml:takeOffSecurityHeight", "20.0")
    add_elem(mc_t, "wpml:globalTransitionalSpeed", "10.0")
    
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
    add_elem(sys_param_t, "wpml:heightMode", coord_h_mode)
    add_elem(sys_param_t, "wpml:positioningType", "GPS")
    
    add_elem(folder_t, "wpml:autoFlightSpeed", str(FLIGHT_SPEED))
    add_elem(folder_t, "wpml:gimbalPitchMode", "usePointSetting")
    add_elem(folder_t, "wpml:globalWaypointTurnMode", "toPointAndPassWithContinuityCurvature")

    # ★ グローバル機首設定：常にPOI(中心)を向くように設定 ★
    gwh_t = add_elem(folder_t, "wpml:globalWaypointHeadingParam")
    add_elem(gwh_t, "wpml:waypointHeadingMode", "towardPOI")
    add_elem(gwh_t, "wpml:waypointPoiPoint", f"{poi_data['lat']},{poi_data['lon']},{poi_data['alt_w']}")
    add_elem(gwh_t, "wpml:waypointHeadingPathMode", "followBadArc")

    pp_t = add_elem(folder_t, "wpml:payloadParam")
    add_elem(pp_t, "wpml:payloadPositionIndex", "0")
    add_elem(pp_t, "wpml:imageFormat", IMAGE_FORMAT_STR)

    # --- waylines.wpml ---
    kml_w = ET.Element(f"{{{KML_NS}}}kml")
    doc_w = add_elem(kml_w, "Document")
    mc_w = add_elem(doc_w, "wpml:missionConfig")
    add_elem(mc_w, "wpml:missionID", f"{MISSION_NAME}_ID")
    add_elem(mc_w, "wpml:flyToWaylineMode", "safely")
    add_elem(mc_w, "wpml:finishAction", "goHome")
    add_elem(mc_w, "wpml:exitOnRCLost", "executeLostAction")
    add_elem(mc_w, "wpml:executeRCLostAction", RC_LOST_ACTION)
    
    di_w = add_elem(mc_w, "wpml:droneInfo")
    add_elem(di_w, "wpml:droneEnumValue", DRONE_ENUM)
    add_elem(di_w, "wpml:droneSubEnumValue", DRONE_SUB_ENUM)

    folder_w = add_elem(doc_w, "Folder")
    add_elem(folder_w, "wpml:templateId", "0")
    add_elem(folder_w, "wpml:executeHeightMode", exec_h_mode)
    add_elem(folder_w, "wpml:waylineId", "0")
    add_elem(folder_w, "wpml:autoFlightSpeed", str(FLIGHT_SPEED))

    pp_w = add_elem(folder_w, "wpml:payloadParam")
    add_elem(pp_w, "wpml:payloadPositionIndex", "0")
    add_elem(pp_w, "wpml:imageFormat", IMAGE_FORMAT_STR)

    for wp in wp_data:
        i = wp['id']
        
        # --- Template Placemark ---
        pm_t = add_elem(folder_t, "Placemark")
        pt_t = add_elem(pm_t, "Point")
        add_elem(pt_t, "coordinates", f"{wp['lon']},{wp['lat']}")
        add_elem(pm_t, "wpml:index", str(i))
        add_elem(pm_t, "wpml:ellipsoidHeight", str(round(wp['alt_w'], 3)))
        add_elem(pm_t, "wpml:height", str(round(wp['alt_e'], 3)))
        add_elem(pm_t, "wpml:useGlobalHeight", "0")
        add_elem(pm_t, "wpml:useGlobalSpeed", "1")
        add_elem(pm_t, "wpml:useGlobalHeadingParam", "1")  # グローバル(towardPOI)に従う
        add_elem(pm_t, "wpml:useGlobalTurnParam", "1")
        add_elem(pm_t, "wpml:gimbalPitchAngle", str(round(pitch_deg, 1)))
        add_elem(pm_t, "wpml:useStraightLine", "0")
        
        tp_t = add_elem(pm_t, "wpml:waypointTurnParam")
        if i == total_len - 1:
            add_elem(tp_t, "wpml:waypointTurnMode", "toPointAndStopWithDiscontinuityCurvature")
            add_elem(tp_t, "wpml:waypointTurnDampingDist", "0")
        else:
            add_elem(tp_t, "wpml:waypointTurnMode", "toPointAndPassWithContinuityCurvature")
            add_elem(tp_t, "wpml:waypointTurnDampingDist", "0.2")

        # アクション構築
        ag_t = add_elem(pm_t, "wpml:actionGroup")
        add_elem(ag_t, "wpml:actionGroupId", str(i))
        add_elem(ag_t, "wpml:actionGroupStartIndex", str(i))
        add_elem(ag_t, "wpml:actionGroupEndIndex", str(i))
        add_elem(ag_t, "wpml:actionGroupMode", "sequence")
        trig_t = add_elem(ag_t, "wpml:actionTrigger")
        add_elem(trig_t, "wpml:actionTriggerType", "reachPoint")
        
        act_idx = 0
        
        # 最初のWP(0番目)でのみ、ジンバル調整とフォーカスを行う
        if i == 0:
            act_gimbal = add_elem(ag_t, "wpml:action")
            add_elem(act_gimbal, "wpml:actionId", str(act_idx))
            add_elem(act_gimbal, "wpml:actionActuatorFunc", "gimbalRotate")
            act_g_p = add_elem(act_gimbal, "wpml:actionActuatorFuncParam")
            add_elem(act_g_p, "wpml:gimbalRotateMode", "absoluteAngle")
            add_elem(act_g_p, "wpml:gimbalPitchRotateEnable", "1")
            add_elem(act_g_p, "wpml:gimbalPitchRotateAngle", str(round(pitch_deg, 1)))
            add_elem(act_g_p, "wpml:gimbalRollRotateEnable", "0")
            add_elem(act_g_p, "wpml:gimbalRollRotateAngle", "0")
            add_elem(act_g_p, "wpml:gimbalYawRotateEnable", "0")
            add_elem(act_g_p, "wpml:gimbalYawRotateAngle", "0")
            add_elem(act_g_p, "wpml:gimbalRotateTimeEnable", "0")
            add_elem(act_g_p, "wpml:gimbalRotateTime", "0")
            add_elem(act_g_p, "wpml:payloadPositionIndex", "0")
            act_idx += 1

            if not USE_INFINITY_FOCUS:
                act_focus = add_elem(ag_t, "wpml:action")
                add_elem(act_focus, "wpml:actionId", str(act_idx))
                add_elem(act_focus, "wpml:actionActuatorFunc", "focus")
                act_f_p = add_elem(act_focus, "wpml:actionActuatorFuncParam")
                add_elem(act_f_p, "wpml:payloadPositionIndex", "0")
                add_elem(act_f_p, "wpml:isPointFocus", "1")
                add_elem(act_f_p, "wpml:focusX", "0.5")
                add_elem(act_f_p, "wpml:focusY", "0.5")
                add_elem(act_f_p, "wpml:isInfiniteFocus", "0")
                act_idx += 1

        # 各WPで写真撮影
        act_photo = add_elem(ag_t, "wpml:action")
        add_elem(act_photo, "wpml:actionId", str(act_idx))
        add_elem(act_photo, "wpml:actionActuatorFunc", "takePhoto")
        act_p_p = add_elem(act_photo, "wpml:actionActuatorFuncParam")
        add_elem(act_p_p, "wpml:fileSuffix", f"CCO_{i}")
        add_elem(act_p_p, "wpml:payloadPositionIndex", "0")
        add_elem(act_p_p, "wpml:useGlobalPayloadLensIndex", "1")

        # --- Waylines Placemark ---
        pm_w = add_elem(folder_w, "Placemark")
        pt_w = add_elem(pm_w, "Point")
        add_elem(pt_w, "coordinates", f"{wp['lon']},{wp['lat']}")
        add_elem(pm_w, "wpml:index", str(i))
        add_elem(pm_w, "wpml:executeHeight", str(round(wp['alt_w'], 3)))
        add_elem(pm_w, "wpml:waypointSpeed", str(FLIGHT_SPEED))

        hp_w = add_elem(pm_w, "wpml:waypointHeadingParam")
        add_elem(hp_w, "wpml:waypointHeadingMode", "towardPOI")
        add_elem(hp_w, "wpml:waypointPoiPoint", f"{poi_data['lat']},{poi_data['lon']},{poi_data['alt_w']}")
        add_elem(hp_w, "wpml:waypointHeadingPathMode", "followBadArc")

        tp_w = add_elem(pm_w, "wpml:waypointTurnParam")
        if i == total_len - 1:
            add_elem(tp_w, "wpml:waypointTurnMode", "toPointAndStopWithDiscontinuityCurvature")
            add_elem(tp_w, "wpml:waypointTurnDampingDist", "0")
        else:
            add_elem(tp_w, "wpml:waypointTurnMode", "toPointAndPassWithContinuityCurvature")
            add_elem(tp_w, "wpml:waypointTurnDampingDist", "0.2")

        # アクション構築(waylines用コピー)
        ag_w = add_elem(pm_w, "wpml:actionGroup")
        add_elem(ag_w, "wpml:actionGroupId", str(i))
        add_elem(ag_w, "wpml:actionGroupStartIndex", str(i))
        add_elem(ag_w, "wpml:actionGroupEndIndex", str(i))
        add_elem(ag_w, "wpml:actionGroupMode", "sequence")
        trig_w = add_elem(ag_w, "wpml:actionTrigger")
        add_elem(trig_w, "wpml:actionTriggerType", "reachPoint")
        
        act_idx = 0
        if i == 0:
            act_gimbal_w = add_elem(ag_w, "wpml:action")
            add_elem(act_gimbal_w, "wpml:actionId", str(act_idx))
            add_elem(act_gimbal_w, "wpml:actionActuatorFunc", "gimbalRotate")
            act_g_p_w = add_elem(act_gimbal_w, "wpml:actionActuatorFuncParam")
            add_elem(act_g_p_w, "wpml:gimbalRotateMode", "absoluteAngle")
            add_elem(act_g_p_w, "wpml:gimbalPitchRotateEnable", "1")
            add_elem(act_g_p_w, "wpml:gimbalPitchRotateAngle", str(round(pitch_deg, 1)))
            add_elem(act_g_p_w, "wpml:gimbalRollRotateEnable", "0")
            add_elem(act_g_p_w, "wpml:gimbalRollRotateAngle", "0")
            add_elem(act_g_p_w, "wpml:gimbalYawRotateEnable", "0")
            add_elem(act_g_p_w, "wpml:gimbalYawRotateAngle", "0")
            add_elem(act_g_p_w, "wpml:gimbalRotateTimeEnable", "0")
            add_elem(act_g_p_w, "wpml:gimbalRotateTime", "0")
            add_elem(act_g_p_w, "wpml:payloadPositionIndex", "0")
            act_idx += 1

            if not USE_INFINITY_FOCUS:
                act_focus_w = add_elem(ag_w, "wpml:action")
                add_elem(act_focus_w, "wpml:actionId", str(act_idx))
                add_elem(act_focus_w, "wpml:actionActuatorFunc", "focus")
                act_f_p_w = add_elem(act_focus_w, "wpml:actionActuatorFuncParam")
                add_elem(act_f_p_w, "wpml:payloadPositionIndex", "0")
                add_elem(act_f_p_w, "wpml:isPointFocus", "1")
                add_elem(act_f_p_w, "wpml:focusX", "0.5")
                add_elem(act_f_p_w, "wpml:focusY", "0.5")
                add_elem(act_f_p_w, "wpml:isInfiniteFocus", "0")
                act_idx += 1

        act_photo_w = add_elem(ag_w, "wpml:action")
        add_elem(act_photo_w, "wpml:actionId", str(act_idx))
        add_elem(act_photo_w, "wpml:actionActuatorFunc", "takePhoto")
        act_p_p_w = add_elem(act_photo_w, "wpml:actionActuatorFuncParam")
        add_elem(act_p_p_w, "wpml:fileSuffix", f"CCO_{i}")
        add_elem(act_p_p_w, "wpml:payloadPositionIndex", "0")
        add_elem(act_p_p_w, "wpml:useGlobalPayloadLensIndex", "1")

    return kml_t, kml_w

# -------------------------
# C. KMZ出力モジュール
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

    os.remove(tmp_t); os.remove(tmp_w); os.rmdir(temp_dir)
    print(f"✅ 成功！ '{output_kmz_path}' を出力しました。")


# ==========================================================
# 3. メイン実行ブロック
# ==========================================================
print("========================================")
print(f" 円周斜め撮影(CCO) ({DRONE_TYPE} + P1) ")
print("========================================\n")

print(f"[GIS処理] 中心点(POI)を基準に、半径 {CIRCLE_RADIUS_M}m で {ANGLE_STEP_DEG}度 ごとにウェイポイントを計算中...")
wp_data, poi_data, pitch_deg, exec_h_mode, coord_h_mode = process_orbit_waypoints(
    SHP_PATH, 
    DEM_PATH, 
    CIRCLE_RADIUS_M, 
    ANGLE_STEP_DEG, 
    TARGET_ALTITUDE_M, 
    LOCAL_EPSG_CODE
)

print(f"[XML生成] KML/WPML構造を構築中...")
print(f"  └ 撮影枚数: {len(wp_data)}枚 (1周)")
print(f"  └ 機首方位: 常に中心点(POI)を追従")
template_tree, waylines_tree = generate_dji_xml(wp_data, poi_data, pitch_deg, exec_h_mode, coord_h_mode)

print("[出力処理] KMZファイルをパッケージング中...")
export_kmz(template_tree, waylines_tree, OUTPUT_KMZ)

print("--- 処理がすべて完了しました ---")