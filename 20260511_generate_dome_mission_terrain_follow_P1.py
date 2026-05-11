# Author: Mashiro
# Last update: 2026/5/11
# Descripion: generate dome mission with terrain follow line route; P1

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
# 1. グローバル設定 (ユーザー設定値)
# ==========================================================
SHP_PATH     = "center_point.shp"       # 中心点(POI)のPointデータ
DEM_PATH     = "your_dem_data.tif"      # DEMデータ
OUTPUT_DIR   = "./missions"
MISSION_NAME = "dome_orbit_mission"
OUTPUT_KMZ   = os.path.join(OUTPUT_DIR, f"{MISSION_NAME}.kmz")

# --- 機体設定 ---
DRONE_TYPE   = "M350"           # "M400", "M350", "M300"
PAYLOAD_ENUM = "50"             # Zenmuse P1

# ==========================================
# ★ ドームフライト (多層CCO + 直上) 角度設定
# ==========================================
LOCAL_EPSG_CODE = 6680          

DOME_RADIUS_M = 50.0            # ドームの半径（中心点からの一定の斜距離）(m)
ANGLE_STEP_DEG  = 15.0          # 各レイヤーの円周上で何度ごとに撮影するか

# --- 仰角の計算モード分岐 ---
AUTO_ELEVATION_MODE = True      # True: レイヤー数から自動計算 / False: リストを手動指定

# [自動モードの場合]
AUTO_TOTAL_LAYERS = 4           # 直上(Nadir)を含むドーム全体の総レイヤー数
                                # 例: 4の場合、(90 / 4) = 22.5度刻み (22.5°, 45°, 67.5°, 90°)

# [手動モードの場合]
MANUAL_ELEVATION_ANGLES = [30.0, 45.0, 60.0]  # 直上以外の仰角リスト
MANUAL_INCLUDE_NADIR_TOP = True               # 直上(Nadir: 90度)を追加するかどうか

# --- フライト詳細設定 ---
FLIGHT_SPEED    = 5.0           # 飛行速度 (m/s)
USE_INFINITY_FOCUS = False      # True: 無限遠 / False: 最初のみAF固定
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
# 2. ロジック分岐処理 (自動計算 or 手動設定)
# ==========================================================
if AUTO_ELEVATION_MODE:
    if AUTO_TOTAL_LAYERS < 1:
        raise ValueError("AUTO_TOTAL_LAYERS は 1 以上を指定してください。")
    # 90度を指定レイヤー数で等分
    elev_step = 90.0 / AUTO_TOTAL_LAYERS
    target_elev_angles = [elev_step * i for i in range(1, AUTO_TOTAL_LAYERS)]
    target_include_top = True
    print(f"[モード] 自動計算 (総レイヤー数: {AUTO_TOTAL_LAYERS}) -> 仰角: {target_elev_angles}, 直上: あり")
else:
    target_elev_angles = MANUAL_ELEVATION_ANGLES
    target_include_top = MANUAL_INCLUDE_NADIR_TOP
    print(f"[モード] 手動指定 -> 仰角: {target_elev_angles}, 直上: {'あり' if target_include_top else 'なし'}")

# ==========================================================
# 3. コアロジック (ドームルート計算)
# ==========================================================
def process_dome_waypoints(shp_path, dem_path, dome_radius, elev_angles, angle_step, add_top, local_epsg):
    gdf = gpd.read_file(shp_path)
    if gdf.crs is None or gdf.crs.to_epsg() != 4326:
        gdf = gdf.to_crs(epsg=4326)
    
    poi_wgs84 = gdf.geometry.iloc[0]
    gdf_proj = gdf.to_crs(epsg=local_epsg)
    poi_proj = gdf_proj.geometry.iloc[0]

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

    wp_data = []
    wp_index = 0
    
    set_network_enabled(True)
    transformer = Transformer.from_crs("EPSG:4979", "EPSG:4326+5773", always_xy=True)

    # 各レイヤー(仰角)ごとに円周軌道を生成
    for elev_deg in sorted(elev_angles):
        elev_rad = math.radians(elev_deg)
        layer_alt = dome_radius * math.sin(elev_rad)
        layer_radius = dome_radius * math.cos(elev_rad)
        pitch_deg = -elev_deg
        
        print(f"[レイヤー構築] 仰角: {elev_deg:.1f}° -> 高度: {layer_alt:.1f}m, 軌道半径: {layer_radius:.1f}m, ピッチ: {pitch_deg:.1f}°")

        angles = np.arange(0, 360, angle_step)
        for angle in angles:
            rad = math.radians(angle)
            x = poi_proj.x + layer_radius * math.cos(rad)
            y = poi_proj.y + layer_radius * math.sin(rad)
            
            wp_pt = gpd.GeoSeries([Point(x, y)], crs=f"EPSG:{local_epsg}").to_crs(epsg=4326).iloc[0]
            
            alt_w, alt_e = layer_alt, layer_alt
            if has_dem:
                with rasterio.open(dem_path) as src:
                    with WarpedVRT(src, crs="EPSG:4326") as vrt:
                        val = list(vrt.sample([(wp_pt.x, wp_pt.y)]))[0][0]
                        wp_ground_wgs84 = float(val) if val > -1000 else 0.0
                        alt_w = wp_ground_wgs84 + layer_alt
                        _, _, alt_e = transformer.transform(wp_pt.x, wp_pt.y, alt_w)

            wp_data.append({
                'id': wp_index,
                'lon': wp_pt.x,
                'lat': wp_pt.y,
                'alt_w': alt_w,
                'alt_e': alt_e,
                'pitch': pitch_deg
            })
            wp_index += 1

    # 頂点(直上/Nadir)の追加
    if add_top:
        print(f"[レイヤー構築] 直上(Nadir) -> 高度: {dome_radius:.1f}m, 軌道半径: 0.0m, ピッチ: -90.0°")
        alt_w, alt_e = dome_radius, dome_radius
        if has_dem:
            alt_w = poi_alt_wgs84 + dome_radius
            _, _, alt_e = transformer.transform(poi_wgs84.x, poi_wgs84.y, alt_w)
            
        wp_data.append({
            'id': wp_index,
            'lon': poi_wgs84.x,
            'lat': poi_wgs84.y,
            'alt_w': alt_w,
            'alt_e': alt_e,
            'pitch': -90.0
        })

    poi_data = {'lon': poi_wgs84.x, 'lat': poi_wgs84.y, 'alt_w': poi_alt_wgs84}
    return wp_data, poi_data, execute_height_mode, coord_height_mode

# ==========================================================
# 4. XML構築モジュール
# ==========================================================
def add_elem(parent, tag, text=None):
    if tag.startswith("wpml:"):
        elem = ET.SubElement(parent, f"{{{WPML_NS}}}{tag[5:]}")
    else:
        elem = ET.SubElement(parent, f"{{{KML_NS}}}{tag}")
    if text is not None: 
        elem.text = str(text)
    return elem

def generate_dji_xml(wp_data, poi_data, exec_h_mode, coord_h_mode):
    ET.register_namespace('', KML_NS)
    ET.register_namespace('wpml', WPML_NS)
    timestamp = str(int(time.time() * 1000))
    total_len = len(wp_data)

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

    gwh_t = add_elem(folder_t, "wpml:globalWaypointHeadingParam")
    add_elem(gwh_t, "wpml:waypointHeadingMode", "towardPOI")
    add_elem(gwh_t, "wpml:waypointPoiPoint", f"{poi_data['lat']},{poi_data['lon']},{poi_data['alt_w']}")
    add_elem(gwh_t, "wpml:waypointHeadingPathMode", "followBadArc")

    pp_t = add_elem(folder_t, "wpml:payloadParam")
    add_elem(pp_t, "wpml:payloadPositionIndex", "0")
    add_elem(pp_t, "wpml:imageFormat", IMAGE_FORMAT_STR)

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
        current_pitch = wp['pitch']
        
        pm_t = add_elem(folder_t, "Placemark")
        pt_t = add_elem(pm_t, "Point")
        add_elem(pt_t, "coordinates", f"{wp['lon']},{wp['lat']}")
        add_elem(pm_t, "wpml:index", str(i))
        add_elem(pm_t, "wpml:ellipsoidHeight", str(round(wp['alt_w'], 3)))
        add_elem(pm_t, "wpml:height", str(round(wp['alt_e'], 3)))
        add_elem(pm_t, "wpml:useGlobalHeight", "0")
        add_elem(pm_t, "wpml:useGlobalSpeed", "1")
        add_elem(pm_t, "wpml:useGlobalHeadingParam", "1")
        add_elem(pm_t, "wpml:useGlobalTurnParam", "1")
        add_elem(pm_t, "wpml:gimbalPitchAngle", str(round(current_pitch, 1)))
        add_elem(pm_t, "wpml:useStraightLine", "0")
        
        tp_t = add_elem(pm_t, "wpml:waypointTurnParam")
        if i == total_len - 1:
            add_elem(tp_t, "wpml:waypointTurnMode", "toPointAndStopWithDiscontinuityCurvature")
            add_elem(tp_t, "wpml:waypointTurnDampingDist", "0")
        else:
            add_elem(tp_t, "wpml:waypointTurnMode", "toPointAndPassWithContinuityCurvature")
            add_elem(tp_t, "wpml:waypointTurnDampingDist", "0.2")

        ag_t = add_elem(pm_t, "wpml:actionGroup")
        add_elem(ag_t, "wpml:actionGroupId", str(i))
        add_elem(ag_t, "wpml:actionGroupStartIndex", str(i))
        add_elem(ag_t, "wpml:actionGroupEndIndex", str(i))
        add_elem(ag_t, "wpml:actionGroupMode", "sequence")
        trig_t = add_elem(ag_t, "wpml:actionTrigger")
        add_elem(trig_t, "wpml:actionTriggerType", "reachPoint")
        
        act_idx = 0
        if i == 0:
            act_gimbal = add_elem(ag_t, "wpml:action")
            add_elem(act_gimbal, "wpml:actionId", str(act_idx))
            add_elem(act_gimbal, "wpml:actionActuatorFunc", "gimbalRotate")
            act_g_p = add_elem(act_gimbal, "wpml:actionActuatorFuncParam")
            add_elem(act_g_p, "wpml:gimbalRotateMode", "absoluteAngle")
            add_elem(act_g_p, "wpml:gimbalPitchRotateEnable", "1")
            add_elem(act_g_p, "wpml:gimbalPitchRotateAngle", str(round(current_pitch, 1)))
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

        act_photo = add_elem(ag_t, "wpml:action")
        add_elem(act_photo, "wpml:actionId", str(act_idx))
        add_elem(act_photo, "wpml:actionActuatorFunc", "takePhoto")
        act_p_p = add_elem(act_photo, "wpml:actionActuatorFuncParam")
        add_elem(act_p_p, "wpml:fileSuffix", f"Dome_{i}")
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
            add_elem(act_g_p_w, "wpml:gimbalPitchRotateAngle", str(round(current_pitch, 1)))
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
        add_elem(act_p_p_w, "wpml:fileSuffix", f"Dome_{i}")
        add_elem(act_p_p_w, "wpml:payloadPositionIndex", "0")
        add_elem(act_p_p_w, "wpml:useGlobalPayloadLensIndex", "1")

    return kml_t, kml_w

# -------------------------
# 5. KMZ出力モジュール
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
# 6. メイン実行ブロック
# ==========================================================
print("========================================")
print(f" ドーム型フライト生成 ({DRONE_TYPE} + P1) ")
print("========================================\n")

print(f"[GIS処理] ドーム半径 {DOME_RADIUS_M}m でウェイポイントを計算中...")
wp_data, poi_data, exec_h_mode, coord_h_mode = process_dome_waypoints(
    SHP_PATH, 
    DEM_PATH, 
    DOME_RADIUS_M, 
    target_elev_angles,
    ANGLE_STEP_DEG, 
    target_include_top,
    LOCAL_EPSG_CODE
)

print(f"[XML生成] KML/WPML構造を構築中...")
print(f"  └ 総撮影枚数: {len(wp_data)}枚")
template_tree, waylines_tree = generate_dji_xml(wp_data, poi_data, exec_h_mode, coord_h_mode)

print("[出力処理] KMZファイルをパッケージング中...")
export_kmz(template_tree, waylines_tree, OUTPUT_KMZ)

print("--- 処理がすべて完了しました ---")