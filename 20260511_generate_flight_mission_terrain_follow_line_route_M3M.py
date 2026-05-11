# Author: Mashiro
# Last update: 2026/5/11
# Descripion: generate waypoint mission with terrain follow; M3M

# load modules
import math
import os
import time
import zipfile
import xml.etree.ElementTree as ET

import geopandas as gpd
import rasterio
from rasterio.vrt import WarpedVRT
from shapely.geometry import Point
from shapely.ops import linemerge
from pyproj import Transformer
from pyproj.network import set_network_enabled

# ==========================================================
# 1. ユーザー設定 (ファイルパスとフライトオプション)
# ==========================================================

# --- 入力ファイル (スクリプトと同じ階層に置くか、フルパスを指定) ---
LINE_SHP_PATH = "target_lines.shp"      # 飛行ルートのラインデータ
DEM_PATH      = "target_dem.tif"        # DEMデータ (無い場合は相対高度になります)

# --- 出力設定 ---
OUTPUT_DIR    = "./missions"            # 出力先の親フォルダ
MISSION_NAME  = "m3m_interval_mission"  # ミッション名
OUTPUT_KMZ    = os.path.join(OUTPUT_DIR, f"{MISSION_NAME}.kmz")

# --- 座標系設定 ---
LOCAL_EPSG_CODE = 6680

# ==========================================
# ★ カメラ・センサー設定 (M3M: RGB / MS 切替)
# ==========================================
# True : RGBとMS(マルチスペクトル)を同時撮影
# False: RGB(可視光)のみ撮影
CAPTURE_MS_SENSOR = True

# --- 機首方位 (Yaw) 設定 ---
USE_FOLLOW_WAYLINE = True               # True: ルートに沿って機首を向ける / False: 下記の固定角度を使用
FIXED_YAW_DEG      = 0.0                # USE_FOLLOW_WAYLINE = False の時に全WPに適用される機首方位(度)

# --- フライト・撮影設定 ---
TARGET_AGL    = 3.0       # 対地目標高度 (m) ※相対高度モード時はこれが飛行高度になります
INTERVAL_DIST = 5.0       # ラインを分割する間隔 (m)
FLIGHT_SPEED  = 1.0       # 飛行速度 (m/s)
INTERVAL_TIME = 2.0       # インターバル撮影の間隔 (秒)
GIMBAL_PITCH  = -90.0     # ジンバル角度 (真下)

# ★通信ロスト設定: goBack(RTH) / hover / continue
RC_LOST_ACTION = "goBack" 

# --- XMLネームスペースと固定ハードウェアID ---
KML_NS  = "http://www.opengis.net/kml/2.2"
WPML_NS = "http://www.dji.com/wpmz/1.0.6"

DRONE_ENUM     = "77"   # Mavic 3 Enterprise Series
DRONE_SUB_ENUM = "2"    # M3M (Mavic 3 Multispectral)
PAYLOAD_ENUM   = "68"   # M3M Payload

# センサー設定から保存フォーマット文字列を生成 (WPML仕様準拠)
IMAGE_FORMAT_STR = "wide,narrow_band" if CAPTURE_MS_SENSOR else "wide"


# ==========================================================
# 2. 関数定義 (内部ロジック群)
# ==========================================================

# -------------------------
# A. GIS処理モジュール
# -------------------------
def process_lines_to_waypoints(line_shp, dem_path, target_agl, interval_dist, local_epsg):
    line_gdf = gpd.read_file(line_shp)
    if line_gdf.crs is None: 
        line_gdf = line_gdf.set_crs(epsg=4326)
    
    line_gdf_proj = line_gdf.to_crs(epsg=local_epsg)

    line_geom = line_gdf_proj.geometry.union_all() if hasattr(line_gdf_proj.geometry, 'union_all') else line_gdf_proj.geometry.unary_union
    if line_geom.geom_type == 'MultiLineString':
        line_geom = linemerge(line_geom)
    lines = list(line_geom.geoms) if line_geom.geom_type == 'MultiLineString' else [line_geom]

    points_proj = []
    for single_line in lines:
        current_dist = 0
        while current_dist <= single_line.length:
            points_proj.append(single_line.interpolate(current_dist))
            current_dist += interval_dist
        if current_dist - interval_dist < single_line.length:
            points_proj.append(single_line.interpolate(single_line.length))

    pts_gdf_proj = gpd.GeoSeries(points_proj, crs=f"EPSG:{local_epsg}")
    pts_gdf_4326 = pts_gdf_proj.to_crs(epsg=4326)
    
    sorted_gdf = gpd.GeoDataFrame(
        [{'new_id': i, 'geometry': p} for i, p in enumerate(pts_gdf_4326)], 
        crs="EPSG:4326"
    )

    execute_height_mode = "WGS84"
    coordinate_height_mode = "EGM96"

    if os.path.exists(dem_path):
        try:
            with rasterio.open(dem_path) as src:
                with WarpedVRT(src, crs="EPSG:4326") as vrt:
                    coords = [(p.x, p.y) for p in sorted_gdf.geometry]
                    raw_elevations = [val[0] for val in vrt.sample(coords)]

            sorted_gdf['flight_alt_wgs84'] = [float(e) + target_agl if e > -1000 else 0 for e in raw_elevations]

            set_network_enabled(True)
            transformer = Transformer.from_crs("EPSG:4979", "EPSG:4326+5773", always_xy=True)
            egm96_alts = []
            for index, row in sorted_gdf.iterrows():
                lon, lat, alt_w = row.geometry.x, row.geometry.y, row['flight_alt_wgs84']
                _, _, alt_e = transformer.transform(lon, lat, alt_w)
                egm96_alts.append(round(alt_e, 3))
            
            sorted_gdf['flight_alt_egm96'] = egm96_alts
            print("[高度処理] DEMを検出。地形フォロー(WGS84/EGM96)を適用します。")
        except Exception as e:
            print(f"⚠️ DEM処理エラー ({e}): 離陸地点相対高度モードに切り替えます。")
            execute_height_mode = "relativeToStartPoint"
            coordinate_height_mode = "relativeToStartPoint"
            sorted_gdf['flight_alt_wgs84'] = target_agl
            sorted_gdf['flight_alt_egm96'] = target_agl
    else:
        print("⚠️ DEM未検出: 離陸地点からの相対高度モードを適用します。")
        execute_height_mode = "relativeToStartPoint"
        coordinate_height_mode = "relativeToStartPoint"
        sorted_gdf['flight_alt_wgs84'] = target_agl
        sorted_gdf['flight_alt_egm96'] = target_agl

    return sorted_gdf, execute_height_mode, coordinate_height_mode

# -------------------------
# B. XML構築モジュール
# -------------------------
def add_elem(parent, tag, text=None):
    if tag.startswith("wpml:"):
        elem = ET.SubElement(parent, f"{{{WPML_NS}}}{tag[5:]}")
    else:
        elem = ET.SubElement(parent, f"{{{KML_NS}}}{tag}")
    if text is not None: 
        elem.text = str(text)
    return elem

def build_actions(pm, i, total_len):
    if i == 0:
        ag0 = add_elem(pm, "wpml:actionGroup")
        add_elem(ag0, "wpml:actionGroupId", "0")
        add_elem(ag0, "wpml:actionGroupStartIndex", "0")
        add_elem(ag0, "wpml:actionGroupEndIndex", "0")
        add_elem(ag0, "wpml:actionGroupMode", "sequence")
        trig0 = add_elem(ag0, "wpml:actionTrigger")
        add_elem(trig0, "wpml:actionTriggerType", "reachPoint")
        
        act0 = add_elem(ag0, "wpml:action")
        add_elem(act0, "wpml:actionId", "0")
        add_elem(act0, "wpml:actionActuatorFunc", "gimbalRotate")
        act0_p = add_elem(act0, "wpml:actionActuatorFuncParam")
        add_elem(act0_p, "wpml:gimbalRotateMode", "absoluteAngle")
        add_elem(act0_p, "wpml:gimbalPitchRotateEnable", "1")
        add_elem(act0_p, "wpml:gimbalPitchRotateAngle", str(GIMBAL_PITCH))
        add_elem(act0_p, "wpml:gimbalRollRotateEnable", "0")
        add_elem(act0_p, "wpml:gimbalRollRotateAngle", "0")
        add_elem(act0_p, "wpml:gimbalYawRotateEnable", "0")
        add_elem(act0_p, "wpml:gimbalYawRotateAngle", "0")
        add_elem(act0_p, "wpml:gimbalRotateTimeEnable", "0")
        add_elem(act0_p, "wpml:gimbalRotateTime", "0")
        add_elem(act0_p, "wpml:payloadPositionIndex", "0")

        ag1 = add_elem(pm, "wpml:actionGroup")
        add_elem(ag1, "wpml:actionGroupId", "1")
        add_elem(ag1, "wpml:actionGroupStartIndex", "0")
        add_elem(ag1, "wpml:actionGroupEndIndex", str(total_len - 1))
        add_elem(ag1, "wpml:actionGroupMode", "parallel") 
        trig1 = add_elem(ag1, "wpml:actionTrigger")
        add_elem(trig1, "wpml:actionTriggerType", "multipleTiming")
        add_elem(trig1, "wpml:actionTriggerParam", str(int(INTERVAL_TIME))) 
        
        act1 = add_elem(ag1, "wpml:action")
        add_elem(act1, "wpml:actionId", "0")
        add_elem(act1, "wpml:actionActuatorFunc", "takePhoto")
        act1_p = add_elem(act1, "wpml:actionActuatorFuncParam")
        add_elem(act1_p, "wpml:fileSuffix", "interval")
        add_elem(act1_p, "wpml:payloadPositionIndex", "0")
        # ★ 個別指定をせず、グローバル設定を参照する
        add_elem(act1_p, "wpml:useGlobalPayloadLensIndex", "1")

def generate_dji_xml(waypoints_gdf, exec_height_mode, coord_height_mode, use_follow, fixed_yaw):
    ET.register_namespace('', KML_NS)
    ET.register_namespace('wpml', WPML_NS)
    timestamp = str(int(time.time() * 1000))
    total_len = len(waypoints_gdf)

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
    add_elem(mc_t, "wpml:takeOffSecurityHeight", "20")
    add_elem(mc_t, "wpml:globalTransitionalSpeed", "10")
    
    # ★ 指定の機体・ペイロードID
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

    # ★ グローバルなペイロード設定 (ここでRGB/MSの保存フォーマットを定義)
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
    add_elem(mc_w, "wpml:takeOffSecurityHeight", "20")
    add_elem(mc_w, "wpml:globalTransitionalSpeed", "10")
    
    di_w = add_elem(mc_w, "wpml:droneInfo")
    add_elem(di_w, "wpml:droneEnumValue", DRONE_ENUM)
    add_elem(di_w, "wpml:droneSubEnumValue", DRONE_SUB_ENUM)

    folder_w = add_elem(doc_w, "Folder")
    add_elem(folder_w, "wpml:templateId", "0")
    add_elem(folder_w, "wpml:executeHeightMode", exec_height_mode)
    add_elem(folder_w, "wpml:waylineId", "0")
    add_elem(folder_w, "wpml:autoFlightSpeed", str(FLIGHT_SPEED))

    # ★ waylines.wpml側にもグローバル設定を配置
    pp_w = add_elem(folder_w, "wpml:payloadParam")
    add_elem(pp_w, "wpml:payloadPositionIndex", "0")
    add_elem(pp_w, "wpml:imageFormat", IMAGE_FORMAT_STR)

    for index, row in waypoints_gdf.iterrows():
        lon, lat = row.geometry.x, row.geometry.y
        alt_w, alt_e, i = row['flight_alt_wgs84'], row['flight_alt_egm96'], int(row['new_id'])
        
        # --- Template Placemark ---
        pm_t = add_elem(folder_t, "Placemark")
        pt_t = add_elem(pm_t, "Point")
        add_elem(pt_t, "coordinates", f"{lon},{lat}")
        add_elem(pm_t, "wpml:index", str(i))
        add_elem(pm_t, "wpml:ellipsoidHeight", str(round(alt_w, 3)))
        add_elem(pm_t, "wpml:height", str(alt_e))
        add_elem(pm_t, "wpml:useGlobalHeight", "0")
        add_elem(pm_t, "wpml:useGlobalSpeed", "1")
        add_elem(pm_t, "wpml:useGlobalHeadingParam", "0")
        add_elem(pm_t, "wpml:gimbalPitchAngle", str(GIMBAL_PITCH))
        add_elem(pm_t, "wpml:useStraightLine", "1")
        
        if i == total_len - 1:
            add_elem(pm_t, "wpml:useGlobalTurnParam", "0")
        else:
            add_elem(pm_t, "wpml:useGlobalTurnParam", "1")
        
        hp_t = add_elem(pm_t, "wpml:waypointHeadingParam")
        if use_follow:
            add_elem(hp_t, "wpml:waypointHeadingMode", "followWayline")
        else:
            add_elem(hp_t, "wpml:waypointHeadingMode", "smoothTransition")
            add_elem(hp_t, "wpml:waypointHeadingAngle", str(fixed_yaw))

        tp_t = add_elem(pm_t, "wpml:waypointTurnParam")
        if i == total_len - 1:
            add_elem(tp_t, "wpml:waypointTurnMode", "toPointAndStopWithDiscontinuityCurvature")
            add_elem(tp_t, "wpml:waypointTurnDampingDist", "0")
        else:
            add_elem(tp_t, "wpml:waypointTurnMode", "toPointAndPassWithContinuityCurvature")
            add_elem(tp_t, "wpml:waypointTurnDampingDist", "0.2")
        
        build_actions(pm_t, i, total_len)

        # --- Waylines Placemark ---
        pm_w = add_elem(folder_w, "Placemark")
        pt_w = add_elem(pm_w, "Point")
        add_elem(pt_w, "coordinates", f"{lon},{lat}")
        add_elem(pm_w, "wpml:index", str(i))
        add_elem(pm_w, "wpml:executeHeight", str(round(alt_w, 3)))
        add_elem(pm_w, "wpml:waypointSpeed", str(FLIGHT_SPEED))
        
        hp_w = add_elem(pm_w, "wpml:waypointHeadingParam")
        if use_follow:
            add_elem(hp_w, "wpml:waypointHeadingMode", "followWayline")
        else:
            add_elem(hp_w, "wpml:waypointHeadingMode", "smoothTransition")
            add_elem(hp_w, "wpml:waypointHeadingAngle", str(fixed_yaw))
            add_elem(hp_w, "wpml:waypointHeadingAngleEnable", "1")
        
        tp_w = add_elem(pm_w, "wpml:waypointTurnParam")
        if i == total_len - 1:
            add_elem(tp_w, "wpml:waypointTurnMode", "toPointAndStopWithDiscontinuityCurvature")
            add_elem(tp_w, "wpml:waypointTurnDampingDist", "0")
        else:
            add_elem(tp_w, "wpml:waypointTurnMode", "toPointAndPassWithContinuityCurvature")
            add_elem(tp_w, "wpml:waypointTurnDampingDist", "0.2")
        
        build_actions(pm_w, i, total_len)

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
print(f"--- ミッション '{MISSION_NAME}' (M3M専用: インターバル撮影) の構築を開始 ---")

print(f"[GIS処理] ラインを {INTERVAL_DIST}m 間隔で分割し、DEM高度を取得中...")
processed_waypoints, exec_h_mode, coord_h_mode = process_lines_to_waypoints(
    LINE_SHP_PATH, 
    DEM_PATH, 
    TARGET_AGL, 
    INTERVAL_DIST,
    LOCAL_EPSG_CODE
)

sensor_mode_msg = "RGB + MS (マルチスペクトル)" if CAPTURE_MS_SENSOR else "RGBのみ"
yaw_mode_msg = "ルート追従 (followWayline)" if USE_FOLLOW_WAYLINE else f"固定角度 ({FIXED_YAW_DEG}°)"
print(f"[XML生成] センサー: {sensor_mode_msg}")
print(f"[XML生成] Yaw制御: {yaw_mode_msg}")
template_tree, waylines_tree = generate_dji_xml(processed_waypoints, exec_h_mode, coord_h_mode, USE_FOLLOW_WAYLINE, FIXED_YAW_DEG)

print("[出力処理] KMZファイルをパッケージング中...")
export_kmz(template_tree, waylines_tree, OUTPUT_KMZ)

print("--- 処理がすべて完了しました ---")