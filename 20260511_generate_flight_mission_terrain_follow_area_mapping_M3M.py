# Author: Mashiro
# Last update: 2026/5/11
# Descripion: generate area mission terrain follow with under the 25 m

import os
import sys
import math
import time
import zipfile
import xml.etree.ElementTree as ET
import numpy as np
import geopandas as gpd
from shapely.geometry import Point, Polygon, box
import shapely.affinity
import rasterio
import fiona

# geopandasでKMLを読み込むためのドライバー有効化
fiona.drvsupport.supported_drivers['KML'] = 'rw'

# ==========================================================
# 1. グローバル設定 (ユーザー設定値)
# ==========================================================
# --- 入出力ファイル ---
KML_FILE_PATH = "target_area.kml"   # 対象エリアのポリゴンKMLファイル
DEM_FILE_PATH = "your_dem_data.tif" # DEMファイル（無い場合は相対高度モード）
MISSION_NAME = "M3M_Mapping_001"    # 出力されるミッション名
OUTPUT_DIR = "./missions"           # 出力先ディレクトリ

# ==========================================
# ★ M3M センサー・カメラ設定 (RGB / MS 切替)
# ==========================================
# True : 「RGB + MS(マルチスペクトル)」同時撮影 (TIFF 2秒制約 / 狭角FOV)
# False: 「RGB(可視光)」のみ撮影 (JPEG 0.7秒制約 / 広角FOV)
CAPTURE_MS_SENSOR = True

# --- フライト基本設定 ---
TARGET_ALTITUDE_M = 30.0            # 目標対地高度 (m)
FRONT_OVERLAP = 0.80                # フロントラップ (80% = 0.80)
SIDE_OVERLAP = 0.70                 # サイドラップ (70% = 0.70)
COURSE_ANGLE_DEG = 45.0             # コース角度 (度)

# ★先ほど抜け落ちていた設定★
TARGET_TIME_INTERVAL_SEC = 2.0      # 目標とする撮影インターバル (秒)

# --- 速度の安全設定 ---
MAX_FLIGHT_SPEED = 15.0             # 機体の最高許容速度 (m/s)
RC_LOST_ACTION = "goBack"           # 信号切断時アクション: goBack(RTH)

# --- フォーカス設定 ---
USE_INFINITY_FOCUS = True           # True: 無限遠(∞) / False: 最初のWPで一度だけAF

# --- アルゴリズム設定 ---
SAFETY_MARGIN_LAYERS = 1            # 外周に追加する安全マージン

# ==========================================
# 内部ID & M3M ハードウェア仕様マッピング
# ==========================================
# WPMLハードウェアID
DRONE_ENUM = "77"
DRONE_SUB_ENUM = "2"
PAYLOAD_ENUM = "68"

# センサー設定に応じたパラメータの動的切り替え
if CAPTURE_MS_SENSOR:
    # --- RGB + MS モード ---
    IMAGE_FORMAT_STR = "wide,narrow_band"
    MIN_INTERVAL_SEC = 2.0          # MS(TIFF)の最小インターバル
    # MSカメラのFOV (H: 61.2°, V: 48.10°) を使用して隙間を防ぐ
    H_FOV_DEG = 61.2
    V_FOV_DEG = 48.10
else:
    # --- RGBのみ モード ---
    IMAGE_FORMAT_STR = "wide"
    MIN_INTERVAL_SEC = 0.7          # RGB(JPEG)の最小インターバル
    # RGBカメラのFOV (対角84°, 4:3比率) から計算
    diag_rad = math.radians(84.0)
    aspect_diag = math.sqrt(4**2 + 3**2) # 5.0
    H_FOV_DEG = math.degrees(2 * math.atan((4 / aspect_diag) * math.tan(diag_rad / 2)))
    V_FOV_DEG = math.degrees(2 * math.atan((3 / aspect_diag) * math.tan(diag_rad / 2)))


# ==========================================
# 2. コアロジック (ルート計算)
# ==========================================
def load_polygon_from_kml(filepath):
    try:
        gdf_kml = gpd.read_file(filepath, driver='KML')
        polygons = gdf_kml[gdf_kml.geometry.type == 'Polygon']
        if polygons.empty: return None
        return polygons.geometry.iloc[0]
    except Exception:
        return None

def calc_grid_sizes():
    """設定されたFOVに基づいて地表面の撮影範囲とコース間隔を計算"""
    h_fov_rad = math.radians(H_FOV_DEG)
    v_fov_rad = math.radians(V_FOV_DEG)

    ground_w = 2 * TARGET_ALTITUDE_M * math.tan(h_fov_rad / 2)
    ground_h = 2 * TARGET_ALTITUDE_M * math.tan(v_fov_rad / 2)
    
    course_dist = ground_w * (1.0 - SIDE_OVERLAP)
    photo_dist = ground_h * (1.0 - FRONT_OVERLAP)
    return course_dist, photo_dist

def generate_route(polygon_wgs84, course_dist, photo_dist):
    gdf = gpd.GeoDataFrame(index=[0], crs='epsg:4326', geometry=[polygon_wgs84])
    poly_meters = gdf.to_crs(epsg=3857).geometry[0]
    centroid = poly_meters.centroid
    rotated_poly = shapely.affinity.rotate(poly_meters, -COURSE_ANGLE_DEG, origin=centroid)
    minx, miny, maxx, maxy = rotated_poly.bounds

    xs = np.arange(minx - photo_dist * 5, maxx + photo_dist * 5, photo_dist)
    ys = np.arange(miny - course_dist * 5, maxy + course_dist * 5, course_dist)
    
    active_cells = set()
    for i in range(len(xs) - 1):
        for j in range(len(ys) - 1):
            if rotated_poly.intersects(box(xs[i], ys[j], xs[i+1], ys[j+1])):
                active_cells.add((i, j))

    dilated_cells = set(active_cells)
    for _ in range(SAFETY_MARGIN_LAYERS):
        expansion = set()
        for (i, j) in dilated_cells:
            for di in [-1, 0, 1]:
                for dj in [-1, 0, 1]:
                    expansion.add((i + di, j + dj))
        dilated_cells.update(expansion)

    lines = {}
    for (i, j) in dilated_cells:
        lines.setdefault(j, []).append(i)

    waypoints = []
    is_left_to_right = True
    for j in sorted(lines.keys()):
        row_i = sorted(lines[j])
        if not is_left_to_right: row_i.reverse()
        for i in row_i:
            waypoints.append(Point(xs[i] + photo_dist / 2, ys[j] + course_dist / 2))
        is_left_to_right = not is_left_to_right

    waypoints_restored = [shapely.affinity.rotate(pt, COURSE_ANGLE_DEG, origin=centroid) for pt in waypoints]
    route_latlon = gpd.GeoDataFrame(geometry=waypoints_restored, crs='epsg:3857').to_crs(epsg=4326)

    route_3d = []
    height_mode = "relativeToStartPoint"
    if os.path.exists(DEM_FILE_PATH):
        try:
            with rasterio.open(DEM_FILE_PATH) as src:
                height_mode = "WGS84"
                for pt in route_latlon.geometry:
                    for val in src.sample([(pt.x, pt.y)]):
                        route_3d.append((pt.x, pt.y, float(val[0]) + TARGET_ALTITUDE_M))
                        break
        except Exception as e:
            print(f"⚠️ DEM読み込みエラー: {e}")
            for pt in route_latlon.geometry:
                route_3d.append((pt.x, pt.y, TARGET_ALTITUDE_M))
    else:
        for pt in route_latlon.geometry:
            route_3d.append((pt.x, pt.y, TARGET_ALTITUDE_M))

    return route_3d, height_mode

# ==========================================
# 3. ElementTree XML エクスポート機能
# ==========================================
KML_NS = "http://www.opengis.net/kml/2.2"
WPML_NS = "http://www.dji.com/wpmz/1.0.6"
ET.register_namespace('', KML_NS)
ET.register_namespace('wpml', WPML_NS)

def add_elem(parent, tag, text=None):
    if tag.startswith("wpml:"):
        elem = ET.SubElement(parent, f"{{{WPML_NS}}}{tag[5:]}")
    else:
        elem = ET.SubElement(parent, f"{{{KML_NS}}}{tag}")
    if text is not None: 
        elem.text = str(text)
    return elem

def build_actions(pm, i, total_len, interval_sec):
    if i == 0:
        # 開始時のジンバル・フォーカス制御
        ag0 = add_elem(pm, "wpml:actionGroup")
        add_elem(ag0, "wpml:actionGroupId", "0")
        add_elem(ag0, "wpml:actionGroupStartIndex", "0")
        add_elem(ag0, "wpml:actionGroupEndIndex", "0")
        add_elem(ag0, "wpml:actionGroupMode", "sequence")
        trig0 = add_elem(ag0, "wpml:actionTrigger")
        add_elem(trig0, "wpml:actionTriggerType", "reachPoint")
        
        act_idx = 0
        act_gimbal = add_elem(ag0, "wpml:action")
        add_elem(act_gimbal, "wpml:actionId", str(act_idx))
        add_elem(act_gimbal, "wpml:actionActuatorFunc", "gimbalRotate")
        act_gimbal_p = add_elem(act_gimbal, "wpml:actionActuatorFuncParam")
        add_elem(act_gimbal_p, "wpml:gimbalRotateMode", "absoluteAngle")
        add_elem(act_gimbal_p, "wpml:gimbalPitchRotateEnable", "1")
        add_elem(act_gimbal_p, "wpml:gimbalPitchRotateAngle", "-90.0")
        add_elem(act_gimbal_p, "wpml:gimbalRollRotateEnable", "0")
        add_elem(act_gimbal_p, "wpml:gimbalRollRotateAngle", "0")
        add_elem(act_gimbal_p, "wpml:gimbalYawRotateEnable", "0")
        add_elem(act_gimbal_p, "wpml:gimbalYawRotateAngle", "0")
        add_elem(act_gimbal_p, "wpml:gimbalRotateTimeEnable", "0")
        add_elem(act_gimbal_p, "wpml:gimbalRotateTime", "0")
        add_elem(act_gimbal_p, "wpml:payloadPositionIndex", "0")
        act_idx += 1

        if not USE_INFINITY_FOCUS:
            act_focus = add_elem(ag0, "wpml:action")
            add_elem(act_focus, "wpml:actionId", str(act_idx))
            add_elem(act_focus, "wpml:actionActuatorFunc", "focus")
            act_focus_p = add_elem(act_focus, "wpml:actionActuatorFuncParam")
            add_elem(act_focus_p, "wpml:payloadPositionIndex", "0")
            add_elem(act_focus_p, "wpml:isPointFocus", "1")
            add_elem(act_focus_p, "wpml:focusX", "0.5")
            add_elem(act_focus_p, "wpml:focusY", "0.5")
            add_elem(act_focus_p, "wpml:isInfiniteFocus", "0")
            act_idx += 1

        # インターバル撮影 (全区間)
        ag1 = add_elem(pm, "wpml:actionGroup")
        add_elem(ag1, "wpml:actionGroupId", "1")
        add_elem(ag1, "wpml:actionGroupStartIndex", "0")
        add_elem(ag1, "wpml:actionGroupEndIndex", str(total_len - 1))
        add_elem(ag1, "wpml:actionGroupMode", "parallel") 
        trig1 = add_elem(ag1, "wpml:actionTrigger")
        add_elem(trig1, "wpml:actionTriggerType", "multipleTiming")
        add_elem(trig1, "wpml:actionTriggerParam", str(interval_sec)) 
        
        act_photo = add_elem(ag1, "wpml:action")
        add_elem(act_photo, "wpml:actionId", "0")
        add_elem(act_photo, "wpml:actionActuatorFunc", "takePhoto")
        act_photo_p = add_elem(act_photo, "wpml:actionActuatorFuncParam")
        add_elem(act_photo_p, "wpml:fileSuffix", "interval")
        add_elem(act_photo_p, "wpml:payloadPositionIndex", "0")
        # ★ グローバルレンズ設定を参照
        add_elem(act_photo_p, "wpml:useGlobalPayloadLensIndex", "1")

def export_kmz_with_et(waypoints, height_mode, speed, interval_sec):
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    kmz_path = os.path.join(OUTPUT_DIR, f"{MISSION_NAME}.kmz")
    timestamp = str(int(time.time() * 1000))
    total_wp = len(waypoints)

    # --------------------------------------------------
    # A. template.kml
    # --------------------------------------------------
    kml_t = ET.Element(f"{{{KML_NS}}}kml")
    doc_t = add_elem(kml_t, "Document")
    add_elem(doc_t, "wpml:author", "AutoMapper")
    add_elem(doc_t, "wpml:createTime", timestamp)
    add_elem(doc_t, "wpml:updateTime", timestamp)

    mc_t = add_elem(doc_t, "wpml:missionConfig")
    add_elem(mc_t, "wpml:flyToWaylineMode", "safely")
    add_elem(mc_t, "wpml:finishAction", "goHome")
    add_elem(mc_t, "wpml:exitOnRCLost", "executeLostAction")
    add_elem(mc_t, "wpml:executeRCLostAction", RC_LOST_ACTION)
    add_elem(mc_t, "wpml:takeOffSecurityHeight", "20.0")
    add_elem(mc_t, "wpml:globalTransitionalSpeed", "10.0")
    
    # 機体・ペイロード情報
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
    add_elem(sys_param_t, "wpml:heightMode", height_mode)
    add_elem(sys_param_t, "wpml:positioningType", "GPS")
    
    add_elem(folder_t, "wpml:autoFlightSpeed", str(speed))
    add_elem(folder_t, "wpml:gimbalPitchMode", "usePointSetting")
    add_elem(folder_t, "wpml:globalWaypointTurnMode", "toPointAndPassWithContinuityCurvature")

    gwh_t = add_elem(folder_t, "wpml:globalWaypointHeadingParam")
    add_elem(gwh_t, "wpml:waypointHeadingMode", "followWayline")

    # M3M用 グローバルレンズ設定 (wide または wide,narrow_band)
    pp_t = add_elem(folder_t, "wpml:payloadParam")
    add_elem(pp_t, "wpml:payloadPositionIndex", "0")
    add_elem(pp_t, "wpml:imageFormat", IMAGE_FORMAT_STR)

    for i, (lon, lat, alt) in enumerate(waypoints):
        pm_t = add_elem(folder_t, "Placemark")
        pt_t = add_elem(pm_t, "Point")
        add_elem(pt_t, "coordinates", f"{lon},{lat}")
        add_elem(pm_t, "wpml:index", str(i))
        add_elem(pm_t, "wpml:ellipsoidHeight", str(round(alt, 3)))
        add_elem(pm_t, "wpml:height", str(round(alt, 3)))
        add_elem(pm_t, "wpml:useGlobalHeight", "0")
        add_elem(pm_t, "wpml:useGlobalSpeed", "1")
        add_elem(pm_t, "wpml:useGlobalHeadingParam", "1")
        add_elem(pm_t, "wpml:useGlobalTurnParam", "1")
        add_elem(pm_t, "wpml:gimbalPitchAngle", "-90.0")
        add_elem(pm_t, "wpml:useStraightLine", "1")
        
        tp_t = add_elem(pm_t, "wpml:waypointTurnParam")
        if i == total_wp - 1:
            add_elem(tp_t, "wpml:waypointTurnMode", "toPointAndStopWithDiscontinuityCurvature")
            add_elem(tp_t, "wpml:waypointTurnDampingDist", "0")
        else:
            add_elem(tp_t, "wpml:waypointTurnMode", "toPointAndPassWithContinuityCurvature")
            add_elem(tp_t, "wpml:waypointTurnDampingDist", "0.2")

        build_actions(pm_t, i, total_wp, interval_sec)

    # --------------------------------------------------
    # B. waylines.wpml
    # --------------------------------------------------
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
    add_elem(folder_w, "wpml:executeHeightMode", height_mode)
    add_elem(folder_w, "wpml:waylineId", "0")
    add_elem(folder_w, "wpml:autoFlightSpeed", str(speed))

    pp_w = add_elem(folder_w, "wpml:payloadParam")
    add_elem(pp_w, "wpml:payloadPositionIndex", "0")
    add_elem(pp_w, "wpml:imageFormat", IMAGE_FORMAT_STR)

    for i, (lon, lat, alt) in enumerate(waypoints):
        pm_w = add_elem(folder_w, "Placemark")
        pt_w = add_elem(pm_w, "Point")
        add_elem(pt_w, "coordinates", f"{lon},{lat}")
        add_elem(pm_w, "wpml:index", str(i))
        add_elem(pm_w, "wpml:executeHeight", str(round(alt, 3)))
        add_elem(pm_w, "wpml:waypointSpeed", str(speed))
        
        hp_w = add_elem(pm_w, "wpml:waypointHeadingParam")
        add_elem(hp_w, "wpml:waypointHeadingMode", "followWayline")
        
        tp_w = add_elem(pm_w, "wpml:waypointTurnParam")
        if i == total_wp - 1:
            add_elem(tp_w, "wpml:waypointTurnMode", "toPointAndStopWithDiscontinuityCurvature")
            add_elem(tp_w, "wpml:waypointTurnDampingDist", "0")
        else:
            add_elem(tp_w, "wpml:waypointTurnMode", "toPointAndPassWithContinuityCurvature")
            add_elem(tp_w, "wpml:waypointTurnDampingDist", "0.2")
            
        build_actions(pm_w, i, total_wp, interval_sec)

    # --------------------------------------------------
    # C. ZIP(KMZ)圧縮と保存
    # --------------------------------------------------
    TEMP_WPMZ = os.path.join(OUTPUT_DIR, "wpmz_temp")
    os.makedirs(TEMP_WPMZ, exist_ok=True)
    if hasattr(ET, 'indent'): 
        ET.indent(kml_t, space="  ")
        ET.indent(kml_w, space="  ")

    tmp_t = os.path.join(TEMP_WPMZ, "template.kml")
    tmp_w = os.path.join(TEMP_WPMZ, "waylines.wpml")
    
    with open(tmp_t, 'w', encoding='utf-8') as f: 
        f.write(ET.tostring(kml_t, encoding="utf-8", xml_declaration=True).decode('utf-8'))
    with open(tmp_w, 'w', encoding='utf-8') as f: 
        f.write(ET.tostring(kml_w, encoding="utf-8", xml_declaration=True).decode('utf-8'))

    with zipfile.ZipFile(kmz_path, 'w', zipfile.ZIP_DEFLATED) as kmz:
        kmz.write(tmp_t, arcname="wpmz/template.kml")
        kmz.write(tmp_w, arcname="wpmz/waylines.wpml")

    os.remove(tmp_t)
    os.remove(tmp_w)
    os.rmdir(TEMP_WPMZ)
    print(f"\n🎉 成功！ '{os.path.basename(kmz_path)}' を出力しました。")
    print(f"👉 保存先: {os.path.abspath(kmz_path)}")

# ==========================================
# 4. 実行ブロック (メイン処理)
# ==========================================
sensor_label = "RGB + MS (マルチスペクトル)" if CAPTURE_MS_SENSOR else "RGBのみ"
print("========================================")
print(f" M3M ポリゴンマッピング ({sensor_label}) ")
print("========================================\n")

target_polygon = load_polygon_from_kml(KML_FILE_PATH)
if target_polygon is None:
    print(f"❌ エラー: KMLファイル ({KML_FILE_PATH}) を読み込めませんでした。")
    sys.exit(1)

course_spacing_m, photo_spacing_m = calc_grid_sizes()
print(f"[カメラ仕様] H_FOV: {H_FOV_DEG:.1f}°, V_FOV: {V_FOV_DEG:.1f}°")
print(f"[距離計算] 撮影間隔(進行): {photo_spacing_m:.2f}m, コース間隔(横): {course_spacing_m:.2f}m")

# 指定されたインターバルから要求速度を計算
calc_speed = photo_spacing_m / TARGET_TIME_INTERVAL_SEC
# M3Mのハードウェア限界（JPEG:0.7s, TIFF:2.0s）に基づく絶対的な安全最高速度
max_safe_speed = photo_spacing_m / MIN_INTERVAL_SEC

if calc_speed > max_safe_speed:
    final_speed = max_safe_speed
    actual_interval = MIN_INTERVAL_SEC
    print(f"⚠️ 警告: 設定された速度ではカメラの保存処理が追いつきません。速度を下げます。")
elif calc_speed > MAX_FLIGHT_SPEED:
    final_speed = MAX_FLIGHT_SPEED
    actual_interval = photo_spacing_m / MAX_FLIGHT_SPEED
else:
    final_speed = calc_speed
    actual_interval = TARGET_TIME_INTERVAL_SEC

print(f"[速度・撮影設定] 飛行速度: {final_speed:.2f} m/s, インターバル: {actual_interval:.2f} 秒")

final_waypoints, height_mode_tag = generate_route(target_polygon, course_spacing_m, photo_spacing_m)

print(f"\n[エクスポート] KMZファイルの構築を開始します...")
export_kmz_with_et(
    waypoints=final_waypoints,
    height_mode=height_mode_tag,
    speed=final_speed,
    interval_sec=actual_interval
)