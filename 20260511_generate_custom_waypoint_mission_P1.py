# Author: Mashiro
# Last update: 2026/5/11
# Descripion: generate waypoint mission with terrain follow; P1

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
from pyproj import Transformer
from pyproj.network import set_network_enabled

# ==========================================================
# 1. ユーザー設定 (ファイルパス・ロジック分岐・フライトオプション)
# ==========================================================

# --- 入力ファイル (絶対フルパスを指定) ---
SHP_PATH     = "target_points.shp"
DEM_PATH     = "your_dem_data.tif"

# --- 出力設定 ---
OUTPUT_DIR   = "./missions"                     # 出力先の親フォルダ
MISSION_NAME = "p1_waypoint_mission"            # ミッション名
OUTPUT_KMZ   = os.path.join(OUTPUT_DIR, f"{MISSION_NAME}.kmz")

# ==========================================
# ★ 機体・ペイロード(カメラ) 設定
# ==========================================
DRONE_TYPE   = "M400"           # "M400", "M350", "M300" のいずれかを指定
PAYLOAD_ENUM = "50"             # Zenmuse P1 のペイロードID (一般的に50)

# --- 機首方位 (Yaw) 設定 ---
USE_GLOBAL_YAW      = True                  # True: 全てのWPに一括適用 / False: 以下の個別リストを使用
GLOBAL_YAW_DEG      = 0.0                   # 一括適用時の角度（個別リストの要素が足りない時のデフォルト値にもなります）
INDIVIDUAL_YAW_LIST = [0.0, 45.0, 90.0]     # USE_GLOBAL_YAW = False の時に順番に適用されるリスト

# --- ロジック分岐フラグ ---
OPTIMIZE_ROUTE_SORTING = True               # True: 最短経路で並び替える / False: SHP元の順序を維持する

# --- フライト・アクション設定 ---
TARGET_AGL         = 3.0            # 対地目標高度 (m) ※相対高度モードの時はこれがそのまま飛行高度になります
FLIGHT_SPEED       = 5.0            # 飛行速度 (m/s)
HOVER_TIME         = 3              # 各WPでのホバリング待機時間 (秒)
GIMBAL_PITCH       = -90.0          # ジンバル角度 (真下)
TAKEOFF_SEC_HEIGHT = 20.0           # 離陸安全高度 (m)
GLOBAL_TRANS_SPEED = 10.0           # WP間以外の移動速度 (m/s)

# ★シグナルロスト設定: goBack(RTH) / hover / continue
RC_LOST_ACTION     = "goBack"

# --- XMLネームスペース ---
KML_NS  = "http://www.opengis.net/kml/2.2"
WPML_NS = "http://www.dji.com/wpmz/1.0.6"

# ==========================================
# 内部IDマッピング (DJI WPML仕様に基づく)
# ==========================================
if DRONE_TYPE == "M400":
    DRONE_ENUM = "103"
    DRONE_SUB_ENUM = "0"
elif DRONE_TYPE == "M350":
    DRONE_ENUM = "89"
    DRONE_SUB_ENUM = "0"
elif DRONE_TYPE == "M300":
    DRONE_ENUM = "60"
    DRONE_SUB_ENUM = "0"
else:
    raise ValueError("DRONE_TYPE は 'M400', 'M350', 'M300' のいずれかを指定してください。")

# P1はRGBカメラのため wide 固定
IMAGE_FORMAT_STR = "wide"


# ==========================================================
# 2. 関数定義 (内部ロジック群)
# ==========================================================

# -------------------------
# A. GIS処理モジュール
# -------------------------
def process_waypoints(shp_path, dem_path, target_agl, optimize_sorting, use_global_yaw, global_yaw, yaw_list):
    """ウェイポイントの読み込み、Yawの紐付け、ソート、およびDEM有無による高度モード分岐"""
    gdf = gpd.read_file(shp_path)
    if gdf.crs is None or gdf.crs.to_epsg() != 4326:
        gdf = gdf.to_crs(epsg=4326)

    # SHPの元の順序に合わせてYawを紐付ける
    yaws = []
    for i in range(len(gdf)):
        if use_global_yaw:
            yaws.append(global_yaw)
        else:
            if i < len(yaw_list):
                yaws.append(yaw_list[i])
            else:
                yaws.append(global_yaw)
    gdf['target_yaw'] = yaws

    # 経路ソートの分岐
    if optimize_sorting:
        nodes = [[idx, row.geometry.x, row.geometry.y, row['target_yaw']] for idx, row in gdf.iterrows()]
        sorted_nodes = [nodes.pop(0)]
        while nodes:
            curr = sorted_nodes[-1]
            nxt = min(nodes, key=lambda n: math.hypot(n[1]-curr[1], n[2]-curr[2]))
            sorted_nodes.append(nxt)
            nodes.remove(nxt)
        sorted_gdf = gpd.GeoDataFrame(
            [{'new_id': i, 'geometry': Point(n[1], n[2]), 'target_yaw': n[3]} for i, n in enumerate(sorted_nodes)], 
            crs="EPSG:4326"
        )
    else:
        sorted_gdf = gpd.GeoDataFrame(
            [{'new_id': i, 'geometry': row.geometry, 'target_yaw': row['target_yaw']} for i, row in gdf.iterrows()], 
            crs="EPSG:4326"
        )

    # ==========================================
    # 高度サンプリングとフォールバック処理
    # ==========================================
    execute_height_mode = "WGS84"
    coordinate_height_mode = "EGM96"

    if os.path.exists(dem_path):
        try:
            with rasterio.open(dem_path) as src:
                with WarpedVRT(src, crs="EPSG:4326") as vrt:
                    coords = [(p.x, p.y) for p in sorted_gdf.geometry]
                    raw_elevations = [val[0] for val in vrt.sample(coords)]

            sorted_gdf['flight_alt_wgs84'] = [float(e) + target_agl if e > -1000 else 0 for e in raw_elevations]

            # WGS84 -> EGM96 変換
            set_network_enabled(True)
            transformer = Transformer.from_crs("EPSG:4979", "EPSG:4326+5773", always_xy=True)
            egm96_alts = []
            for index, row in sorted_gdf.iterrows():
                lon, lat, alt_w = row.geometry.x, row.geometry.y, row['flight_alt_wgs84']
                _, _, alt_e = transformer.transform(lon, lat, alt_w)
                egm96_alts.append(round(alt_e, 3))
            
            sorted_gdf['flight_alt_egm96'] = egm96_alts
            print("[高度処理] DEMを検出。絶対高度(WGS84/EGM96)での地形フォローモードを適用します。")

        except Exception as e:
            print(f"⚠️ DEM処理エラー ({e}): 離陸地点からの相対高度モードに切り替えます。")
            execute_height_mode = "relativeToStartPoint"
            coordinate_height_mode = "relativeToStartPoint"
            sorted_gdf['flight_alt_wgs84'] = target_agl
            sorted_gdf['flight_alt_egm96'] = target_agl
    else:
        print("⚠️ DEMファイル未検出: 離陸地点からの相対高度(relativeToStartPoint)モードを適用します。")
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
    ag_reach = add_elem(pm, "wpml:actionGroup")
    add_elem(ag_reach, "wpml:actionGroupId", str(i))
    add_elem(ag_reach, "wpml:actionGroupStartIndex", str(i))
    add_elem(ag_reach, "wpml:actionGroupEndIndex", str(i))
    add_elem(ag_reach, "wpml:actionGroupMode", "sequence")
    
    trig_reach = add_elem(ag_reach, "wpml:actionTrigger")
    add_elem(trig_reach, "wpml:actionTriggerType", "reachPoint")
    
    act_idx = 0
    if i == 0:
        act_gimbal = add_elem(ag_reach, "wpml:action")
        add_elem(act_gimbal, "wpml:actionId", str(act_idx))
        add_elem(act_gimbal, "wpml:actionActuatorFunc", "gimbalRotate")
        act_gimbal_p = add_elem(act_gimbal, "wpml:actionActuatorFuncParam")
        add_elem(act_gimbal_p, "wpml:gimbalRotateMode", "absoluteAngle")
        add_elem(act_gimbal_p, "wpml:gimbalPitchRotateEnable", "1")
        add_elem(act_gimbal_p, "wpml:gimbalPitchRotateAngle", str(GIMBAL_PITCH))
        add_elem(act_gimbal_p, "wpml:gimbalRollRotateEnable", "0")
        add_elem(act_gimbal_p, "wpml:gimbalRollRotateAngle", "0")
        add_elem(act_gimbal_p, "wpml:gimbalYawRotateEnable", "0")
        add_elem(act_gimbal_p, "wpml:gimbalYawRotateAngle", "0")
        add_elem(act_gimbal_p, "wpml:gimbalRotateTimeEnable", "0")
        add_elem(act_gimbal_p, "wpml:gimbalRotateTime", "0")
        add_elem(act_gimbal_p, "wpml:payloadPositionIndex", "0")
        act_idx += 1

    act_hover = add_elem(ag_reach, "wpml:action")
    add_elem(act_hover, "wpml:actionId", str(act_idx))
    add_elem(act_hover, "wpml:actionActuatorFunc", "hover")
    act_hover_p = add_elem(act_hover, "wpml:actionActuatorFuncParam")
    add_elem(act_hover_p, "wpml:hoverTime", str(HOVER_TIME))
    act_idx += 1
    
    act_photo = add_elem(ag_reach, "wpml:action")
    add_elem(act_photo, "wpml:actionId", str(act_idx))
    add_elem(act_photo, "wpml:actionActuatorFunc", "takePhoto")
    act_photo_p = add_elem(act_photo, "wpml:actionActuatorFuncParam")
    add_elem(act_photo_p, "wpml:fileSuffix", f"WP_{i+1}")
    add_elem(act_photo_p, "wpml:payloadPositionIndex", "0")
    # ★ グローバルレンズ設定を参照する
    add_elem(act_photo_p, "wpml:useGlobalPayloadLensIndex", "1")

    if i < total_len - 1:
        ag_between = add_elem(pm, "wpml:actionGroup")
        add_elem(ag_between, "wpml:actionGroupId", str(i + 1000))
        add_elem(ag_between, "wpml:actionGroupStartIndex", str(i))
        add_elem(ag_between, "wpml:actionGroupEndIndex", str(i + 1))
        add_elem(ag_between, "wpml:actionGroupMode", "sequence")
        
        trig_between = add_elem(ag_between, "wpml:actionTrigger")
        add_elem(trig_between, "wpml:actionTriggerType", "betweenAdjacentPoints")
        
        act_even = add_elem(ag_between, "wpml:action")
        add_elem(act_even, "wpml:actionId", "0")
        add_elem(act_even, "wpml:actionActuatorFunc", "gimbalEvenlyRotate")
        act_even_p = add_elem(act_even, "wpml:actionActuatorFuncParam")
        add_elem(act_even_p, "wpml:gimbalPitchRotateAngle", str(GIMBAL_PITCH))
        add_elem(act_even_p, "wpml:gimbalRollRotateAngle", "0")
        add_elem(act_even_p, "wpml:payloadPositionIndex", "0")

def generate_dji_xml(waypoints_gdf, exec_height_mode, coord_height_mode):
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
    add_elem(mc_t, "wpml:takeOffSecurityHeight", str(TAKEOFF_SEC_HEIGHT))
    add_elem(mc_t, "wpml:globalTransitionalSpeed", str(GLOBAL_TRANS_SPEED))
    
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
    add_elem(folder_t, "wpml:globalWaypointTurnMode", "toPointAndStopWithDiscontinuityCurvature")

    gwh_t = add_elem(folder_t, "wpml:globalWaypointHeadingParam")
    add_elem(gwh_t, "wpml:waypointHeadingMode", "usePointSetting")

    # ★ グローバルなペイロード設定 (P1用に wide 固定)
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
    add_elem(mc_w, "wpml:takeOffSecurityHeight", str(TAKEOFF_SEC_HEIGHT))
    add_elem(mc_w, "wpml:globalTransitionalSpeed", str(GLOBAL_TRANS_SPEED))
    
    # ★ 指定の機体ID
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

    # --- ウェイポイントの反復生成 ---
    for index, row in waypoints_gdf.iterrows():
        lon, lat = row.geometry.x, row.geometry.y
        alt_w, alt_e = row['flight_alt_wgs84'], row['flight_alt_egm96']
        i = int(row['new_id'])
        current_yaw = row['target_yaw']

        # Template側のPlacemark
        pm_t = add_elem(folder_t, "Placemark")
        pt_t = add_elem(pm_t, "Point")
        add_elem(pt_t, "coordinates", f"{lon},{lat}")
        add_elem(pm_t, "wpml:index", str(i))
        add_elem(pm_t, "wpml:ellipsoidHeight", str(round(alt_w, 3)))
        add_elem(pm_t, "wpml:height", str(alt_e))
        add_elem(pm_t, "wpml:useGlobalHeight", "0")
        add_elem(pm_t, "wpml:useGlobalSpeed", "1")
        add_elem(pm_t, "wpml:useGlobalHeadingParam", "0") 
        add_elem(pm_t, "wpml:useGlobalTurnParam", "1")
        add_elem(pm_t, "wpml:gimbalPitchAngle", str(GIMBAL_PITCH))
        add_elem(pm_t, "wpml:useStraightLine", "1")
        
        hp_t = add_elem(pm_t, "wpml:waypointHeadingParam")
        add_elem(hp_t, "wpml:waypointHeadingMode", "smoothTransition")
        add_elem(hp_t, "wpml:waypointHeadingAngle", str(current_yaw))
        build_actions(pm_t, i, total_len)

        # Waylines側のPlacemark
        pm_w = add_elem(folder_w, "Placemark")
        pt_w = add_elem(pm_w, "Point")
        add_elem(pt_w, "coordinates", f"{lon},{lat}")
        add_elem(pm_w, "wpml:index", str(i))
        add_elem(pm_w, "wpml:executeHeight", str(round(alt_w, 3)))
        add_elem(pm_w, "wpml:waypointSpeed", str(FLIGHT_SPEED))
        
        hp_w = add_elem(pm_w, "wpml:waypointHeadingParam")
        add_elem(hp_w, "wpml:waypointHeadingMode", "smoothTransition")
        add_elem(hp_w, "wpml:waypointHeadingAngle", str(current_yaw))
        add_elem(hp_w, "wpml:waypointHeadingAngleEnable", "1")
        
        tp_w = add_elem(pm_w, "wpml:waypointTurnParam")
        add_elem(tp_w, "wpml:waypointTurnMode", "toPointAndStopWithDiscontinuityCurvature")
        add_elem(tp_w, "wpml:waypointTurnDampingDist", "0")
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
print("========================================")
print(f" ウェイポイント撮影 ({DRONE_TYPE} + P1) ")
print("========================================\n")

sort_status = "ON (最短経路最適化)" if OPTIMIZE_ROUTE_SORTING else "OFF (SHP元の順序を維持)"
yaw_mode = "一括指定" if USE_GLOBAL_YAW else "個別リスト指定"

print(f"[GIS処理] 機首方位(Yaw): {yaw_mode}")
print(f"[GIS処理] WPソート: {sort_status}")

# Step 1: ウェイポイント処理と高度計算
processed_waypoints, exec_h_mode, coord_h_mode = process_waypoints(
    SHP_PATH, 
    DEM_PATH, 
    TARGET_AGL, 
    OPTIMIZE_ROUTE_SORTING, 
    USE_GLOBAL_YAW, 
    GLOBAL_YAW_DEG, 
    INDIVIDUAL_YAW_LIST
)

# Step 2: XML構築
print(f"[XML生成] KML/WPML構造を構築中...")
print(f"  └ 画像フォーマット: {IMAGE_FORMAT_STR}")
print(f"  └ RTH設定 : {RC_LOST_ACTION}")
template_tree, waylines_tree = generate_dji_xml(processed_waypoints, exec_h_mode, coord_h_mode)

# Step 3: KMZファイルのエクスポート
print("[出力処理] KMZファイルをパッケージング中...")
export_kmz(template_tree, waylines_tree, OUTPUT_KMZ)

print("--- 処理がすべて完了しました ---")