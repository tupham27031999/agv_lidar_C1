from flask import Flask, render_template, url_for, Response, request, jsonify
import numpy as np
import cv2
import math
import threading
import json
import support_app_web
import path




path_phan_mem = path.path_phan_mem

# Directories for saving data and maps
SAVED_DATA_DIR = path_phan_mem + "/data_input_output"
PATH_MAPS_DIR = SAVED_DATA_DIR + "/maps"
PATH_POINTS_DIR = SAVED_DATA_DIR + "/point_lists"
PATH_PATHS_DIR = SAVED_DATA_DIR + "/path_lists"
PATH_GRID_LISTS_DIR = SAVED_DATA_DIR + "/grid_lists"
PATH_LOG_GIAO_TIEP_DIR = SAVED_DATA_DIR + "/log_giao_tiep"

height = 2000
width = 2000

image_all = np.ones((height, width, 3), dtype=np.uint8) * 100
image_all_web = image_all.copy()

# khai báo
list_van_toc = [0.1, 0.2, 0.3, 0.4, 0.5]
list_van_toc_re = [5, 10, 15, 20]
danh_sach_ban_do = support_app_web.get_available_maps(PATH_MAPS_DIR)
danh_sach_khu_vuc = ["Khu vực 1", "Khu vực 2", "Khu vực 3", "Khu vực 4", "Khu vực 5"]

# output web
dict_cai_dat = {
    "update": 0,
    "tien_max": 0.2,  # Tốc độ tối đa tiến
    "re_max": 10,  # Tốc độ tối đa rẽ
    }
# AGV position adjustment
dict_dieu_chinh_vi_tri_agv = {"toa_do_x": 1000, "toa_do_y": 1000, "goc_agv": 0, "setup": 0, "update": 0}
points_color_blue = np.array([[200, 200], [300, 300], [400, 400]])  # Example blue points
points_color_red = np.array([])  # Example red points
# Lock để bảo vệ truy cập vào các biến chia sẻ với luồng main và process_lidar
shared_data_lock = threading.Lock()

# điều khiển quét bản đồ mới hoặc cập nhật bản đồ
dict_ban_do_moi = {
    "ten_ban_do_cu": "",
    "tai_ban_do_cu": 0,
    "cap_nhat_ban_do_cu":0, # nếu == 1 thì bản đồ hiện tại sẽ cập nhật thêm

    "tao_ban_do_moi": 0,
    "ten_ban_do_moi": "",
    "luu_ban_do_moi": 0,

    "dieu_khien_thu_cong": 0, # chỉ bằng 1 khi nhấn nút (điều khiển thủ công và (cap_nhat_ban_do_cu == 1 hoặc tao_ban_do_moi == 1)
    "tien": 0, # bằng 1 khi nhấn nút mũi tên lên
    "lui": 0, # bằng 1 khi nhấn nút mũi tên lùi
    "re_trai": 0,
    "re_phai": 0
}

# khung lựa chọn vị trí
dict_lua_chon_vi_tri = {
    "vi_tri_agv_hien_tai": {"toa_do_x": 0, "toa_do_y": 0, "goc_agv": 0, "update": 0, "color": "red"},
    "vi_tri_bat_dau_lam_viec": {"toa_do_x": 0, "toa_do_y": 0, "goc_agv": 0, "update": 0, "color": "yellow"},
    "vi_tri_ket_thuc_lam_viec": {"toa_do_x": 0, "toa_do_y": 0, "goc_agv": 0, "update": 0, "color": "green"},
    "vi_tri_sac_agv": {"toa_do_x": 0, "toa_do_y": 0, "goc_agv": 0, "update": 0, "color": "hotpink"}
}

# khu vực làm việc
khu_vuc_lam_viec = {
    "cac_khu_vuc_lam_viec": [],
    "danh_sach_khu_vuc_mo": ["loai_1", "loai_2"],
    "ten_khu_vuc_mo": "",
    "tai_khu_vuc_mo": 0,
    "ten_khu_vuc_luu": "",
    "tai_khu_vuc_luu": 0
}

# Khởi tạo ứng dụng Flask
app = Flask(__name__)

# Định nghĩa route cho trang chủ
@app.route("/")
def home():
    return render_template(
        "index.html",
        title="Flask App",
        list_van_toc=list_van_toc,
        list_van_toc_re=list_van_toc_re,
        dict_cai_dat=dict_cai_dat,
        danh_sach_ban_do=danh_sach_ban_do,
        danh_sach_khu_vuc=danh_sach_khu_vuc,
        dict_lua_chon_vi_tri=dict_lua_chon_vi_tri,
        khu_vuc_lam_viec=khu_vuc_lam_viec  # Thêm dòng này để truyền biến khu_vuc_lam_viec
    )


# Định nghĩa route để hiển thị ảnh
@app.route('/img_none_all')
def img_none_all():
    """
    An toàn đọc và sao chép bản đồ để gửi cho client.
    Thao tác này đảm bảo rằng chúng ta không gửi một hình ảnh đang được ghi dở dang.
    """
    map_to_send = None
    with shared_data_lock:
        # Chỉ sao chép (copy) khi cần gửi đi, và thực hiện trong lock
        # để đảm bảo dữ liệu nhất quán.
        if image_all is not None:
            map_to_send = image_all.copy()

    if map_to_send is None:
        return "Bản đồ chưa sẵn sàng", 404

    # Mã hóa và gửi ảnh đi (thực hiện ngoài lock để không chặn các luồng khác)
    success, encoded_image = cv2.imencode('.jpg', map_to_send)
    if not success:
        return "Lỗi mã hóa ảnh", 500
    return Response(encoded_image.tobytes(), mimetype='image/jpeg')


# Route để cập nhật cài đặt từ web
@app.route("/update_setting", methods=['POST'])
def update_setting():
    global dict_cai_dat
    data = request.json
    
    # Lấy các giá trị từ payload JSON
    tien_max_new = data.get('tien_max')
    re_max_new = data.get('re_max')
    update_flag = data.get('update')

    # Cập nhật giá trị nếu chúng tồn tại trong payload
    if tien_max_new is not None:
        dict_cai_dat['tien_max'] = tien_max_new
        print(f"Giá trị 'tien_max' đã được cập nhật thành: {tien_max_new}")

    if re_max_new is not None:
        dict_cai_dat['re_max'] = re_max_new
        print(f"Giá trị 're_max' đã được cập nhật thành: {re_max_new}")
    
    # Cập nhật cờ nếu nó tồn tại
    if update_flag is not None:
        dict_cai_dat['update'] = update_flag
        print("dict_cai_dat[\"update\"] đã được đặt thành 1")

    print("dict_cai_dat: ", dict_cai_dat)

    return jsonify({"status": "success", "message": "Settings updated successfully."})
    
# # Route để xử lý yêu cầu tải lên bản đồ mới
# @app.route('/upload_map', methods=['POST'])
# def upload_map():
#     """
#     Nhận yêu cầu từ client để cập nhật image_all dựa trên bản đồ được chọn.
#     """
#     global image_all, image_lock
#     data = request.get_json()
#     map_name = data.get('map_name')

#     if not map_name:
#         return jsonify({"status": "error", "message": "Tên bản đồ không được cung cấp"}), 400

#     with image_lock:
#         # --- LOGIC CẬP NHẬT BẢN ĐỒ THỰC TẾ ---
#         # Đây là nơi bạn sẽ đặt logic tải bản đồ mới từ file, ví dụ:
#         # image_all = cv2.imread(f"path/to/maps/{map_name}.png")
#         #
#         # Để minh họa rằng việc cập nhật đã hoạt động, tôi sẽ tạo một ảnh mới
#         # và vẽ tên bản đồ đã chọn lên đó.
#         # Bạn hãy thay thế phần minh họa này bằng logic tải file ảnh thực tế của bạn.
#         new_image = np.ones((height, width, 3), dtype=np.uint8) * 240 # Tạo nền màu trắng xám
#         cv2.putText(new_image,
#                     f"Da tai: {map_name}",
#                     (50, 150), # Vị trí text
#                     cv2.FONT_HERSHEY_SIMPLEX,
#                     4, (0, 100, 0), 5, cv2.LINE_AA) # Màu, kích thước, độ dày
#         image_all = new_image.copy() # Gán ảnh mới vào biến toàn cục image_all

#     # Trả về thông báo thành công
#     return jsonify({"status": "success", "message": f"Đã cập nhật bản đồ thành {map_name}"})

# Function to generate points for an arrow overlay
def generate_arrow_points(center_x, center_y, angle_deg, color):
    # Same logic as AGV arrow, but perhaps simpler
    # For demonstration, let's just use a single point for now
    # and let the JS handle the arrow overlay
    return [{"x": center_x, "y": center_y, "color": color, "angle": angle_deg}]
# Endpoint để client lấy thông tin vị trí AGV
@app.route('/get_agv_state', methods=['GET'])
def get_agv_state_route():
    global dict_dieu_chinh_vi_tri_agv, points_color_blue, points_color_red
    agv_body_coords_list = []
    agv_arrow_coords_list = []
    blue_points_list = []
    red_points_list = []
    with shared_data_lock: # Đảm bảo an toàn luồng khi đọc các biến này
        center_x_map = float(dict_dieu_chinh_vi_tri_agv['toa_do_x'])
        center_y_map = float(dict_dieu_chinh_vi_tri_agv['toa_do_y'])
        angle_deg = - float(dict_dieu_chinh_vi_tri_agv['goc_agv'])
        angle_rad = math.radians(angle_deg)

        # AGV Body (Rectangle) - Define local coords then rotate and translate
        rect_width_local = 30  # pixels
        rect_height_local = 20 # pixels
        
        hw = rect_width_local / 2 # half-width
        hh = rect_height_local / 2 # half-height
        local_body_corners_np  = np.array([
            [-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]
        ], dtype=np.float32) # Use float32 for cv2.pointPolygonTest


        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        rotation_m = np.array([[cos_a, -sin_a], [sin_a, cos_a]])

        rotated_local_body_corners = np.dot(local_body_corners_np, rotation_m.T)
        final_body_corners_map_space = rotated_local_body_corners + np.array([center_x_map, center_y_map])

        # Rasterize the AGV body rectangle
        min_x_body = int(np.floor(np.min(final_body_corners_map_space[:, 0])))
        max_x_body = int(np.ceil(np.max(final_body_corners_map_space[:, 0])))
        min_y_body = int(np.floor(np.min(final_body_corners_map_space[:, 1])))
        max_y_body = int(np.ceil(np.max(final_body_corners_map_space[:, 1])))

        # The contour for pointPolygonTest needs to be of type int32 or float32.
        # final_body_corners_map_space is already float32 due to local_body_corners_np.
        contour_body_for_test = final_body_corners_map_space.astype(np.float32) 

        temp_body_coords_list = []
        for x_coord_test in range(min_x_body, max_x_body + 1):
            for y_coord_test in range(min_y_body, max_y_body + 1):
                if cv2.pointPolygonTest(contour_body_for_test, (float(x_coord_test), float(y_coord_test)), False) >= 0:
                    temp_body_coords_list.append([x_coord_test, y_coord_test])
        agv_body_coords_list = temp_body_coords_list

        # AGV Arrow (Triangle) - Define local coords then rotate and translate
        arrow_shaft_len_local = 25 
        arrow_head_len_local = 10
        arrow_head_width_local = 12

        p_tip_local = np.array([arrow_shaft_len_local + arrow_head_len_local, 0])
        p_base1_local = np.array([arrow_shaft_len_local, -arrow_head_width_local / 2])
        p_base2_local = np.array([arrow_shaft_len_local, arrow_head_width_local / 2])
        local_arrow_vertices_np = np.array([p_tip_local, p_base1_local, p_base2_local], dtype=np.float32)

        rotated_local_arrow_vertices = np.dot(local_arrow_vertices_np, rotation_m.T)
        final_arrow_vertices_map_space = rotated_local_arrow_vertices + np.array([center_x_map, center_y_map])

        # Rasterize the AGV arrow triangle
        min_x_arrow = int(np.floor(np.min(final_arrow_vertices_map_space[:, 0])))
        max_x_arrow = int(np.ceil(np.max(final_arrow_vertices_map_space[:, 0])))
        min_y_arrow = int(np.floor(np.min(final_arrow_vertices_map_space[:, 1])))
        max_y_arrow = int(np.ceil(np.max(final_arrow_vertices_map_space[:, 1])))

        contour_arrow_for_test = final_arrow_vertices_map_space.astype(np.float32)

        temp_arrow_coords_list = []
        for x_coord_test in range(min_x_arrow, max_x_arrow + 1):
            for y_coord_test in range(min_y_arrow, max_y_arrow + 1):
                if cv2.pointPolygonTest(contour_arrow_for_test, (float(x_coord_test), float(y_coord_test)), False) >= 0:
                    temp_arrow_coords_list.append([x_coord_test, y_coord_test])
        agv_arrow_coords_list = temp_arrow_coords_list

        # Chuyển đổi NumPy arrays thành list để có thể serialize JSON
        if isinstance(points_color_blue, np.ndarray):
            blue_points_list = points_color_blue.tolist()
        else: # Nếu là list, đảm bảo nó là list các list (cho JSON)
            blue_points_list = [list(p) for p in points_color_blue] if points_color_blue else []

        if isinstance(points_color_red, np.ndarray):
            red_points_list = points_color_red.tolist()
        else: # Nếu là list, đảm bảo nó là list các list
            red_points_list = [list(p) for p in points_color_red] if points_color_red else []

    special_points_data = []
    # Add AGV current position if updated
    if dict_lua_chon_vi_tri['vi_tri_agv_hien_tai']['update'] == 1:
        x = dict_lua_chon_vi_tri['vi_tri_agv_hien_tai']['toa_do_x']
        y = dict_lua_chon_vi_tri['vi_tri_agv_hien_tai']['toa_do_y']
        angle = dict_lua_chon_vi_tri['vi_tri_agv_hien_tai']['goc_agv']
        color = dict_lua_chon_vi_tri['vi_tri_agv_hien_tai']['color']
        special_points_data.append({"x": x, "y": y, "angle": angle, "type": "agv_current", "color": color})

    # Add other special points if they exist
    if dict_lua_chon_vi_tri['vi_tri_bat_dau_lam_viec']['update'] == 1:
        x = dict_lua_chon_vi_tri['vi_tri_bat_dau_lam_viec']['toa_do_x']
        y = dict_lua_chon_vi_tri['vi_tri_bat_dau_lam_viec']['toa_do_y']
        angle = dict_lua_chon_vi_tri['vi_tri_bat_dau_lam_viec']['goc_agv']
        color = dict_lua_chon_vi_tri['vi_tri_bat_dau_lam_viec']['color']
        special_points_data.append({"x": x, "y": y, "angle": angle, "type": "work_start", "color": color})

    # ... (tương tự cho vi_tri_ket_thuc_lam_viec và vi_tri_sac_agv) ...
    if dict_lua_chon_vi_tri['vi_tri_ket_thuc_lam_viec']['update'] == 1:
        x = dict_lua_chon_vi_tri['vi_tri_ket_thuc_lam_viec']['toa_do_x']
        y = dict_lua_chon_vi_tri['vi_tri_ket_thuc_lam_viec']['toa_do_y']
        angle = dict_lua_chon_vi_tri['vi_tri_ket_thuc_lam_viec']['goc_agv']
        color = dict_lua_chon_vi_tri['vi_tri_ket_thuc_lam_viec']['color']
        special_points_data.append({"x": x, "y": y, "angle": angle, "type": "work_end", "color": color})

    if dict_lua_chon_vi_tri['vi_tri_sac_agv']['update'] == 1:
        x = dict_lua_chon_vi_tri['vi_tri_sac_agv']['toa_do_x']
        y = dict_lua_chon_vi_tri['vi_tri_sac_agv']['toa_do_y']
        angle = dict_lua_chon_vi_tri['vi_tri_sac_agv']['goc_agv']
        color = dict_lua_chon_vi_tri['vi_tri_sac_agv']['color']
        special_points_data.append({"x": x, "y": y, "angle": angle, "type": "charge", "color": color})

    return jsonify({
        "status": "success",
        "agv_body_coords": agv_body_coords_list,
        "agv_arrow_coords": agv_arrow_coords_list,
        "points_blue": blue_points_list,
        "points_red": red_points_list,
        "special_points": special_points_data # NEW
    }), 200
@app.route('/update_map_status', methods=['POST'])
def update_map_status():
    global dict_ban_do_moi, danh_sach_ban_do
    data = request.json
    action = data.get('action')
    status = data.get('status')
    map_name = data.get('map_name')

    print(f"Nhận yêu cầu: {action}, Trạng thái: {status}, Tên bản đồ: {map_name}")

    if action == 'load_old_map':
        dict_ban_do_moi["ten_ban_do_cu"] = map_name
        dict_ban_do_moi["tai_ban_do_cu"] = 1
        print("load_old_map", dict_ban_do_moi)
        return jsonify({"status": "success", "message": "Đã đặt cờ tải bản đồ cũ"})

    elif action == 'update_old_map':
        dict_ban_do_moi["cap_nhat_ban_do_cu"] = status
        print("update_old_map", dict_ban_do_moi)
        return jsonify({"status": "success", "message": "Đã cập nhật cờ cập nhật bản đồ cũ"})

    elif action == 'create_new_map':
        dict_ban_do_moi["tao_ban_do_moi"] = status
        print("create_new_map", dict_ban_do_moi)
        return jsonify({"status": "success", "message": "Đã đặt cờ tạo bản đồ mới"})

    elif action == 'update_new_map_name':
        dict_ban_do_moi["ten_ban_do_moi"] = map_name
        print("update_new_map_name", dict_ban_do_moi)
        return jsonify({"status": "success", "message": "Đã cập nhật tên bản đồ mới"})

    elif action == 'save_new_map':
        if dict_ban_do_moi["ten_ban_do_moi"] != "":
            print("save_new_map", dict_ban_do_moi)
            danh_sach_ban_do = support_app_web.get_available_maps(PATH_MAPS_DIR)
            if dict_ban_do_moi["ten_ban_do_moi"] not in danh_sach_ban_do:
                danh_sach_ban_do.append(dict_ban_do_moi["ten_ban_do_moi"])
            with shared_data_lock:
                dict_ban_do_moi["luu_ban_do_moi"] = 1
            return jsonify({"status": "success", "message": "Đã đặt cờ lưu bản đồ mới", "map_list": danh_sach_ban_do})
        return jsonify({"status": "error", "message": "Tên bản đồ mới trống"})

    elif action == 'manual_control':
        dict_ban_do_moi["dieu_khien_thu_cong"] = status
        print("manual_control", dict_ban_do_moi)
        return jsonify({"status": "success", "message": "Đã cập nhật cờ điều khiển thủ công"})
    
    # Xử lý các nút mũi tên
    elif action in ["forward", "backward", "turn_left", "turn_right"]:
        if action == "forward":
            dict_ban_do_moi["tien"] = status
        elif action == "backward":
            dict_ban_do_moi["lui"] = status
        elif action == "turn_left":
            dict_ban_do_moi["trai"] = status
        elif action == "turn_right":
            dict_ban_do_moi["phai"] = status
        print("forward, backward, turn_lef, tturn_right", dict_ban_do_moi)
        return jsonify({"status": "success", "message": f"Đã cập nhật trạng thái phím '{action}'"})
    

    return jsonify({"status": "error", "message": "Hành động không xác định"}), 400

# # Route để cập nhật trạng thái bản đồ, lưu bản đồ mới và trả về danh sách mới nhất
# @app.route("/update_map_status", methods=['POST'])
# def update_map_status():
#     global danh_sach_ban_do, PATH_MAPS_DIR
#     data = request.json
#     action = data.get('action')
#     if action == "save_new_map":
#         # Thực hiện logic lưu bản đồ mới tại đây
#         print("Đã nhận yêu cầu lưu bản đồ mới.")
#         # Ví dụ: support_app_web.save_new_map(new_map_data)
        
#     # Sau khi xử lý xong, luôn cập nhật và trả về danh sách bản đồ mới nhất
#     danh_sach_ban_do = support_app_web.get_available_maps(PATH_MAPS_DIR)
#     print("Danh sách bản đồ đã được cập nhật:", danh_sach_ban_do)
#     return jsonify({"danh_sach_ban_do": danh_sach_ban_do})
###################################################################################################

@app.route('/update_location', methods=['POST'])
def update_location():
    """
    Receives location data (coordinates, angle) from the client and updates
    the global dictionary.
    """
    global dict_lua_chon_vi_tri
    data = request.json
    location_type = data.get('location_type')
    coords = data.get('coords')
    angle = data.get('angle')
    update_flag = data.get('update')

    print(f"Received data for {location_type}: {data}")

    if location_type not in dict_lua_chon_vi_tri:
        return jsonify({"status": "error", "message": "Invalid location type"}), 400
    
    if coords:
        dict_lua_chon_vi_tri[location_type]["toa_do_x"] = coords['x']
        dict_lua_chon_vi_tri[location_type]["toa_do_y"] = coords['y']
        print(f"Updated {location_type} coordinates: {coords['x']}, {coords['y']}")
    
    if angle is not None:
        dict_lua_chon_vi_tri[location_type]["goc_agv"] = angle
        print(f"Updated {location_type} angle: {angle}")

    if (update_flag is not None) and update_flag != 0:
        dict_lua_chon_vi_tri[location_type]["update"] = update_flag

    print(f"dict_lua_chon_vi_tri: {dict_lua_chon_vi_tri}")
    
    return jsonify({"status": "success", "message": f"Updated {location_type} location."})
@app.route('/get_all_locations', methods=['GET'])
def get_all_locations():
    """
    Returns the global dictionary of special locations to the client.
    """
    global dict_lua_chon_vi_tri
    return jsonify({'dict_lua_chon_vi_tri': dict_lua_chon_vi_tri})



# # Khai báo biến toàn cục ở đầu file (nếu chưa có)
# khu_vuc_lam_viec = {'cac_khu_vuc_lam_viec': []}

@app.route('/get_working_areas')
def get_working_areas():
    """
    Trả về danh sách các khu vực làm việc đã lưu dưới dạng JSON.
    """
    try:
        global khu_vuc_lam_viec
        return jsonify(khu_vuc_lam_viec)
    except Exception as e:
        print(f"Error processing get_working_areas: {e}")
        return jsonify({"status": "error", "message": "Internal Server Error"}), 500

@app.route('/update_working_area', methods=['POST'])
def update_working_area():
    """
    Nhận dữ liệu JSON từ client và cập nhật danh sách khu vực làm việc toàn cục.
    """
    global khu_vuc_lam_viec
    try:
        data = request.json
        if not data or 'action' not in data:
            return jsonify({"status": "error", "message": "Invalid request format"}), 400

        action = data.get('action')
        print("data", data)

        if action == 'add' or action == 'resize':
            khu_vuc = data.get('area')
            if not khu_vuc or 'name' not in khu_vuc or 'coords' not in khu_vuc:
                return jsonify({"status": "error", "message": "Invalid area data. Missing 'name' or 'coords'"}), 400
            
            coords = khu_vuc.get('coords')
            if not coords or 'x' not in coords or 'y' not in coords or 'width' not in coords or 'height' not in coords:
                return jsonify({"status": "error", "message": "Invalid coordinates data. Missing x, y, width, or height"}), 400

            # Tìm và cập nhật khu vực nếu đã tồn tại, nếu không thì thêm mới
            found = False
            for i, existing_area in enumerate(khu_vuc_lam_viec['cac_khu_vuc_lam_viec']):
                if existing_area['name'] == khu_vuc['name']:
                    khu_vuc_lam_viec['cac_khu_vuc_lam_viec'][i] = khu_vuc
                    found = True
                    break
            if not found:
                khu_vuc_lam_viec['cac_khu_vuc_lam_viec'].append(khu_vuc)
            print(f"Updated working areas: {khu_vuc_lam_viec}")
            return jsonify({"status": "success", "message": "Working area added/resized successfully.", "areas": khu_vuc_lam_viec['cac_khu_vuc_lam_viec']})

        elif action == 'delete':
            area_name = data.get('area_name')
            if not area_name:
                return jsonify({"status": "error", "message": "Area name not provided for deletion"}), 400

            khu_vuc_lam_viec['cac_khu_vuc_lam_viec'] = [
                area for area in khu_vuc_lam_viec['cac_khu_vuc_lam_viec']
                if area.get('name') != area_name
            ]
            return jsonify({"status": "success", "message": "Working area deleted successfully."})

        else:
            return jsonify({"status": "error", "message": "Invalid action specified"}), 400
    except Exception as e:
        print(f"Error processing update_working_area: {e}")
        return jsonify({"status": "error", "message": "Internal Server Error"}), 500
    
@app.route('/open_working_area', methods=['POST'])
def open_working_area():
    global khu_vuc_lam_viec
    data = request.json
    area_list_name = data.get('name')
    if area_list_name:
        khu_vuc_lam_viec["ten_khu_vuc_mo"] = area_list_name
        khu_vuc_lam_viec["tai_khu_vuc_mo"] = 1
        print(f"Opening working area list: {area_list_name}")
        print(f"Updated khu_vuc_lam_viec: {khu_vuc_lam_viec}")
        return jsonify({"status": "success", "message": f"Opening area list '{area_list_name}'."})
    return jsonify({"status": "error", "message": "Area list name not provided."}), 400

@app.route('/save_working_area', methods=['POST'])
def save_working_area():
    global khu_vuc_lam_viec
    data = request.json
    area_list_name = data.get('name')
    if area_list_name:
        khu_vuc_lam_viec["ten_khu_vuc_luu"] = area_list_name
        khu_vuc_lam_viec["tai_khu_vuc_luu"] = 1
        print(f"Saving working area list as: {area_list_name}")
        print(f"Updated khu_vuc_lam_viec: {khu_vuc_lam_viec}")
        return jsonify({"status": "success", "message": f"Saving working area list as '{area_list_name}'."})
    return jsonify({"status": "error", "message": "Area list name not provided."}), 400
# Chạy ứng dụng
if __name__ == '__main__':
    app.run(debug=True)