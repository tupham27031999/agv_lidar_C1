import numpy as np
import cv2
import math
import path
from support_main.lib_main import edit_csv_tab
import os
path_phan_mem = path.path_phan_mem

# Directories for saving data and maps
SAVED_DATA_DIR = path_phan_mem + "/data_input_output"
PATH_MAPS_DIR = SAVED_DATA_DIR + "/maps"

distan_scan_all = [0, 10000]
path_admin = path_phan_mem + "/setting/admin_window.csv"
if os.name == "nt":
    print("Hệ điều hành là Windows")
    # Đọc file cài đặt cho Windows
    path_admin = path_phan_mem + "/setting/admin_window.csv"
    on_music = 1
elif os.name == "posix":
    print("Hệ điều hành là Ubuntu (Linux)")
    # Đọc file cài đặt cho Ubuntu
    path_admin = path_phan_mem + "/setting/admin_ubuntu.csv"
    on_music = 0
data_admin = edit_csv_tab.load_all_stt(path_admin)
x_goc0 = "none"
y_goc0 = "none"
rotation0 = "none"
kc_them_map = 170
for i in range(0,len(data_admin)):
    if len(data_admin[i]) > 1:
        if data_admin[i][0] == "anpha_scan":
            anpha_scan = [int(float(data_admin[i][1])), int(float(data_admin[i][2]))] 


def transform_lidar_points(scan_data_lidar, lidar_pos_on_agv, lidar_orientation_on_agv_deg, angular_range_deg, scaling_factor):
    """
    Chuyển đổi điểm từ hệ tọa độ cục bộ của Lidar sang hệ tọa độ của AGV.

    Args:
        scan_data_lidar (tuple): Tuple chứa (signals, angles_deg, distances).
                                 signals: list/array các cường độ tín hiệu.
                                 angles_deg: list/array các góc quét (độ), trong hệ tọa độ Lidar (0-360).
                                 distances: list/array các khoảng cách tương ứng.
        lidar_pos_on_agv (tuple): Vị trí (x, y) của Lidar trên AGV so với tâm AGV.
        lidar_orientation_on_agv_deg (float): Góc xoay của Lidar (độ) so với hướng 0 độ của AGV.
                                              (Dương là ngược chiều kim đồng hồ).
        angular_range_deg (tuple): (alpha1, alpha2) - Dải góc (độ) hoạt động của Lidar.
                                   alpha1, alpha2 có thể nằm trong khoảng (-360, 360) và sẽ được chuẩn hóa về 0-360.

    Returns:
        np.ndarray: Mảng các điểm (x, y) trong hệ tọa độ AGV, và signals. Shape (N, 3).
        lidar0_pos = (-agv_w / 2, agv_l / 2 )
        lidar0_orient_deg = 90 + 45 + 180
        lidar0_alpha_range = (-lidar0_orient_deg, 359-lidar0_orient_deg)
    """
    # print(scan_data_lidar)
    # signals, angles_deg, distances = scan_data_lidar
    # scan_data_lidar = np.array(scan_data_lidar)
    # print("scan_data_lidar", scan_data_lidar, scan_data_lidar.shape)
    # print("scan_data_lidar", scan_data_lidar)
    if scan_data_lidar.shape[0] < 1:
        return np.array([[0, 0, 0]])

    signals = scan_data_lidar[:, 0]
    angles_deg = scan_data_lidar[:, 1]
    distances = scan_data_lidar[:, 2] * scaling_factor
    # print("angles_deg", angles_deg)

    # 1. Lọc theo dải góc hoạt động của Lidar và khoảng cách hợp lệ
    alpha1, alpha2 = angular_range_deg
    # print(alpha1, alpha2)

    # if alpha1 <= alpha2:
    #     mask_angle = (angles_deg >= alpha1) & (angles_deg <= alpha2)
    # else:
    mask_angle = (angles_deg <= alpha1) | (angles_deg >= alpha2)
    
    mask_distance = (distances > 0)

    combined_mask = mask_angle & mask_distance
    
    signals_filtered = signals[combined_mask]
    angles_deg_filtered = angles_deg[combined_mask]
    distances_filtered = distances[combined_mask]

    if len(signals_filtered) == 0:
        return np.array([[0, 0, 0]])

    # 2. Chuyển từ cực sang Descartes (local Lidar frame)
    angles_rad_local = np.radians(angles_deg_filtered)
    x_local = distances_filtered * np.cos(angles_rad_local)
    y_local = distances_filtered * np.sin(angles_rad_local)

    # 3. Xây dựng ma trận xoay cho Lidar
    lidar_orientation_rad = np.radians(lidar_orientation_on_agv_deg)
    cos_orient = np.cos(lidar_orientation_rad)
    sin_orient = np.sin(lidar_orientation_rad)

    # Áp dụng phép xoay
    x_rotated = x_local * cos_orient - y_local * sin_orient
    y_rotated = x_local * sin_orient + y_local * cos_orient

    # 4. Tịnh tiến điểm theo vị trí lắp đặt của Lidar (vào hệ AGV)
    x_agv = x_rotated + lidar_pos_on_agv[0]
    y_agv = - y_rotated + lidar_pos_on_agv[1]

    # Ghép các cột x_agv, y_agv, signal lại
    # Lưu ý thứ tự: (x, y, signal)
    transformed_points_agv_frame = np.vstack((x_agv, y_agv, signals_filtered)).T
    
    return transformed_points_agv_frame

def combine_lidar_data(scan1_data, lidar1_pos_on_agv, lidar1_orientation_deg, lidar1_angular_range_deg,
                       scan2_data, lidar2_pos_on_agv, lidar2_orientation_deg, lidar2_angular_range_deg,
                       scaling_factor):
    """
    Gộp dữ liệu từ hai Lidar vào một hệ tọa độ chung của AGV.

    Args:
        scan1_data, scan2_data: Dữ liệu từ Lidar 1 và 2.
        lidar1_pos_on_agv, lidar2_pos_on_agv: Vị trí (x,y) của Lidar 1, 2 trên AGV.
        lidar1_orientation_deg, lidar2_orientation_deg: Góc xoay (độ) của Lidar 1, 2.
        lidar1_angular_range_deg, lidar2_angular_range_deg: Dải góc hoạt động (độ) của Lidar 1, 2.

    Returns:
        np.ndarray: Mảng các điểm (x_agv, y_agv, signal) đã được gộp trong hệ tọa độ AGV.
        np.ndarray: Mảng các điểm (x_agv, y_agv, signal) từ Lidar 1 (để vẽ màu riêng).
        np.ndarray: Mảng các điểm (x_agv, y_agv, signal) từ Lidar 2 (để vẽ màu riêng).
    """
    # print("scan0_data", scan0_data)
    points1_agv_frame = transform_lidar_points(
        scan1_data, lidar1_pos_on_agv, lidar1_orientation_deg, lidar1_angular_range_deg, scaling_factor
    )
    points2_agv_frame = transform_lidar_points(
        scan2_data, lidar2_pos_on_agv, lidar2_orientation_deg, lidar2_angular_range_deg, scaling_factor
    )
    if points1_agv_frame.shape[0] < 1 and points2_agv_frame.shape[0] < 1:
        combined_points = np.empty((0, 3))
    elif points1_agv_frame.shape[0] < 1:
        combined_points = points2_agv_frame
    elif points2_agv_frame.shape[0] < 1:
        combined_points = points1_agv_frame
    else:
        combined_points = np.vstack((points1_agv_frame, points2_agv_frame))
    # print("points1_agv_frame, points2_agv_frame", points1_agv_frame, points2_agv_frame)
    return combined_points, points1_agv_frame, points2_agv_frame


def visualize_combined_lidar(
    points0_agv, points1_agv, points2_agv,
    agv_width, agv_length,
    lidar0_pos_on_agv, lidar1_pos_on_agv, lidar2_pos_on_agv,
    img_size_pixels=800, scale_factor=50
):
    """
    Trực quan hóa dữ liệu Lidar đã gộp trên một hình ảnh.

    Args:
        points1_agv (np.ndarray): Điểm từ Lidar 1 trong hệ tọa độ AGV (N,3).
        points2_agv (np.ndarray): Điểm từ Lidar 2 trong hệ tọa độ AGV (M,3).
        agv_width (float): Chiều rộng của AGV (đơn vị giống Lidar).
        agv_length (float): Chiều dài của AGV (đơn vị giống Lidar).
        lidar1_pos_on_agv (tuple): Vị trí (x,y) của Lidar 1 trên AGV.
        lidar2_pos_on_agv (tuple): Vị trí (x,y) của Lidar 2 trên AGV.
        img_size_pixels (int): Kích thước ảnh vuông (pixel).
        scale_factor (float): Số pixel cho mỗi đơn vị của Lidar/AGV.
    
    Returns:
        np.ndarray: Hình ảnh OpenCV (BGR) thể hiện dữ liệu.
    """
    img = np.full((img_size_pixels, img_size_pixels, 3), (200, 200, 200), dtype=np.uint8)
    center_x_img, center_y_img = img_size_pixels // 2, img_size_pixels // 2

    # Hàm chuyển đổi tọa độ thế giới (AGV) sang pixel ảnh
    def world_to_pixel(wx, wy):
        px = int(center_x_img + wx * scale_factor)
        py = int(center_y_img - wy * scale_factor) # Trục y ảnh ngược với Descartes
        return px, py

    # 1. Vẽ AGV (hình chữ nhật)
    agv_half_w_px = int(agv_width / 2 * scale_factor)
    agv_half_l_px = int(agv_length / 2 * scale_factor)
    pt1_agv = (center_x_img - agv_half_w_px, center_y_img - agv_half_l_px)
    pt2_agv = (center_x_img + agv_half_w_px, center_y_img + agv_half_l_px)
    cv2.rectangle(img, pt1_agv, pt2_agv, (0, 0, 0), 2) # Màu đen

    # Vẽ hướng phía trước của AGV (mũi tên nhỏ)
    arrow_start = (center_x_img, center_y_img)
    arrow_end = (center_x_img, center_y_img + int(agv_length/2.5 * scale_factor)) # Trỏ lên trên (y giảm)
    cv2.arrowedLine(img, arrow_start, arrow_end, (50, 50, 50), 2, tipLength=0.3)


    # 2. Vẽ vị trí Lidar trên AGV
    l0_px, l0_py = world_to_pixel(lidar0_pos_on_agv[0], lidar0_pos_on_agv[1])
    cv2.circle(img, (l0_px, l0_py), 5, (0, 0, 0), -1) # Lidar 1 màu đỏ
    cv2.putText(img, "L0", (l0_px + 7, l0_py + 7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1)

    l1_px, l1_py = world_to_pixel(lidar1_pos_on_agv[0], lidar1_pos_on_agv[1])
    cv2.circle(img, (l1_px, l1_py), 5, (0, 0, 255), -1) # Lidar 1 màu đỏ
    cv2.putText(img, "L1", (l1_px + 7, l1_py + 7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

    l2_px, l2_py = world_to_pixel(lidar2_pos_on_agv[0], lidar2_pos_on_agv[1])
    cv2.circle(img, (l2_px, l2_py), 5, (255, 0, 0), -1) # Lidar 2 màu xanh dương
    cv2.putText(img, "L2", (l2_px + 7, l2_py + 7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)

    # 3. Vẽ các điểm Lidar
    if points0_agv.ndim == 2 and points0_agv.shape[0] > 0:
        for pt_agv in points0_agv:
            px, py = world_to_pixel(pt_agv[0], pt_agv[1])
            if 0 <= px < img_size_pixels and 0 <= py < img_size_pixels:
                cv2.circle(img, (px, py), 2, (0, 0, 0), -1) # Lidar 1 màu tím

    if points1_agv.ndim == 2 and points1_agv.shape[0] > 0:
        for pt_agv in points1_agv:
            px, py = world_to_pixel(pt_agv[0], pt_agv[1])
            if 0 <= px < img_size_pixels and 0 <= py < img_size_pixels:
                cv2.circle(img, (px, py), 2, (255, 0, 255), -1) # Lidar 1 màu tím

    if points2_agv.ndim == 2 and points2_agv.shape[0] > 0:
        for pt_agv in points2_agv:
            px, py = world_to_pixel(pt_agv[0], pt_agv[1])
            if 0 <= px < img_size_pixels and 0 <= py < img_size_pixels:
                cv2.circle(img, (px, py), 2, (0, 255, 0), -1) # Lidar 2 màu xanh lá

    return img
# --- Tải dữ liệu ---
def load_lidar_data(folder_path, file_index):
    file_path = f"{folder_path}/scan_{file_index}.npy" # scan_0
    try:
        data = np.load(file_path)
        if data[0].shape != 0:
            # Giả sử định dạng là: hàng 0: signals, hàng 1: angles, hàng 2: distances
            signals = data[:, 0]
            angles_deg = data[:, 1]
            distances = data[:, 2]
            # print(angles_deg)
            return signals, angles_deg, distances
        else:
            print(f"Lỗi: Dữ liệu không đúng định dạng trong file {file_path}")
            return [], [], []
    except FileNotFoundError:
        # Đây là điều kiện dừng vòng lặp, không phải lỗi
        print(f"Thông báo: Đã xử lý hết file hoặc không tìm thấy {file_path}. Dừng lại.")
        return None, None, None
    except Exception as e:
        print(f"Lỗi khi tải file {file_path}: {e}")
        return None, None, None

# --- Tải dữ liệu ---
def load_lidar_data2(folder_path, file_index):
    file_path = f"{folder_path}/scan_{file_index}.npy" # scan_0
    try:
        data = np.load(file_path)
        if data[0].shape != 0:
            # Giả sử định dạng là: hàng 0: signals, hàng 1: angles, hàng 2: distances
            signals = data[:, 0]
            angles_deg = data[:, 1]
            distances = data[:, 2]
            # print(angles_deg)
            return signals, angles_deg, distances
        else:
            print(f"Lỗi: Dữ liệu không đúng định dạng trong file {file_path}")
            return [], [], []
    except FileNotFoundError:
        # Đây là điều kiện dừng vòng lặp, không phải lỗi
        print(f"Thông báo: Đã xử lý hết file hoặc không tìm thấy {file_path}. Dừng lại.")
        return None, None, None
    except Exception as e:
        print(f"Lỗi khi tải file {file_path}: {e}")
        return None, None, None
def convert_scan_lidar(scan1_data_example=np.array([[0, 0, 0]]), 
                       scan2_data_example=np.array([[0, 0, 0]]), 
                       scaling_factor = 0.05,
                       lidar1_orient_deg = -45,
                       lidar2_orient_deg = 126,
                       agv_w=-16,
                       agv_l=-27):

    # Lidar 1: Vị trí và hướng (tùy chỉnh theo thực tế)
    lidar1_pos = (-agv_w / 2, agv_l / 2)
    lidar1_alpha_range = (anpha_scan[0], anpha_scan[1])

    # Lidar 2: Vị trí và hướng (tùy chỉnh theo thực tế)
    lidar2_pos = (agv_w / 2, -agv_l / 2)
    lidar2_alpha_range = (anpha_scan[0], anpha_scan[1])

    # print("scan0_data_example,",scan0_data_example)
    p_all_agv, p1_agv, p2_agv = combine_lidar_data(
        scan1_data_example, lidar1_pos, lidar1_orient_deg, lidar1_alpha_range,
        scan2_data_example, lidar2_pos, lidar2_orient_deg, lidar2_alpha_range,
        scaling_factor = scaling_factor
    )
    return p_all_agv, p1_agv, p2_agv
    # print(p1_agv)

    # vis_img = visualize_combined_lidar(
    #     p0_agv, p1_agv, p2_agv, agv_w, agv_l, lidar0_pos, lidar1_pos, lidar2_pos,
    #     img_size_pixels=800, scale_factor=scale_factor # Tăng kích thước và scale để dễ nhìn hơn
    # )
    # cv2.putText(vis_img, f"File: scan_{file_index}.npy", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)

    # return check, vis_img


# --- Ví dụ sử dụng ---
if __name__ == '__main__':
    # --- Cấu hình ---
    # Đường dẫn tới thư mục chứa dữ liệu scan
    folder_path_lidar0 = "C:/tupn/phan_mem/a_agv/agv_sent_web/data_input_output/scan_data_1"
    folder_path_lidar1 = "C:/tupn/phan_mem/a_agv/agv_sent_web/data_input_output/scan_data_1"
    folder_path_lidar2 = "C:/tupn/phan_mem/a_agv/agv_sent_web/data_input_output/scan_data_2"

    scale_factor = 0.05
    # Thông số AGV và Lidar (đơn vị mét) - Cần khớp với dữ liệu
    agv_w = -300
    agv_l = -800
    
    # # Lidar 0: Vị trí và hướng (tùy chỉnh theo thực tế)
    # lidar0_pos = (0, 0)
    # lidar0_orient_deg = 90
    # lidar0_alpha_range = (0, 359)

    # Lidar 0: Vị trí và hướng (tùy chỉnh theo thực tế)
    lidar0_pos = (-agv_w / 2, agv_l / 2 )
    lidar0_orient_deg = 90 + 45 + 180
    lidar0_alpha_range = (-lidar0_orient_deg, 359-lidar0_orient_deg)

    # Lidar 1: Vị trí và hướng (tùy chỉnh theo thực tế)
    lidar1_pos = (-agv_w / 2, agv_l / 2)
    lidar1_orient_deg = 90 + 45 + 180
    lidar1_alpha_range = (-lidar1_orient_deg, 359-lidar1_orient_deg)

    # Lidar 2: Vị trí và hướng (tùy chỉnh theo thực tế)
    lidar2_pos = (agv_w / 2, -agv_l / 2)
    lidar2_orient_deg = 90 + 42
    lidar2_alpha_range = (-lidar2_orient_deg, 359-lidar2_orient_deg)

    

    # --- Vòng lặp xử lý ---
    file_index = 0
    while True:
        check, vis_img = convert_scan_lidar(file_index)
        if check == False:
            break
        cv2.imshow("Combined Lidar Data", vis_img)

        if cv2.waitKey(100) & 0xFF == ord('q'): # Đợi 100ms, nhấn 'q' để thoát
            break

        file_index += 1 # Chuyển sang file tiếp theo

    cv2.destroyAllWindows()
