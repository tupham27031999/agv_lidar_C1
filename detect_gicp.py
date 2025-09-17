import open3d as o3d
import numpy as np
import cv2
import time
import math
import os





def load_and_prepare_scan(filepath):
    if not os.path.exists(filepath):
        print(f"Không tìm thấy tệp '{filepath}'")
        return None
    
    try:
        scan_data = np.load(filepath)
        if scan_data.ndim != 2 or scan_data.shape[1] not in [2, 3]:
            print(f"Định dạng không hợp lệ trong '{filepath}'. Shape: {scan_data.shape}")
            return None
        
        scan_data = np.asarray(scan_data, dtype=np.float64)
        
        if scan_data.shape[1] == 3:
            points_3d = polar_to_cartesian_3d(scan_data)
        else:
            print(f"Phát hiện định dạng cartesian 2D trong '{filepath}'. Đang thêm trục Z...")
            z_column = np.zeros((scan_data.shape[0], 1))
            points_3d = np.hstack((scan_data, z_column))
        
        return points_3d
    except Exception as e:
        print(f"Lỗi khi tải tệp '{filepath}': {str(e)}")
        return None

# --- PHẦN 2: CÁC HÀM SLAM ---

def polar_to_cartesian_3d(scan_data):
    if scan_data is None or len(scan_data) == 0:
        return np.array([])
    points_cartesian = []
    for point in scan_data:
        quality, angle, distance = point
        #is_in_front_arc = (angle <= 135) or (angle >= 225)
        if distance > 1000 and distance < 7000 and quality > 10 :#and is_in_front_arc:
            angle_rad = math.radians(angle)
            x = distance * math.cos(angle_rad)
            y = -distance * math.sin(angle_rad)
            points_cartesian.append([x, y, 0.0])
    return np.array(points_cartesian)

def lidar_to_point_cloud(points):
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    return point_cloud

def downsample_point_cloud(point_cloud, voxel_size):
    return point_cloud.voxel_down_sample(voxel_size)

def filter_outliers(point_cloud, nb_neighbors=10, std_ratio=2.5):
    pcd, _ = point_cloud.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    return pcd

def remove_duplicate_points(points, voxel_size=20.0):
    pcd = lidar_to_point_cloud(points)
    pcd_down = downsample_point_cloud(pcd, voxel_size)
    return np.asarray(pcd_down.points)

def remove_dynamic_points(current_points, prev_points, distance_threshold=250.0):
    if prev_points is None or len(prev_points) == 0:
        return current_points 
    pcd_current = lidar_to_point_cloud(current_points)
    pcd_prev = lidar_to_point_cloud(prev_points)
    distances = pcd_current.compute_point_cloud_distance(pcd_prev)
    distances = np.asarray(distances)
    static_indices = np.where(distances < distance_threshold)[0]
    static_pcd = pcd_current.select_by_index(static_indices)
    return np.asarray(static_pcd.points)

def transform_points(points, rotation_matrix, translation_vector):
    points = np.asarray(points)
    return np.dot(points, rotation_matrix.T) + translation_vector

def gicp(points1, points2, threshold=10,  voxel_size=0.1, trans_init=np.eye(4)):
    ''' 
        voxel_size_old = 0.1 | đang test là 2: tạo 1 khung vuông 2x2 và gộp các điểm trong khung vuông
        max_iteration: số lần lặp tối đa
        max_nn + radius: số điểm tối đa max_nn nằm trong bán kính radius được xét (radius = 0.1(old) or 1)

        radius=1, max_nn=30, max_iteration=1000, voxel_size=2  --> tb 2.74
        radius=0.1, max_nn=30, max_iteration=1000, voxel_size=0.1 --> tb 2.79

        Tham số threshold chính là khoảng cách xa nhất mà một cặp điểm có thể được coi là một "cặp tương ứng hợp lệ" (inlier).
        có thể điều chỉnh threshold = 5
        
    '''
    source_pcd = lidar_to_point_cloud(points1)
    target_pcd = lidar_to_point_cloud(points2)

    source_pcd = downsample_point_cloud(source_pcd, voxel_size)
    target_pcd = downsample_point_cloud(target_pcd, voxel_size)


    # Tính toán các vector pháp tuyến cho PointCloud
    source_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    target_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # Tính toán các ma trận hiệp phương sai cho PointCloud
    source_pcd.estimate_covariances(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    target_pcd.estimate_covariances(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # Thiết lập các tham số cho GICP
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1000)

    # Sử dụng GICP để căn chỉnh các điểm quét
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        criteria)
    return reg_p2p.inlier_rmse, reg_p2p.transformation
def bresenham_line(x0, y0, x1, y1):
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    points.append((x1, y1))
    return points

def update_occupancy_map(map_all, mask_map_all, points_global, robot_pos, map_center_xy, delta_xy, scaling_factor,
                         p_occ_inc=0.2, p_free_dec=0.95): # tăng dần độ đậm (nếu điểm đen bị quét trắng nhiều lần sẽ nhạt dần)
    if len(points_global) == 0:
        return
    h, w = map_all.shape[:2]

    robot_x = int(map_center_xy[0] + robot_pos[0] * scaling_factor)
    robot_y = int(map_center_xy[1] - robot_pos[1] * scaling_factor)

    x1 = max(0, robot_x - delta_xy)
    y1 = max(0, robot_y - delta_xy)
    x2 = min(w, robot_x + delta_xy)
    y2 = min(h, robot_y + delta_xy)

    map_all_new = map_all[y1:y2, x1:x2, :]
    mask_map_all_new = mask_map_all[y1:y2, x1:x2]

    height, width = mask_map_all_new.shape
    
    robot_x_new = robot_x - x1
    robot_y_new = robot_y - y1
    for pt in points_global:
        px = int(map_center_xy[0] + pt[0] * scaling_factor - x1)
        py = int(map_center_xy[1] - pt[1] * scaling_factor - y1)

        if not (0 <= px < width and 0 <= py < height):
            continue
        
        line = bresenham_line(robot_x_new, robot_y_new, px, py)

        for (x, y) in line[:-1]:
            if 0 <= x < width and 0 <= y < height:
                mask_map_all_new[y, x] = max(0.0, mask_map_all_new[y, x] * p_free_dec)

        x, y = line[-1]
        if 0 <= x < width and 0 <= y < height:
            mask_map_all_new[y, x] = min(1.0, mask_map_all_new[y, x] + p_occ_inc)

    occ_uint8 = ((1-mask_map_all_new) * 255).astype(np.uint8)
    map_all_new[:, :, 0] = occ_uint8 
    map_all_new[:, :, 1] = occ_uint8 
    map_all_new[:, :, 2] = occ_uint8 

    map_all[y1:y2, x1:x2, :] = map_all_new
    mask_map_all[y1:y2, x1:x2] = mask_map_all_new
    cv2.imshow("hhh", map_all_new)
    return map_all, mask_map_all

# --- PHẦN 3: CHƯƠNG TRÌNH CHÍNH ---
# --- PHẦN 1: CẤU HÌNH VÀ TẢI DỮ LIỆU ---

class Config:
    def __init__(self):
        self.ICP_VOXEL_SIZE = 30.0 # hệ số giảm mẫu
        self.ICP_THRESHOLD = 200.0 # hệ số gicp
        self.global_map = o3d.geometry.PointCloud() # danh sách đám mây điểm gốc
        self.global_pose = np.eye(4) # ma trận tịnh tiến, xoay
        self.first_scan_points = 0 # kiểm tra load map lần đầu
        self.radius = 10000 # bán kích lấu mẫu trong danh sách điểm gốc
        self.giam_mau = 1000 # số lượng bắt đầu lấy giảm mẫu
        self.rmse_ok = 0
    def detect(self, map_all, mask_map_all, global_map, scan_data, MAX_RMSE1_THRESHOLD = 50, MAX_RMSE2_THRESHOLD = 50, scaling_factor = 0.05, update = 1):
        h, w, _ = map_all.shape
        map_center_xy = [w//2, h//2]

        current_points = scan_data
        current_points_global = current_points
        # Chỉ lấy điểm trong bán kính (ví dụ 3000mm quanh robot)
        robot_xy = self.global_pose[:3, 3]
        if self.first_scan_points == 0 and update == 1:
            self.first_scan_points = 1
            global_map.points.extend(o3d.utility.Vector3dVector(scan_data))
            map_all, mask_map_all = update_occupancy_map(map_all, mask_map_all, current_points_global, robot_xy, map_center_xy, delta_xy = 1000, scaling_factor = scaling_factor) # thêm vào danh sách điểm gốc
        
        map_points_all = np.asarray(global_map.points)
        map_points_for_icp = self.filter_points_in_radius(map_points_all, robot_xy, radius=self.radius)


        # map_for_display = map_all.copy()
        # tt = time.time()
        # print("map_points_for_icp.shape", map_points_for_icp.shape)
        rmse, transformation_matrix = gicp(current_points, map_points_for_icp, self.ICP_THRESHOLD, self.ICP_VOXEL_SIZE, trans_init=self.global_pose)
        # print(f"ICP RMSE: {rmse:.4f}", time.time() - tt, map_all.shape)
        # if rmse < 40 and rmse != 0:
        # if self.rmse_ok < 20:
        #     self.rmse_ok = self.rmse_ok + 1
        if rmse <= MAX_RMSE1_THRESHOLD:
            self.global_pose = transformation_matrix # transformation_matrix đã được cộng dồn sau đó lại cập nhật vào global_pose
        if rmse <= MAX_RMSE2_THRESHOLD:
            # Sau khi biến đổi sang tọa độ toàn cục (danh sach điểm vừa mới quét)
            current_points_global = transform_points(current_points, self.global_pose[:3, :3], self.global_pose[:3, 3])
            
            # giảm mẫu thay đổi định dạng điểm đầu vào
            points_to_add = remove_duplicate_points(current_points_global, voxel_size=self.ICP_VOXEL_SIZE)
            
            points_to_add = self.filter_new_points_by_occupancy(points_to_add, mask_map_all, map_center_xy, scaling_factor)
                

            if len(points_to_add) > 0: # thêm danh sách điểm vào danh sach điểm gốc
                global_map.points.extend(o3d.utility.Vector3dVector(points_to_add))
            if len(global_map.points) > self.giam_mau: # giảm mẫu
                global_map = downsample_point_cloud(global_map, 50)

            robot_pos_map = self.global_pose[:3, 3]  # vị trí của robot trong danh sách điểm gốc
            # print("robot_pos_map2", robot_pos_map)
            # hh = time.time()
            if update == 1:
                map_all, mask_map_all = update_occupancy_map(map_all, mask_map_all, current_points_global, robot_pos_map, map_center_xy, delta_xy = 260, scaling_factor = scaling_factor) # thêm vào danh sách điểm gốc
            # print("h-----------h", time.time() - hh)

            global_map = self.prune_global_map(global_map, mask_map_all, map_center_xy, scaling_factor)
            # map_for_display = map_all.copy()

            # vẽ lên ảnh map tỷ lệ
            # self.scan_on_map(map_all, current_points_global, map_center_xy, self.RESOLUTION_MM_PER_PIXEL, color=(0, 255, 0))
        # cv2.imshow("kkk", map_all)
        return map_all, mask_map_all, global_map, rmse, current_points_global, self.global_pose[:3, :3], self.global_pose[:3, 3]


    def filter_points_in_radius(self, all_points, center, radius):
        """
        Lọc các điểm trong bán kính quanh tâm (center)
        all_points: np.ndarray shape (N, 3)
        center: np.array shape (3,)
        radius: float (mm)
        """
        deltas = all_points[:, :2] - center[:2]  # chỉ xét x, y
        distances = np.linalg.norm(deltas, axis=1)
        mask = distances <= radius
        return all_points[mask]
    def filter_new_points_by_occupancy(self, points_to_add, occupancy_probs, map_center_px, scaling_factor, free_threshold=0.3):
        """Lọc các điểm mới dựa trên bản đồ chiếm dụng hiện có.
        Loại bỏ các điểm rơi vào các ô được coi là không gian trống để giảm nhiễu.
        """
        if len(points_to_add) == 0 or occupancy_probs is None:
            return points_to_add
    
        height, width = occupancy_probs.shape
    
        indices_to_keep = []
        for i, point in enumerate(points_to_add):
            px = int(map_center_px[0] + point[0] * scaling_factor)
            py = int(map_center_px[1] - point[1] * scaling_factor)
    
            if not (0 <= px < width and 0 <= py < height):
                indices_to_keep.append(i)
                continue
    
            if occupancy_probs[py, px] < free_threshold:
                continue
        
            indices_to_keep.append(i)
    
        return points_to_add[indices_to_keep]
    def prune_global_map(self, global_map, occupancy_probs, map_center_px, scaling_factor, free_threshold=0.3):
        #lọc những điểm nằm ngoài vùng và những điểm có tỉ lệ chiếm dụng nhỏ hơn ngưỡng
        if len(global_map.points) == 0 or occupancy_probs is None:
            return global_map      
        points = np.asarray(global_map.points)
        height, width = occupancy_probs.shape
    
        indices_to_keep = []
        for i, point in enumerate(points):
            px = int(map_center_px[0] + point[0] * scaling_factor)
            py = int(map_center_px[1] - point[1] * scaling_factor)
    
            if not (0 <= px < width and 0 <= py < height):
                indices_to_keep.append(i)
                continue
            if occupancy_probs[py, px] < free_threshold:
                continue
            indices_to_keep.append(i)
    
        return global_map.select_by_index(indices_to_keep)
    def scan_on_map(self, occupancy_map, points, map_center_px, resolution, color=(0, 255, 0)):
        for point in points:
            point_x_px = int(map_center_px[0] + point[0] / resolution)
            point_y_px = int(map_center_px[1] - point[1] / resolution)
            if 0 <= point_x_px < occupancy_map.shape[1] and 0 <= point_y_px < occupancy_map.shape[0]:
                cv2.circle(occupancy_map, (point_x_px, point_y_px), 1, color, -1)
            


    def draw_local_map(self, points, center_pos, map_size=8000, resolution=30):
        """
        Tạo ảnh hiển thị các điểm trong local map
        - center_pos: vị trí robot (x, y)
        - map_size: chiều dài cạnh bản đồ (mm)
        - resolution: mm/pixel
        """
        pixels = int(map_size / resolution)
        map_img = np.full((pixels, pixels, 3), 255, dtype=np.uint8)
        center_px = pixels // 2
    
        for pt in points:
            dx = pt[0] - center_pos[0]
            dy = pt[1] - center_pos[1]
            x_px = int(center_px + dx / resolution)
            y_px = int(center_px - dy / resolution)
            if 0 <= x_px < pixels and 0 <= y_px < pixels:
                cv2.circle(map_img, (x_px, y_px), 1, (0, 0, 0), -1)
    
        return map_img
    
    