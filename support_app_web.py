import os
import numpy as np
import open3d as o3d

def get_available_maps(PATH_MAPS_DIR):
    """Scans the map directory and returns a list of unique map basenames."""
    map_basenames = set()
    if os.path.exists(PATH_MAPS_DIR):
        for f_name in os.listdir(PATH_MAPS_DIR):
            if f_name.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif', '.npy', '.npz', '.pcd')):
                basename = os.path.splitext(f_name)[0]
                map_basenames.add(basename)
    return sorted(list(map_basenames))

def save_current_map(map_name, save_path_dir, map_all):
    """
    Lưu self.map_all (3 kênh BGR) vào file .npy.
    Args:
        map_name (str): Tên của bản đồ (không bao gồm phần mở rộng .npy).
        save_path_dir (str): Đường dẫn đến thư mục để lưu bản đồ.
    """
    if not map_name:
        print("Lỗi: Tên bản đồ rỗng. Không thể lưu.")
        return
    if not os.path.exists(save_path_dir):
        try:
            os.makedirs(save_path_dir, exist_ok=True)
            print(f"Đã tạo thư mục: {save_path_dir}")
        except OSError as e:
            print(f"Lỗi tạo thư mục {save_path_dir}: {e}")
            return

    file_path = os.path.join(save_path_dir, f"{map_name}.npy")
    
    if map_all.shape[2] >= 3: # Đảm bảo có ít nhất 3 kênh
        map_to_save = map_all # Lấy 3 kênh đầu tiên (giả sử là BGR)
    else:
        print(f"Lỗi: map_all không có đủ 3 kênh màu: {map_all.shape}")
        return
    try:
        np.save(file_path, map_to_save)
        print(f"Bản đồ đã được lưu thành công vào: {file_path}")
    except Exception as e:
        print(f"Lỗi khi lưu bản đồ vào {file_path}: {e}")
def save_mask_map(map_name, save_path_dir, mask_map_array):
    """
    Lưu mảng NumPy mask_map_array vào một tệp tin nén .npz.

    Args:
        file_path (str): Đường dẫn tệp tin để lưu, ví dụ: "data/mask_map.npz".
        mask_map_array (np.ndarray): Mảng NumPy cần lưu.
    """
    if not map_name:
        print("Lỗi: Tên bản đồ rỗng. Không thể lưu.")
        return
    if not os.path.exists(save_path_dir):
        try:
            os.makedirs(save_path_dir, exist_ok=True)
            print(f"Đã tạo thư mục: {save_path_dir}")
        except OSError as e:
            print(f"Lỗi tạo thư mục {save_path_dir}: {e}")
            return

    file_path = os.path.join(save_path_dir, f"{map_name}.npz")
    try:
        np.savez_compressed(file_path, mask_map=mask_map_array)
        print(f"Đã lưu mask map thành công vào: {file_path}")
    except Exception as e:
        print(f"Lỗi khi lưu mask map: {e}")
def save_point_cloud(map_name, save_path_dir, point_cloud):
    """
    Lưu đối tượng PointCloud vào tệp tin .pcd.

    Args:
        file_path (str): Đường dẫn tệp tin để lưu, ví dụ: "data/my_cloud.pcd".
        point_cloud (o3d.geometry.PointCloud): Đối tượng đám mây điểm cần lưu.
    """
    if not map_name:
        print("Lỗi: Tên bản đồ rỗng. Không thể lưu.")
        return
    if not os.path.exists(save_path_dir):
        try:
            os.makedirs(save_path_dir, exist_ok=True)
            print(f"Đã tạo thư mục: {save_path_dir}")
        except OSError as e:
            print(f"Lỗi tạo thư mục {save_path_dir}: {e}")
            return

    file_path = os.path.join(save_path_dir, f"{map_name}.pcd")
    try:
        o3d.io.write_point_cloud(file_path, point_cloud)
        print(f"Đã lưu đám mây điểm thành công vào: {file_path}")
    except Exception as e:
        print(f"Lỗi khi lưu đám mây điểm: {e}")

def load_point_cloud(self, file_path):
    """
    Đọc tệp tin .pcd và trả về đối tượng PointCloud.

    Args:
        file_path (str): Đường dẫn tệp tin .pcd để đọc.

    Returns:
        o3d.geometry.PointCloud: Đối tượng đám mây điểm đã đọc.
    """
    try:
        point_cloud = o3d.io.read_point_cloud(file_path)
        print(f"Đã đọc thành công đám mây điểm từ: {file_path}")
        return point_cloud
    except Exception as e:
        print(f"Lỗi khi đọc đám mây điểm: {e}")
        return None