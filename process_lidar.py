import numpy as np
import cv2, threading, time, math, shutil, os
from support_main.lib_main import edit_csv_tab, remove
import path
import app_web
import open3d as o3d
import detect_gicp






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
        if data_admin[i][0] == "distan_scan_all":
            distan_scan_all = [int(float(data_admin[i][1])), int(float(data_admin[i][2]))] 
        if data_admin[i][0] == "x_goc0":
            if data_admin[i][1] != "none":
                x_goc0 = int(float(data_admin[i][1]))
        if data_admin[i][0] == "y_goc0":
            if data_admin[i][1] != "none":
                y_goc0 = int(float(data_admin[i][1]))
        if data_admin[i][0] == "rotation0":
            if data_admin[i][1] != "none":
                rotation0 = float(data_admin[i][1])
        if data_admin[i][0] == "kc_them_map":
            kc_them_map = int(float(data_admin[i][1]))
        if data_admin[i][0] == "anpha_scan":
            anpha_scan = [int(float(data_admin[i][1])), int(float(data_admin[i][2]))] 
        
print("distan_scan_all", distan_scan_all, x_goc0, y_goc0, rotation0, anpha_scan)

class process_data_lidar:
    def __init__(self, window_size,window_size_all,scaling_factor,rmse1,rmse2):
        self.window_size_x_img2 = 400
        self.window_size_y_img2 = 400

        self.window_size_x_all = window_size_all
        self.window_size_y_all = window_size_all
        self.scaling_factor = scaling_factor
        self.scan_xy = np.array([[0, 0, 0]])

        self.map_all = np.full((self.window_size_y_all, self.window_size_x_all, 3), 128, dtype=np.uint8)
        self.mask_map_all = np.full((self.window_size_y_all, self.window_size_x_all), 0.5, dtype=np.float32)  # bắt đầu ở 0.5
        self.global_map = o3d.geometry.PointCloud() # danh sách đám mây điểm gốc
        self.img1 = self.map_all.copy()
        self.img2 = self.map_all.copy()[int(self.window_size_x_all//2 - self.window_size_y_img2):int(self.window_size_x_all//2 + self.window_size_y_img2),
                                     int(self.window_size_x_all//2 - self.window_size_x_img2):int(self.window_size_x_all//2 + self.window_size_x_img2)]

        self.rmse = 0
        self.rmse1 = rmse1
        self.rmse2 = rmse2

        self.x_goc = int(self.window_size_x_all / 2)
        self.y_goc = int(self.window_size_y_all / 2)
        self.rotation = 0
        self.tam_x_agv, self.tam_y_agv = [int(self.window_size_x_all / 2), int(self.window_size_y_all / 2)]
        self.huong_x = 0
        self.huong_y = 0


        self.time_start = 0

        # bằng 1 thì cập nhật vào map (thực hiện quét map)
        self.add_all_point = 0
        # thêm vào map nếu = 1 ( bỏ qua điều khiện < rmse)
        self.add_map = 0

        self.detect_gicp_lidar = detect_gicp.Config()
    def main_loop(self, scan_xy):
        self.process_data_lidar(scan_xy)
        # self.load_data_web()
    # def load_data_web(self):
    #     if webserver.dict_chon_ban_do["update"] == 1:
    #         webserver.dict_chon_ban_do["update"] = 0
    #         map_name = webserver.dict_chon_ban_do["ten_ban_do"]
    #         map_name, extension = os.path.splitext(map_name)
    #         map_path = os.path.join(PATH_MAPS_DIR, map_name)

    #         load_ok = 0
            
    #         if os.path.exists(map_path + ".npy"):
    #             loaded_map_all = np.load(map_path + ".npy")
    #             if loaded_map_all is not None and loaded_map_all.dtype != np.uint8:
    #                 loaded_map_all = loaded_map_all.astype(np.uint8)
    #             load_ok = load_ok + 1
    #         if os.path.exists(map_path + ".npz"):    
    #             loaded_mask_map_all = np.load(map_path + ".npz")['mask_map']
    #             if loaded_mask_map_all is not None and loaded_mask_map_all.dtype != np.float32:
    #                 loaded_mask_map_all = loaded_mask_map_all.astype(np.float32)
    #             load_ok = load_ok + 1
    #         if os.path.exists(map_path + ".pcd"):  
    #             global_map = o3d.io.read_point_cloud(map_path + ".pcd")
    #             load_ok = load_ok + 1
    #         if load_ok == 3:
    #             self.map_all = loaded_map_all
    #             self.mask_map_all = loaded_mask_map_all
    #             self.global_map = global_map
    #             print("----------------------- load_map")
    
    def process_data_lidar(self, scan_xy):
        if distan_scan_all[0] == 1:
            x_coords = scan_xy[:, 0]
            y_coords = scan_xy[:, 1]
            distances = np.sqrt(x_coords**2 + y_coords**2)
            scan_all = scan_xy[distances < distan_scan_all[1]]
            # print("scan_all.shape", scan_all.shape)
        else:
            scan_all = scan_xy
        self.scan_xy = scan_all
        # self.sent_data_driver_motor["scan"] = scan_xy #################################

        self.img1 = self.map_all.copy()
        h_img1,w_img1,_ = self.img1.shape

        px = self.scan_xy[:,0]
        py = self.scan_xy[:,1]
        arr_test = np.vstack((px, py, np.zeros_like(px))).T  # Thêm chiều z = 0 để tạo PointCloud 3D
        if self.time_start == 0:
            self.time_start = time.time()
        
        if (app_web.dict_dieu_chinh_vi_tri_agv["setup"] == 0) and time.time() - self.time_start > 0:
            if arr_test.shape[0] > 10:
                stop_agv = 0
                tt = time.time()
                self.map_all, self.mask_map_all, self.global_map, self.rmse, new_arr_ok, r, t = self.detect_gicp_lidar.detect(self.map_all.copy(), self.mask_map_all.copy(), self.global_map, arr_test, self.rmse1, self.rmse2, self.scaling_factor, update=self.add_all_point)
                print("-----------",time.time() - tt, self.rmse)
                self.x_goc = int(t[0] * self.scaling_factor + self.window_size_x_all//2)
                self.y_goc = int(- t[1] * self.scaling_factor + self.window_size_y_all//2)
                self.rotation = - np.arccos((np.trace(r) - 1) / 2)
                
                new_arr_ok[:, 0] = new_arr_ok[:, 0] * self.scaling_factor + self.window_size_x_all//2 
                new_arr_ok[:, 1] = - new_arr_ok[:, 1] * self.scaling_factor + self.window_size_y_all//2 
                for ii in range(0, max(len(new_arr_ok), new_arr_ok.shape[0])):
                    # hiển thị kết quả của icp lên ảnh
                    if ii < len(new_arr_ok[:, 0]) - 1 and int(new_arr_ok[ii, 0]) < w_img1 and int(new_arr_ok[ii, 1]) < h_img1:
                        cv2.circle(self.img1, (int(new_arr_ok[ii, 0]), int(new_arr_ok[ii, 1])), 1, (255, 0, 255), -1)

                self.add_map = 0
            else:
                stop_agv = 1
                new_arr_ok = arr_test
            
            self.rotation = self.normalize_angle_rad(self.rotation)
            self.tam_x_agv, self.tam_y_agv = self.translate_point(self.x_goc, self.y_goc, - self.rotation, distance=0)

        else:
            
            new_arr_ok = arr_test
            self.tam_x_agv = self.x_goc
            self.tam_y_agv = self.y_goc
            stop_agv = 1
        
        h_img1, w_img1, _ = self.img1.shape
        cv2.circle(self.img1, (int(self.tam_x_agv), int(self.tam_y_agv)), 5, (255, 0, 0), -1)
        self.huong_x = int(self.tam_x_agv + 20 * math.cos(np.pi - self.rotation))
        self.huong_y = int(self.tam_y_agv + 20 * math.sin(np.pi - self.rotation))

        self.huong_x2 = int(self.tam_x_agv + 20 * math.cos(-self.rotation))
        self.huong_y2 = int(self.tam_y_agv + 20 * math.sin(-self.rotation))

        cv2.arrowedLine(self.img1, (int(self.tam_x_agv), int(self.tam_y_agv)), (self.huong_x2, self.huong_y2), (255, 0, 0), 1, tipLength=0.2)

        x1 = max(0, self.x_goc - self.window_size_x_img2)
        y1 = max(0, self.y_goc - self.window_size_y_img2)
        x2 = min(self.x_goc + self.window_size_x_img2, w_img1)
        y2 = min(self.y_goc + self.window_size_y_img2, h_img1)

        self.img2 = self.img1.copy()[y1:y2,x1:x2,:]


        app_web.points_color_blue = new_arr_ok[:,:2]
        app_web.image_all = self.map_all.copy()
        if app_web.dict_dieu_chinh_vi_tri_agv["update"] == 0 and app_web.dict_dieu_chinh_vi_tri_agv["setup"] == 0:
            app_web.dict_dieu_chinh_vi_tri_agv["toa_do_x"] = self.tam_x_agv
            app_web.dict_dieu_chinh_vi_tri_agv["toa_do_y"] = self.tam_y_agv
            app_web.dict_dieu_chinh_vi_tri_agv["goc_agv"] = (self.rotation * 180 / np.pi) % 360
        # self.sent_data_driver_motor["tam_x_agv"] = self.tam_x_agv
        # self.sent_data_driver_motor["tam_y_agv"] = self.tam_y_agv
        # self.sent_data_driver_motor["rotation"] = self.rotation
        # self.sent_data_driver_motor["stop"] = stop_agv
        # self.sent_data_driver_motor["huong_x"] = self.huong_x
        # self.sent_data_driver_motor["huong_y"] = self.huong_y

    def translate_point(self, x, y, alpha, distance=5):
        """
        Dịch chuyển điểm (x, y) theo góc alpha và khoảng cách distance.
        
        Args:
            x (float): Tọa độ x của điểm đầu vào.
            y (float): Tọa độ y của điểm đầu vào.
            alpha (float): Góc alpha (đơn vị: radians).
            distance (float): Khoảng cách dịch chuyển (mặc định: 5 pixel).
        
        Returns:
            tuple: Tọa độ điểm mới (x_new, y_new).
        """
        x_new = x + distance * math.cos(alpha)
        y_new = y + distance * math.sin(alpha)
        return x_new, y_new
    def normalize_angle_rad(self, angle):
        """
        Chuyển đổi góc (radian) về khoảng [-pi, pi] radian.
        Ví dụ: 7 -> ~0.716 rad, -4 -> ~2.283 rad
        """
        angle = angle % (2 * np.pi)
        if angle > np.pi:
            angle -= 2 * np.pi
        if angle < -np.pi:
            angle += 2 * np.pi
        return angle    
    


