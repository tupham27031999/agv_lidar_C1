from app_web import app
import app_web
import threading
import time
import webbrowser
import cv2
import numpy as np
import socket
import path
import os
from support_main.lib_main import edit_csv_tab
import open3d as o3d
from pynput import keyboard
from support_main import connect_lidar_sick, connect_lidar
import connect_lidar_C1
import process_lidar
import detect_gicp
import convert_2_lidar
import driver_control_input

# image_lock shared_data_lock

def get_local_ip():
    """
    Tự động lấy địa chỉ IPv4 của máy tính trong mạng LAN.
    """
    s = None
    try:
        # Tạo một socket để kết nối ra ngoài.
        # Không cần gửi dữ liệu, chỉ cần thực hiện kết nối để hệ điều hành
        # chọn interface mạng phù hợp.
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80)) # 8.8.8.8 là DNS của Google
        ip_address = s.getsockname()[0]
        return ip_address
    except Exception as e:
        print(f"Không thể tự động lấy địa chỉ IP, sử dụng '12_7.0.0.1'. Lỗi: {e}")
        return "127.0.0.1" # Trả về localhost nếu có lỗi
    finally:
        if s:
            s.close()


path_phan_mem = path.path_phan_mem
path_admin = path_phan_mem + "/setting/admin_window.csv"

# Directories for saving data and maps
SAVED_DATA_DIR = path_phan_mem + "/data_input_output"
PATH_MAPS_DIR = SAVED_DATA_DIR + "/maps"
name_map = "map"

if os.name == "nt":
    print("Hệ điều hành là Windows")
    # Đọc file cài đặt cho Windows
    path_admin = path_phan_mem + "/setting/admin_window.csv"
elif os.name == "posix":
    print("Hệ điều hành là Ubuntu (Linux)")
    # Đọc file cài đặt cho Ubuntu
    path_admin = path_phan_mem + "/setting/admin_ubuntu.csv"
path_folder_scan_lidar1 = path_phan_mem + "/data_input_output/scan_data_1"
path_folder_scan_lidar2 = path_phan_mem + "/data_input_output/scan_data_2"
path_folder_scan_lidar3 = path_phan_mem + "/data_input_output/scan_data_3"
window_size = 700
window_size_all = 5000
scaling_factor = 0.05
rmse1 = 3
rmse2 = 3 
host = get_local_ip() # Tự động lấy IP
port = "5000"
loai_lidar = "usb"
data_admin = edit_csv_tab.load_all_stt(path_admin)
for i in range(0,len(data_admin)):
    if len(data_admin[i]) > 1:
        if data_admin[i][0] == "window_size": # kích thước ảnh check map
            window_size = int(float(data_admin[i][1]))
        if data_admin[i][0] == "window_size_all": # kich thuoc ảnh tổng
            window_size_all = int(float(data_admin[i][1]))
        if data_admin[i][0] == "scaling_factor":
            scaling_factor = float(data_admin[i][1])
        if data_admin[i][0] == "rmse1": # sai so 
            rmse1 = int(float(data_admin[i][1]))
        if data_admin[i][0] == "rmse2": # sai so cap nhat map
            rmse2 = int(float(data_admin[i][1]))
        if data_admin[i][0] == "port":
            port = int(float(data_admin[i][1]))
        if data_admin[i][0] == "loai_lidar":
            loai_lidar = data_admin[i][1]

print("scaling_factor", scaling_factor, host)
# host = "172.26.76.151"
# port = "5000"
path_folder_scan_data_0 = SAVED_DATA_DIR + "/scan_data_0"
path_folder_scan_data_1 = SAVED_DATA_DIR + "/scan_data_1"
path_folder_scan_data_2 = SAVED_DATA_DIR + "/scan_data_2"
path_folder_scan_data_3 = SAVED_DATA_DIR + "/scan_data_3"



def save_scan_to_npy(scan_data, filename, directory="scan_data"):
    """
    Lưu trữ dữ liệu scan (mảng NumPy) vào một file .npy.

    Args:
        scan_data (np.ndarray): Mảng NumPy chứa dữ liệu scan cần lưu.
        filename (str): Tên file để lưu (không bao gồm phần mở rộng .npy).
        directory (str): Thư mục để lưu file. Mặc định là "scan_data".

    Returns:
        bool: True nếu lưu thành công, False nếu có lỗi.
        str: Đường dẫn đầy đủ đến file đã lưu, hoặc thông báo lỗi.
    """
    if not isinstance(scan_data, np.ndarray):
        return False, "Lỗi: scan_data phải là một mảng NumPy."
    if not filename:
        return False, "Lỗi: Tên file không được để trống."

    try:
        os.makedirs(directory, exist_ok=True)
        filepath = os.path.join(directory, f"{filename}.npy")
        np.save(filepath, scan_data)
        return True, f"Đã lưu dữ liệu scan vào: {os.path.abspath(filepath)}"
    except Exception as e:
        return False, f"Lỗi khi lưu file .npy: {e}"

def load_scan_from_npy(filename, directory="scan_data"):
    """
    Đọc dữ liệu scan từ một file .npy.

    Args:
        filename (str): Tên file cần đọc (không bao gồm phần mở rộng .npy).
        directory (str): Thư mục chứa file. Mặc định là "scan_data".

    Returns:
        np.ndarray or None: Mảng NumPy chứa dữ liệu scan nếu đọc thành công,
                            None nếu có lỗi.
        str: Thông báo thành công hoặc lỗi.
    """
    if not filename:
        return None, "Lỗi: Tên file không được để trống."

    filepath = os.path.join(directory, f"{filename}.npy")
    try:
        if not os.path.exists(filepath):
            return None, f"Lỗi: File không tồn tại tại đường dẫn: {os.path.abspath(filepath)}"
        scan_data = np.load(filepath)
        return scan_data, f"Đã đọc thành công dữ liệu từ: {os.path.abspath(filepath)}"
    except Exception as e:
        return None, f"Lỗi khi đọc file .npy: {e}"

class support_main:
    def __init__(self,window_size,window_size_all,scaling_factor,rmse1,rmse2):
        self.stt_scan = 0
        self.window_size = window_size
        self.window_size_all = window_size_all
        self.scaling_factor = scaling_factor
        self.rmse1 = rmse1
        self.rmse2 = rmse2

        self.is_saving_map_mode = False
        self.current_map_name_input = ""
        self.save_map_window_name = "Luu Ban Do - Nhap Ten" # Đã sửa để có dấu
        self.input_rect = (50, 70, 300, 30) # x, y, w, h for text input box
        self.save_button_rect = (150, 130, 100, 40) 

        self.kiem_tra_connect = {"lidar": "on", "driver_motor": "off", "esp32": "off", "process_lidar": "on"}

        self.img = np.zeros((self.window_size,self.window_size,3), np.uint8)
        self.map_all = np.zeros((self.window_size,self.window_size,3), np.uint8)

        # Khởi động listener cho bàn phím
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

        self.data_sent_process_lidar = {"add_all_point": 0, "add_map": 0}

        self.key_states = {}
        self.time_dk_tay = 0
        self.data_sent_dk_driver = {"dk_tay": 0, "data_dk_tay": ""}
        self.tien, self.lui, self.trai, self.phai, self.stop = np.zeros(5)

        self.connect = {"lidar": False, "driver_motor": False, "esp32": False}

        self.connect_while = True
        # usb/ethernet
        self.loai_lidar = loai_lidar
        if self.kiem_tra_connect["lidar"] == "on":
            # Giả sử process_lidar được khởi tạo ở đây nếu cần thiết sớm
            # hoặc đảm bảo nó được khởi tạo trước khi main_loop chạy lần đầu
            if self.kiem_tra_connect["process_lidar"] == "on":
                self.process_lidar = process_lidar.process_data_lidar(self.window_size,self.window_size_all,self.scaling_factor,self.rmse1,self.rmse2)
            if self.loai_lidar != "usb":
                self.lidar = connect_lidar_sick.LidarP()
                threading.Thread(target=self.lidar.process_data).start()
            else:
                self.lidar = connect_lidar_C1.main_lidar()
                self.lidar.connect_lidar()

        if self.kiem_tra_connect["process_lidar"] == "on":
            if not hasattr(self, 'process_lidar'): # Đảm bảo chỉ khởi tạo một lần
                self.process_lidar = process_lidar.process_data_lidar(self.window_size,self.window_size_all,self.scaling_factor,self.rmse1,self.rmse2)
        
        if self.kiem_tra_connect["esp32"] == "on":
            load_data_esp = 1
        else:
            load_data_esp = 0
        self.detect_data_driver = driver_control_input.detect_data_sent_driver(load_data_esp)

    def main_loop(self):


        list_data1 = os.listdir(path_folder_scan_lidar1)
        list_data2 = os.listdir(path_folder_scan_lidar2)
        list_data3 = os.listdir(path_folder_scan_lidar3)
        if self.kiem_tra_connect["lidar"] == "off":
            # if self.stt_scan < len(list_data1) and self.stt_scan < len(list_data2):
            #     scan_alpha_1 = np.load(path_folder_scan_lidar1 + "/scan_"+ str(self.stt_scan) +".npy")
            #     scan_alpha_2 = np.load(path_folder_scan_lidar2 + "/scan_"+ str(self.stt_scan) +".npy")
            #     # self.stt_scan = self.stt_scan + 1
            #     self.stt_scan = 5
            # else:
            #     self.connect_while = False
            #     self.stt_scan = self.stt_scan - 1
            #     scan_alpha_1 = np.load(path_folder_scan_lidar1 + "/scan_"+ str(self.stt_scan) +".npy")
            #     scan_alpha_2 = np.load(path_folder_scan_lidar2 + "/scan_"+ str(self.stt_scan) +".npy")
            
            if self.stt_scan < len(list_data3):
                scan_alpha_1 = np.load(path_folder_scan_lidar1 + "/Scan_data_"+ str(self.stt_scan) +".npy")
                self.stt_scan = self.stt_scan + 1
                # self.stt_scan = 5
            else:
                self.connect_while = False
                self.stt_scan = self.stt_scan - 1
                scan_alpha_1 = np.load(path_folder_scan_lidar1 + "/Scan_data_"+ str(self.stt_scan) +".npy")
        else:
            scan_alpha_1, _ = self.load_data_lidar()
            # print(scan_alpha_1)

        # scan_xy, scan1, scan2 = convert_2_lidar.convert_scan_lidar(scan1_data_example=scan_alpha_1, 
        #                                                              scan2_data_example=scan_alpha_2, 
        #                                                              scaling_factor = self.scaling_factor)
        scan_xy, scan1, scan2 = convert_2_lidar.convert_scan_lidar( scan1_data_example=scan_alpha_1, 
                                                                    scan2_data_example=scan_alpha_1, 
                                                                    scaling_factor = 1,
                                                                    lidar1_orient_deg = 0,
                                                                    lidar2_orient_deg = 0,
                                                                    agv_w=0,
                                                                    agv_l=0)
        # scan_xy = detect_gicp.remove_duplicate_points(scan_xy)
        # print(scan_xy)
        # print(scan_xy.shape)

        # scan_xy, scan1, scan2 = convert_2_lidar.convert_scan_lidar(scan1_data_example=scan_alpha_3, 
        #                                                             scan2_data_example=scan_alpha_3, 
        #                                                             scaling_factor = 1,
        #                                                             lidar1_orient_deg = 0,
        #                                                             lidar2_orient_deg = 0,
        #                                                             agv_w=0,
        #                                                             agv_l=0)

        # scan, _ = self.load_data_lidar()
        # print("scan = ", scan, scan0.shape)

        if self.kiem_tra_connect["process_lidar"] == "on":
            # self.detect_data_driver.load_data_driver_motor = self.process_lidar.sent_data_driver_motor ###################################################
            # thêm map, quét map hay không
            self.set_data_process()
            # xử lý tín hiệu lidar icp
            self.process_lidar.main_loop(scan_xy) # process_lidar.py cập nhật self.process_lidar.tam_x_agv, .tam_y_agv, .rotation
            # xử lý tín hiệu web gửi về, và gửi cho driver motor
            self.detect_data_driver.void_loop()
        
        self.load_data_web()
        self.show_img()
    def load_data_web(self):
        ################################### lưu bản đồ ########################################
        map_name_to_save = ""
        # Sử dụng lock để đọc và ghi vào dictionary chia sẻ một cách an toàn.
        # Thao tác này đảm bảo rằng luồng web không thể thay đổi dict_ban_do_moi
        # trong khi luồng chính đang xử lý nó.
        with app_web.shared_data_lock:
            if app_web.dict_ban_do_moi["ten_ban_do_moi"] and app_web.dict_ban_do_moi["luu_ban_do_moi"] == 1:
                map_name_to_save = app_web.dict_ban_do_moi["ten_ban_do_moi"]
                # Reset các cờ ngay bên trong lock để tránh xử lý lại yêu cầu
                app_web.dict_ban_do_moi["tao_ban_do_moi"] = 0
                app_web.dict_ban_do_moi["ten_ban_do_moi"] = ""
                app_web.dict_ban_do_moi["luu_ban_do_moi"] = 0
            # Đọc trạng thái điều khiển bằng tay bên trong lock
            self.data_sent_dk_driver["dk_tay"] = app_web.dict_ban_do_moi["tao_ban_do_moi"]
        # Thực hiện việc lưu file (tốn thời gian) bên ngoài lock
        if map_name_to_save:
            self.save_current_map(map_name_to_save, PATH_MAPS_DIR, self.process_lidar.map_all)
            self.save_mask_map(map_name_to_save, PATH_MAPS_DIR, self.process_lidar.mask_map_all)
            self.save_point_cloud(map_name_to_save, PATH_MAPS_DIR, self.process_lidar.global_map)
        ################################### lưu bản đồ ########################################


    def show_img(self):
        if self.kiem_tra_connect["process_lidar"] == "on":
            self.img = self.process_lidar.img2
        
        
        color_gray = (120, 120, 120)
        color_red = (0, 0, 255)
        color_green = (0, 255, 0)
        # hiển thị thông tin
        # process data lidar
        if self.kiem_tra_connect["process_lidar"] == "on":
            # rmse độ chính xác (càng bé càng tốt)
            rmse = self.process_lidar.rmse
            # trung_binh = self.process_lidar.trung_binh
            # bien_dem = self.process_lidar.bien_dem
            # if bien_dem != 0:
            #     trung_binh = trung_binh/bien_dem
            # print(trung_binh, bien_dem)
            if rmse > self.rmse1:
                cv2.putText(self.img, "Do chinh xac: " + str(int(rmse * 100) / 100), (20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_red, 2)
            else:
                cv2.putText(self.img, "Do chinh xac: " + str(int(rmse * 100) / 100), (20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_green, 2)
            # cv2.putText(self.img, "Trung binh: " + str(trung_binh), (20,80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_green, 2)
        # chế độ điều khiển tay
        self.detect_data_driver.dk_agv_thu_cong = self.data_sent_dk_driver["dk_tay"]
        if self.data_sent_dk_driver["dk_tay"] == 1:
            if self.kiem_tra_connect["driver_motor"] == "on":
                cv2.putText(self.img, "Dieu khien tay: ON", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_green, 2)
            else:
                cv2.putText(self.img, "Dieu khien tay: ON (driver disconnect)", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_green, 2)
            self.data_sent_process_lidar["add_all_point"] = 1
        else:
            cv2.putText(self.img, "Dieu khien tay: OFF", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_gray, 2)
            self.data_sent_dk_driver["data_dk_tay"] = ""
            self.data_sent_process_lidar["add_all_point"] = 0

        
        cv2.imshow("map",self.img)
        # Xử lý sự kiện bàn phím
        key = cv2.waitKey(1) & 0xFF

        # Xử lý phím cho cửa sổ chính "map"
        if key == ord('s'):
            if not self.is_saving_map_mode: # Chỉ mở nếu chưa ở mode save
                self.is_saving_map_mode = True
                self.current_map_name_input = ""
                cv2.namedWindow(self.save_map_window_name)
                cv2.setMouseCallback(self.save_map_window_name, self.save_map_mouse_callback)
                self.display_save_map_window() # Hiển thị lần đầu
        # Xử lý sự kiện bàn phím
        elif key == ord('d'):
            self.data_sent_dk_driver["dk_tay"] = not self.data_sent_dk_driver["dk_tay"]
        # them các điểm vào map
        elif key == ord('a'):
            self.data_sent_process_lidar["add_map"] = 1
        # thoat chương trình
        elif key == ord('q') or cv2.getWindowProperty('map', cv2.WND_PROP_VISIBLE) < 1:
            if self.kiem_tra_connect["lidar"] == "on":
                if self.loai_lidar != "usb":
                    self.lidar.connect = False
                else:
                    self.lidar.disconnect()
            self.connect_while = False
        else:
            # lưu map
            if self.is_saving_map_mode:
                self.display_save_map_window()
                self.handle_save_map_input(key)
        # Xử lý sự kiện bàn phím
        if self.data_sent_dk_driver["dk_tay"] == 1:
            if self.key_states.get(keyboard.Key.up, False):  # tiến
                self.tien = 1
                self.time_dk_tay = time.time()
            else:
                self.tien = 0
            if self.key_states.get(keyboard.Key.down, False):  # lùi
                self.lui = 1
                self.time_dk_tay = time.time()
            else:
                self.lui = 0
            if self.key_states.get(keyboard.Key.left, False):  # trái
                self.trai = 1
                self.time_dk_tay = time.time()
            else:
                self.trai = 0
            if self.key_states.get(keyboard.Key.right, False):  # phải
                self.phai = 1
                self.time_dk_tay = time.time()
            else:
                self.phai = 0
            if self.tien == 0 and self.lui == 0 and self.trai == 0 and self.phai == 0 and (self.time_dk_tay == 0 or time.time() - self.time_dk_tay > 1):  # stop
                self.stop = 1
            
            if self.stop == 1:
                self.data_dk_tay = "stop"
            if self.tien == 1:
                if self.trai == 0 and self.phai == 0:
                    self.data_dk_tay = "tien"
                if self.trai == 1:
                    self.data_dk_tay = "dich_tien_trai"
                if self.phai == 1:
                    self.data_dk_tay = "dich_tien_phai"
            if self.lui == 1:
                if self.trai == 0 and self.phai == 0:
                    self.data_dk_tay = "lui"
                if self.trai == 1:
                    self.data_dk_tay = "dich_lui_trai"
                if self.phai == 1:
                    self.data_dk_tay = "dich_lui_phai"
            if self.tien == 0 and self.lui == 0:
                if self.trai == 1:
                    self.data_dk_tay = "trai"
                if self.phai == 1:
                    self.data_dk_tay = "phai"
            self.detect_data_driver.data_dk_tay = self.data_dk_tay
        else:
            self.data_dk_tay = ""
            self.detect_data_driver.data_dk_tay = ""
        

            
    def on_press(self, key):
        try:
            self.key_states[key.char] = True
        except AttributeError:
            self.key_states[key] = True

    def on_release(self, key):
        try:
            self.key_states[key.char] = False
        except AttributeError:
            self.key_states[key] = False
    def load_data_lidar(self):
        if self.kiem_tra_connect["lidar"] == "on":
            if self.loai_lidar != "usb":
                scan, check = self.lidar.get_data()
            else:
                self.lidar.connect_lidar()
                scan, check = self.lidar.return_data()
                self.lidar.time_close = time.time()
        else:
            scan = np.array([[0, 0, 0]])
            check = False
        scan = np.array(scan)
        return scan, check
    def set_data_process(self):
        self.process_lidar.add_all_point = self.data_sent_process_lidar["add_all_point"]
        self.process_lidar.add_map = self.data_sent_process_lidar["add_map"]
    
    def save_current_map(self, map_name, save_path_dir):
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
        
        if self.map_all.shape[2] >= 3: # Đảm bảo có ít nhất 3 kênh
            map_to_save = self.map_all[:, :, :3] # Lấy 3 kênh đầu tiên (giả sử là BGR)
        else:
            print(f"Lỗi: map_all không có đủ 3 kênh màu: {self.map_all.shape}")
            return
        try:
            np.save(file_path, map_to_save)
            print(f"Bản đồ đã được lưu thành công vào: {file_path}")
        except Exception as e:
            print(f"Lỗi khi lưu bản đồ vào {file_path}: {e}")
    def handle_save_map_input(self, key):
        if not self.is_saving_map_mode:
            return False # Key not handled by this mode

        if key == 27: # ESC
            self.is_saving_map_mode = False
            cv2.destroyWindow(self.save_map_window_name)
            return True
        elif key == 13: # Enter
            self.trigger_save_map()
            return True
        elif key == 8: # Backspace
            self.current_map_name_input = self.current_map_name_input[:-1]
            return True
        elif 32 <= key <= 126: # Printable ASCII (cho tên file tiếng Anh)
            # Giới hạn độ dài tên file để tránh tràn giao diện
            if len(self.current_map_name_input) < 30: # Giới hạn 30 ký tự ví dụ
                self.current_map_name_input += chr(key)
            return True
        # Nếu bạn muốn hỗ trợ nhập tiếng Việt có dấu ở đây, sẽ cần giải pháp phức tạp hơn
        # vì cv2.waitKey() trả về mã ASCII.
        return False # Key not handled by this specific logic

    def save_map_mouse_callback(self, event, x, y, flags, param):
        if not self.is_saving_map_mode:
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            bx, by, bw, bh = self.save_button_rect
            if bx <= x <= bx + bw and by <= y <= by + bh:
                self.trigger_save_map()

    def trigger_save_map(self):
        # if self.current_map_name_input.strip():
        #     name_map_to_save = self.current_map_name_input.strip()
        #     print(f"Đang lưu bản đồ với tên: {name_map_to_save}.npy")
        #     if self.kiem_tra_connect["process_lidar"] == "on" and hasattr(self, 'process_lidar'):
        #         self.save_current_map(name_map_to_save, PATH_MAPS_DIR, self.process_lidar.map_all)
        #         self.save_mask_map(name_map_to_save, PATH_MAPS_DIR, self.process_lidar.mask_map_all)
        #         self.save_point_cloud(name_map_to_save, PATH_MAPS_DIR, self.process_lidar.global_map)
        #         # Cập nhật danh sách bản đồ trên webserver
        #         if hasattr(webserver, 'get_available_maps') and callable(webserver.get_available_maps):
        #             webserver.list_ban_do = webserver.get_available_maps()
        #             print("Đã cập nhật danh sách bản đồ cho webserver.")
        #         else:
        #             print("Không tìm thấy module webserver hoặc hàm get_available_maps để cập nhật danh sách bản đồ.")

        #     self.is_saving_map_mode = False
        #     cv2.destroyWindow(self.save_map_window_name)
        #     self.current_map_name_input = ""
        # else:
        #     print("Tên bản đồ không được để trống.")
        #     # Có thể hiển thị thông báo lỗi trên cửa sổ "Save Map" (cần vẽ thêm)

        pass
    def save_current_map(self, map_name, save_path_dir, map_all):
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
    def save_mask_map(self, map_name, save_path_dir, mask_map_array):
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
    def save_point_cloud(self, map_name, save_path_dir, point_cloud):
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
    def display_save_map_window(self):
        if not self.is_saving_map_mode:
            return

        img_save_dialog = np.full((230, 400, 3), (220, 220, 220), np.uint8) # Nền xám nhạt
        
        cv2.putText(img_save_dialog, "Nhap ten ban do:", (self.input_rect[0], self.input_rect[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 1)
        cv2.rectangle(img_save_dialog, (self.input_rect[0], self.input_rect[1]), (self.input_rect[0] + self.input_rect[2], self.input_rect[1] + self.input_rect[3]), (255,255,255), -1)
        cv2.rectangle(img_save_dialog, (self.input_rect[0], self.input_rect[1]), (self.input_rect[0] + self.input_rect[2], self.input_rect[1] + self.input_rect[3]), (0,0,0), 1)
        cv2.putText(img_save_dialog, self.current_map_name_input, (self.input_rect[0] + 5, self.input_rect[1] + self.input_rect[3] - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 1)
        cv2.rectangle(img_save_dialog, (self.save_button_rect[0], self.save_button_rect[1]), (self.save_button_rect[0] + self.save_button_rect[2], self.save_button_rect[1] + self.save_button_rect[3]), (100,180,100), -1) # Nút màu xanh lá
        cv2.putText(img_save_dialog, "Luu", (self.save_button_rect[0] + 30, self.save_button_rect[1] + self.save_button_rect[3] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)
        cv2.rectangle(img_save_dialog, (self.save_button_rect[0], self.save_button_rect[1]), (self.save_button_rect[0] + self.save_button_rect[2], self.save_button_rect[1] + self.save_button_rect[3]), (0,0,0), 1)
        cv2.putText(img_save_dialog, "Nhan ENTER de luu, ESC de huy.", (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50,50,50), 1)
        cv2.imshow(self.save_map_window_name, img_save_dialog)

# Tên cửa sổ hiển thị
window_name = "image_all Image Display"

# Hàm để chạy Flask trong một luồng riêng
def run_flask():
    app.run(host=host, port=port, debug=False, use_reloader=False)

detect = support_main(window_size, window_size_all, scaling_factor, rmse1, rmse2)

if __name__ == "__main__":
    # Register the SIGINT handler in the main thread of main.py
    # Chạy Flask trong một luồng riêng
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    time.sleep(2)

    # Mở trang web trong trình duyệt mặc định
    webbrowser.open(f"http://{get_local_ip()}:{port}")


    # Vòng lặp chính
    while detect.connect_while:
        detect.main_loop()
