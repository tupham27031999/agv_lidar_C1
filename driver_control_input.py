import app_web
from support_main import crop_img_Atar, tim_duong_di, connect_driver
import numpy as np
import path, os
from support_main.lib_main import edit_csv_tab
import scan_an_toan
import process_lidar
import ket_noi_esp
import threading

path_phan_mem = path.path_phan_mem

distan_scan_all = [0, 10000]
dis_min = 0
dis_max = 2000
anpha_scan = [130,200]
anpha_scan_an_toan_tien = [0,70,280,360]
anpha_scan_an_toan_re_phai = [50, 130, 50, 130]
anpha_scan_an_toan_re_trai =[200, 300, 200, 300]
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

for i in range(0,len(data_admin)):
    if len(data_admin[i]) > 1:
        if data_admin[i][0] == "khoang_cach_duong_di":
            khoang_cach_duong_di = int(float(data_admin[i][1]))
        if data_admin[i][0] == "khoang_cach_dich":
            khoang_cach_dich = int(float(data_admin[i][1]))
        if data_admin[i][0] == "anpha_scan_an_toan_tien":
            anpha_scan_an_toan_tien = [int(float(data_admin[i][1])), int(float(data_admin[i][2])),int(float(data_admin[i][3])), int(float(data_admin[i][4]))] 
        if data_admin[i][0] == "anpha_scan_an_toan_re_trai":
            anpha_scan_an_toan_re_trai = [int(float(data_admin[i][1])), int(float(data_admin[i][2])),int(float(data_admin[i][3])), int(float(data_admin[i][4]))] 
        if data_admin[i][0] == "anpha_scan_an_toan_re_phai":
            anpha_scan_an_toan_re_phai = [int(float(data_admin[i][1])), int(float(data_admin[i][2])),int(float(data_admin[i][3])), int(float(data_admin[i][4]))] 
        if data_admin[i][0] == "anpha_scan":
            anpha_scan = [int(float(data_admin[i][1])), int(float(data_admin[i][2]))] 
        if data_admin[i][0] == "khoang_canh_an_toan_tien":
            khoang_canh_an_toan_tien = [int(float(data_admin[i][1])), int(float(data_admin[i][2]))] 
        if data_admin[i][0] == "khoang_cach_tim_duoi_di":
            khoang_cach_tim_duoi_di = [int(float(data_admin[i][1])), int(float(data_admin[i][2]))] 
        if data_admin[i][0] == "khoang_cach_an_toan_re":
            khoang_cach_an_toan_re = int(float(data_admin[i][1]))
        if data_admin[i][0] == "alpha_star_scan_trai":
            alpha_star_scan_trai = [int(float(data_admin[i][1])), int(float(data_admin[i][2])),int(float(data_admin[i][3])), int(float(data_admin[i][4]))] 
        if data_admin[i][0] == "alpha_star_scan_phai":
            alpha_star_scan_phai = [int(float(data_admin[i][1])), int(float(data_admin[i][2])),int(float(data_admin[i][3])), int(float(data_admin[i][4]))] 
        if data_admin[i][0] == "khoang_cach_astar_scan":
            khoang_cach_astar_scan = int(float(data_admin[i][1]))
        if data_admin[i][0] == "distan_scan_all":
            distan_scan_all = [int(float(data_admin[i][1])), int(float(data_admin[i][2]))] 
        if data_admin[i][0] == "dis_min":
            dis_min = int(float(data_admin[i][1]))
        if data_admin[i][0] == "dis_max":
            dis_max = int(float(data_admin[i][1]))
        if data_admin[i][0] == "khoang_cach_dich_min":
            khoang_cach_dich_min = int(float(data_admin[i][1]))
        if data_admin[i][0] == "khoang_cach_dich_max":
            khoang_cach_dich_max = int(float(data_admin[i][1]))


driver_motor_check = 0

class detect_data_sent_driver:
    def __init__(self, load_data_esp = 1):
        self.map_all = np.full((5000, 5000, 4), (150, 150, 150, 0), np.uint8)
        self.img1 = self.map_all.copy()

        self.khoang_canh_an_toan_tien = khoang_canh_an_toan_tien

        if driver_motor_check == 1:
            self.driver_motor = connect_driver.sent_data_driver()
        else:
            self.driver_motor = ""
        
        self.scan_vat_can = scan_an_toan.kiem_tra_vat_can()
        self.load_data_driver_motor = {}
        self.convert_data_run_agv0 = {"run_diem_2": "NG", "run_huong": "NG", "run_tin_hieu": "NG", "run_tin_hieu_tam_thoi": "NG"}
        if load_data_esp == 1:
            try:
                pass
                threading.Thread(target=ket_noi_esp.python_esp32).start() ######`##################
            except OSError as e:
                print("khong ket noi duoc esp")
                pass
        self.input_esp = ket_noi_esp.input_esp
        self.connect_esp32 = ket_noi_esp.check_connect_esp
        ########################################################################################
        self.v_tien_max = 0                 # load_data_web - ok
        self.v_re_max = 0                   # load_data_web - ok
        self.point_start_LQR = []           # load_data_process - ok
        self.point_end_LQR = []             # xu_ly_tin_hieu - ok
        self.angle = 0                      # xu_ly_tin_hieu - ok
        self.distance = 0                   # xu_ly_tin_hieu - ok
        self.check_angle_distance = 0       # xu_ly_tin_hieu - ok
        self.stop_sent_driver = 0           # xu_ly_tin_hieu - ok
        self.di_cham = 0                    # load_data_process - ok
        self.a_v = 800
        self.dang_re = 0                    # xu_ly_tin_hieu - ok
        self.tien_rl = 200
        
        self.diem_dau = [] # dùng để tạo self.point_old
        self.diem_dich = [] # dùng để tạo self.point_old, kiểm tra vị trí điểm 1 so với point_old
        # Format: danh_sach_diem = {"tên điểm": [tọa độ x, tọa độ y, "loại điểm", góc agv]}
        self.danh_sach_diem = {}
        # Format: danh_sach_duong = {"tên đường": ["tên điểm 1", "tên điểm 2"]}
        self.done_tin_hieu_tam_thoi = 0
        self.danh_sach_duong = {}
        self.robot_direction = []
        self.point_old = []

        self.stop_rmse = 0
        self.stop_vat_can = 0
        self.stop = 0
        self.name_music = "none"

        self.check_tien = 1
        self.check_trai = 0
        self.check_phai = 0

        self.run_auto_controller = 0

        self.convert_data_run_agv = self.convert_data_run_agv0.copy()

        self.distance_old = 1000
        self.dk_agv_thu_cong = 0
        self.data_dk_tay = ""



        
    def reset_data(self):
        pass

    def void_loop(self):
        if self.run_auto_controller == 1:
            self.load_data_process() # load các giá trị tọa độ góc của agv, check an toàn
            self.xu_ly_tin_hieu() # load các giá trị điểm đầu điểm đích, ... từ web, sau đó tính các giá trị góc, khoảng cách yêu cầu
            if driver_motor_check == 1 and self.driver_motor.connect == True: # điều khiển động cơ
                if self.data_dk_tay != "":
                    self.driver_motor.dk_agv_thu_cong = self.dk_agv_thu_cong
                    self.dk_ban_phim(self.data_dk_tay)
                else:
                    self.data_sent_drive() # gửi các giá trị tính toán, khai báo qua driver motor

    def convert_tin_hieu(self, tin_hieu_str, danh_sach_diem, danh_sach_duong):
        """
        Kiểm tra và tách chuỗi tín hiệu AGV.

        Args:
            tin_hieu_str (str): Chuỗi tín hiệu đầu vào.
                                Ví dụ: "P1-P3-VT_5000-VR_500-T_IN1.1-TT_IN2.1"
            danh_sach_diem (dict): Dictionary chứa thông tin các điểm.
                                Ví dụ: {"P1": [100, 200, "không hướng", 0], ...}
            danh_sach_duong (dict): Dictionary chứa thông tin các đường đi.
                                    Ví dụ: {"P1_P2": ["P1", "P2"], ...}

        Returns:
            dict: Dictionary chứa thông tin đã được xử lý.
        """
        data_all = {
            "tín hiệu hợp lệ": "True", # Mặc định là True, sẽ đổi thành False nếu cần
            "điểm 1": "None",
            "điểm 2": "None",
            "góc điểm 2": ["không hướng", 0],
            "tiến max": "None",
            "rẽ max": "None",
            "tín hiệu": [],
            "tín hiệu tạm thời": []
        }
        parts = tin_hieu_str.split('-')

        if len(parts) < 2:
            data_all["tín hiệu hợp lệ"] = "False" # Không đủ thông tin điểm
            return data_all # Không đủ thông tin điểm

        diem1_str = parts[0]
        diem2_str = parts[1]

        # Kiểm tra sự tồn tại của điểm 1 và điểm 2
        if diem1_str in danh_sach_diem:
            data_all["điểm 1"] = diem1_str
        else:
            data_all["tín hiệu hợp lệ"] = "False" # Điểm 1 không hợp lệ

        if diem2_str in danh_sach_diem:
            data_all["điểm 2"] = diem2_str
            # Lấy thông tin góc của điểm 2
            data_all["góc điểm 2"] = [danh_sach_diem[diem2_str][2], danh_sach_diem[diem2_str][3]]
        else:
            data_all["tín hiệu hợp lệ"] = "False" # Điểm 2 không hợp lệ

        # Nếu một trong các điểm không hợp lệ, thì tín hiệu tổng thể không hợp lệ
        # nhưng vẫn tiếp tục parse các phần khác
        if data_all["điểm 1"] == "None" or data_all["điểm 2"] == "None":
            data_all["tín hiệu hợp lệ"] = "False"


        # Kiểm tra đường đi giữa điểm 1 và điểm 2
        # Chỉ thực hiện kiểm tra đường đi nếu cả hai điểm đều hợp lệ
        if data_all["điểm 1"] != "None" and data_all["điểm 2"] != "None":
            duong_truc_tiep = f"{diem1_str}_{diem2_str}"
            duong_nguoc_lai = f"{diem2_str}_{diem1_str}"
            if not (duong_truc_tiep in danh_sach_duong or duong_nguoc_lai in danh_sach_duong):
                data_all["tín hiệu hợp lệ"] = "False"
        else: # Nếu điểm không hợp lệ, đường đi chắc chắn không hợp lệ
            data_all["tín hiệu hợp lệ"] = "False"


        # Xử lý các phần còn lại của tín hiệu
        for part in parts[2:]:
            if part.startswith("VT_"):
                try:
                    data_all["tiến max"] = part.split('_')[1]
                except IndexError:
                    pass # Bỏ qua nếu định dạng sai
            elif part.startswith("VR_"):
                try:
                    data_all["rẽ max"] = part.split('_')[1]
                except IndexError:
                    pass # Bỏ qua nếu định dạng sai
            elif part.startswith("T_"):
                try:
                    signals_str = part.split('_')[1:] # Bỏ "T"
                    for sig_pair_str in signals_str:
                        sig_parts = sig_pair_str.split('.')
                        if len(sig_parts) == 2:
                            data_all["tín hiệu"].append([sig_parts[0], sig_parts[1]])
                except IndexError:
                    pass # Bỏ qua nếu định dạng sai
            elif part.startswith("TT_"):
                try:
                    signals_str = part.split('_')[1:] # Bỏ "TT"
                    for sig_pair_str in signals_str:
                        sig_parts = sig_pair_str.split('.')
                        if len(sig_parts) == 2:
                            data_all["tín hiệu tạm thời"].append([sig_parts[0], sig_parts[1]])
                except IndexError:
                    pass # Bỏ qua nếu định dạng sai
        return data_all

    def kiem_tra_tin_hieu_esp32(self, data):
        self.input_esp = ket_noi_esp.input_esp
        self.connect_esp32 = ket_noi_esp.check_connect_esp

        """
        Kiểm tra xem tất cả các điều kiện trong check_data có khớp với input_esp không.

        Args:
            check_data (list): Danh sách các điều kiện cần kiểm tra.
                            Mỗi điều kiện là một list con dạng [key, value_mong_muon].
                            Ví dụ: [['IN4', 0], ['IN3', 1]]
            input_esp (dict): Dictionary chứa trạng thái của các input.
                            Ví dụ: {"IN1":0, "IN2":0, ..., "IN12":0}

        Returns:
            bool: True nếu tất cả các điều kiện trong check_data khớp với input_esp,
                ngược lại là False.
        """
        if not data:
            # Nếu không có điều kiện nào để kiểm tra, mặc định là True.
            # Bạn có thể thay đổi hành vi này nếu cần (ví dụ: trả về False).
            return True
        if self.connect_esp32 == False:
            # Nếu không kết nối được với esp32, coi như không khớp.
            return False

        for dieu_kien in data:
            # Đảm bảo mỗi điều kiện là một cặp [key, value]
            if not isinstance(dieu_kien, list) or len(dieu_kien) != 2:
                # Điều kiện không hợp lệ, coi như không khớp.
                # Hoặc bạn có thể raise một Exception ở đây nếu muốn.
                return False
            
            key_can_kiem_tra = dieu_kien[0]
            gia_tri_mong_muon = dieu_kien[1]

            # 1. Kiểm tra xem key có tồn tại trong input_esp không
            if key_can_kiem_tra not in self.input_esp:
                return False  # Key không tồn tại, điều kiện không khớp

            # 2. Kiểm tra xem giá trị có khớp không
            # Giả định rằng kiểu dữ liệu của giá trị mong muốn và giá trị trong input_esp là tương thích để so sánh.
            if self.input_esp[key_can_kiem_tra] != gia_tri_mong_muon:
                return False  # Giá trị không khớp

        # Nếu tất cả các điều kiện trong check_data đều được kiểm tra và khớp
        return True
    def load_data_web(self):
        # tin_hieu_nhan = app_web.tin_hieu_nhan
        tin_hieu_nhan = ""
        dict_cai_dat = app_web.dict_cai_dat
        # self.danh_sach_diem = app_web.danh_sach_diem
        # self.danh_sach_duong = app_web.danh_sach_duong
        self.danh_sach_diem = {}
        self.danh_sach_duong = {}
        self.v_tien_max = dict_cai_dat["van_toc_tien_max"]
        self.v_re_max = dict_cai_dat["van_toc_re_max"]
        return tin_hieu_nhan
    
    def xu_ly_tin_hieu(self):
        # {"run_diem_2": "NG", "run_huong": "NG", "run_tin_hieu": "NG", "run_tin_hieu_tam_thoi": "NG"}
        
        # data_all = {
        #     "tín hiệu hợp lệ": "True", # Mặc định là True, sẽ đổi thành False nếu cần
        #     "điểm 1": "None",
        #     "điểm 2": "None",
        #     "góc điểm 2": ["không hướng", 0],
        #     "tiến max": "None",
        #     "rẽ max": "None",
        #     "tín hiệu": [],
        #     "tín hiệu tạm thời": []
        # }

        tin_hieu_nhan = self.load_data_web()
        data_tin_hieu_nhan =  self.convert_tin_hieu(tin_hieu_nhan, self.danh_sach_diem, self.danh_sach_duong)
        tien_max = data_tin_hieu_nhan["tiến max"]
        re_max = data_tin_hieu_nhan["rẽ max"]
        diem_dau = data_tin_hieu_nhan["điểm 1"]
        diem_dich = data_tin_hieu_nhan["điểm 2"]
        goc_dich = data_tin_hieu_nhan["góc điểm 2"]
        tin_hieu_hop_le = data_tin_hieu_nhan["tín hiệu hợp lệ"]
        tin_hieu = data_tin_hieu_nhan["tín hiệu"]
        tin_hieu_tam_thoi = data_tin_hieu_nhan["tín hiệu tạm thời"]
        
        if tin_hieu_hop_le == "False" or self.stop_rmse == 1 or self.stop_vat_can == 1:
            self.stop = 1
            self.stop_sent_driver = 1
        if self.stop == 1 or self.stop_sent_driver == 1:
            if self.name_music != "none":
                if self.name_music == "re_trai":
                    self.name_music = "none"
                if self.name_music == "re_phai":
                    self.name_music = "none"
        if self.stop == 0:
            
            self.stop_xu_ly_tin_hieu = 1
            if tien_max != "None":
                self.v_tien_max = int(float(tien_max))
            if re_max != "None":
                self.v_re_max = int(float(re_max))

            if goc_dich[0] == "không hướng":
                self.convert_data_run_agv["run_huong"] = "OK"
            
            # kiem tra tin hieu
            if len(tin_hieu) != 0:
                if self.kiem_tra_tin_hieu_esp32(tin_hieu) == True:
                    tin_hieu = []
            if len(tin_hieu_tam_thoi) != 0:
                if self.kiem_tra_tin_hieu_esp32(tin_hieu_tam_thoi) == True:
                    self.done_tin_hieu_tam_thoi = 1

            if len(tin_hieu) == 0:
                self.convert_data_run_agv["run_tin_hieu"] = "OK"
            else:
                self.convert_data_run_agv["run_tin_hieu"] = "NG"
            if self.done_tin_hieu_tam_thoi == 1:
                self.convert_data_run_agv["run_tin_hieu_tam_thoi"] = "OK"

            
            if len(diem_dau) != 0:
                if len(self.diem_dau) != 0:
                    if diem_dau[0] != self.diem_dau[0] or diem_dau[1] != self.diem_dau[1]:
                        self.point_old = self.diem_dau
            self.diem_dau = diem_dau
            self.diem_dich = diem_dich

            # xử lý tín hiệu web truyền tới
            if len(self.diem_dau) != 0 and self.diem_dich != 0:
                point_end_LQR = self.diem_dich
                goc_quay_check = 0
                distance = 0
                check_angle_distance = "distance"
                if self.convert_data_run_agv["run_diem_2"] != "OK":
                    # load điểm gần agv để đi tới
                    check, distance, angle_deg = tim_duong_di.calculate_distance_and_angle(self.diem_dau, self.diem_dich, self.robot_direction)
                    if distance > 80:
                        img_Astar, max_point_Astar, x_min_Astar, y_min_Astar, x_max_Astar, y_max_Astar = crop_img_Atar.img_crop(self.img1.copy(), 
                                                                                            self.diem_dau.copy(), self.diem_dich.copy(), distance = 80)
                        point_end_LQR = [max_point_Astar[0],max_point_Astar[1]]

                    # xác định khoảng cách và góc điểm đích
                    # check, distance, angle_deg = tim_duong_di.calculate_distance_and_angle(self.diem_dau, self.diem_dich, self.robot_direction)
                    check_kc, distance_kc, angle_deg_kc = tim_duong_di.calculate_distance_and_angle(self.diem_dau, self.point_old, self.robot_direction)

                    check_line, distance_line, angle_deg_line = tim_duong_di.calculate_distance_and_angle(self.diem_dich, self.point_old, self.diem_dau)
                    # goc bám sát đường đi
                    if angle_deg_line > 90:
                        angle_deg_line = 180 - angle_deg_line
                    if angle_deg_line < -90:
                        angle_deg_line = -180 - angle_deg_line
                    # goc quay yêu cầu
                    goc_quay_check = angle_deg - angle_deg_line

                    # thay đổi quét vật cản khi vật đi chậm và gần đích
                    vt_check_kc = max(self.driver_motor.vt_phai * 10, self.driver_motor.vt_trai * 10)
                    number_kc = 0
                    if vt_check_kc <= 2000 and (distance < 50 or distance_kc < 50):
                    # if vt_check_kc <= 2000 and (distance < 50):
                        a = int(vt_check_kc / 100)
                        number_kc = 20 - a
                        if number_kc > 15:
                            number_kc = 15
                    self.khoang_canh_an_toan_tien = [khoang_canh_an_toan_tien[0], khoang_canh_an_toan_tien[1] - number_kc]

                    # agv rẽ trái hoặc phải
                    if abs(angle_deg) > 30:
                        self.dang_re = 1
                    if abs(angle_deg) < 12 and self.dang_re == 1:
                        self.dang_re = 0
                    if on_music == 1:
                        # music
                        if angle_deg < -30:
                            self.name_music = "re_phai"
                        else:
                            if self.name_music == "re_phai":
                                self.name_music = "none"

                        if angle_deg > 30:
                            self.name_music = "re_trai"
                        else:
                            if self.name_music == "re_trai":
                                self.name_music = "none"

                    # lựa chọn khoảng cách đến đích
                    khoang_cach_dich = khoang_cach_dich_min
                    if self.convert_data_run_agv["run_huong"] == "OK":
                        khoang_cach_dich = khoang_cach_dich_max
                    # kiểm tra điều kiện đã đến đích
                    if (distance <= 2 or distance <= khoang_cach_dich  or (distance > self.distance_old and self.distance_old <= khoang_cach_dich)):
                        self.convert_data_run_agv["run_diem_2"] = "OK"
                    if distance < self.distance_old:
                        self.distance_old = distance
                else:
                    if self.convert_data_run_agv["run_huong"] != "OK":
                        check_angle_distance = "angle"
                        start_point = self.point_start_LQR
                        robot_direction = self.robot_direction
                        A = np.array(diem_dich)
                        C = np.array(start_point)
                        B = np.array(robot_direction)
                        # Tính vector AC
                        AC = C - A
                        # Tịnh tiến điểm B theo vector AC
                        end_point_angle = B + AC
                        check_ang, distance_ang, angle_deg = tim_duong_di.calculate_distance_and_angle(start_point, end_point_angle, robot_direction)

                        goc_quay_check = angle_deg
                        self.dang_re = 1

                        # music
                        if on_music == 1:
                            if angle_deg < 0:
                                self.name_music = "re_phai"
                            else:
                                if self.name_music == "re_phai":
                                    self.name_music = "none"

                            if angle_deg > 0:
                                self.name_music = "re_trai"
                            else:
                                if self.name_music == "re_trai":
                                    self.name_music = "none"

                self.point_end_LQR = point_end_LQR
                self.check_angle_distance = check_angle_distance
                self.angle = goc_quay_check
                self.distance = distance
            else:
                self.stop_sent_driver = 1

    def kiem_tra_dich_den(self):
        pass
    def load_data_process(self):
        sent_data_driver_motor = self.load_data_driver_motor
        # điểm 1 lần đầu (nếu là rỗng) scan
        if len(self.diem_dau) == 0:
            self.diem_dau = [sent_data_driver_motor["tam_x_agv"], sent_data_driver_motor["tam_y_agv"]]
            self.point_old = self.diem_dau

        self.point_start_LQR = [sent_data_driver_motor["tam_x_agv"], sent_data_driver_motor["tam_y_agv"]]
        self.goc_agv = sent_data_driver_motor["rotation"]
        self.robot_direction = [sent_data_driver_motor["huong_x"], sent_data_driver_motor["huong_y"]]
        self.img1 = sent_data_driver_motor["img1"]
        self.stop_rmse = sent_data_driver_motor["stop"]

        # load_du_lieu scan vat can
        self.check_an_toan = self.load_scan_vat_can(sent_data_driver_motor["scan"], 
                                                    self.check_tien, self.check_trai, self.check_phai,
                                                    sent_data_driver_motor["huong_agv"], 
                                                    sent_data_driver_motor["tam_x_agv"], sent_data_driver_motor["tam_y_agv"], 
                                                    sent_data_driver_motor["huong_x"], sent_data_driver_motor["huong_y"],
                                                    sent_data_driver_motor["scaling_factor"], 
                                                    sent_data_driver_motor["window_size_x_all"], sent_data_driver_motor["window_size_y_all"])
        if self.check_an_toan == "stop":
            self.stop_vat_can = 1
        else:
            self.stop_vat_can = 0
        if self.check_an_toan == "slow":
            self.di_cham = 1
        else:
            self.di_cham = 0
    def load_scan_vat_can(self, scan, check_tien, check_trai, check_phai, 
                          rotation, x_goc, y_goc, huong_x, huong_y, 
                          scaling_factor, window_size_x_all, window_size_y_all):
        output = self.scan_vat_can.detect(scan, check_tien, check_trai, check_phai, 
                                 rotation, x_goc, y_goc, huong_x, huong_y, 
                                 scaling_factor, window_size_x_all, window_size_y_all,
                                 self.khoang_canh_an_toan_tien, khoang_cach_an_toan_re, khoang_cach_tim_duoi_di)
        return output
    
    
    
    
    def data_sent_drive(self):
        self.driver_motor.load_data_sent_drive(self.v_tien_max, self.v_re_max, self.point_start_LQR, self.point_end_LQR,
                             self.goc_agv, self.angle, self.distance, self.check_angle_distance,
                             self.stop, self.di_cham, self.a_v, self.dang_re, self.tien_rl)
        # return v_tien_max,  v_re_max, point_start_LQR, point_end_LQR, \
        #     goc_agv, goc_dich, angle, distance, check_angle_distance, \
        #         stop = 0, check_an_toan = [], tim_duong = 0, a_v = 800, dang_re = 0, tien_rl = 200
    
    def dk_ban_phim(self,data_input, van_toc_max_tien):
        if data_input != "":
            if data_input == "stop":
                self.driver_motor.sent_data_controller(vt_trai = 0, vt_phai = 0)
            if data_input == "tien":
                self.driver_motor.sent_data_controller(vt_trai = van_toc_max_tien, vt_phai = van_toc_max_tien)
            if data_input == "trai":
                self.driver_motor.sent_data_controller(vt_trai = -int(van_toc_max_tien/4), vt_phai = int(van_toc_max_tien/4))
            if data_input == "phai":
                self.driver_motor.sent_data_controller(vt_trai = int(van_toc_max_tien/4), vt_phai = -int(van_toc_max_tien/4))
            if data_input == "dich_tien_trai":
                self.driver_motor.sent_data_controller(vt_trai = int((van_toc_max_tien/4)*3), vt_phai = van_toc_max_tien)
            if data_input == "dich_tien_phai":
                self.driver_motor.sent_data_controller(vt_trai = van_toc_max_tien, vt_phai = int((van_toc_max_tien/4)*3))

            if data_input == "lui":
                self.driver_motor.sent_data_controller(vt_trai = -van_toc_max_tien, vt_phai = -van_toc_max_tien)
            if data_input == "dich_lui_trai":
                self.driver_motor.sent_data_controller(vt_trai = -van_toc_max_tien, vt_phai = -(van_toc_max_tien/3))
            if data_input == "dich_lui_phai":
                self.driver_motor.sent_data_controller(vt_trai = -int(van_toc_max_tien/3), vt_phai = -int(van_toc_max_tien))
    

