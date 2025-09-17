import numpy as np
import path
from support_main.lib_main import edit_csv_tab, remove
import os
import math

path_phan_mem = path.path_phan_mem

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
        if data_admin[i][0] == "anpha_scan_an_toan_tien":
            anpha_scan_an_toan_tien = [int(float(data_admin[i][1])), int(float(data_admin[i][2])),int(float(data_admin[i][3])), int(float(data_admin[i][4]))] 
        if data_admin[i][0] == "anpha_scan_an_toan_re_trai":
            anpha_scan_an_toan_re_trai = [int(float(data_admin[i][1])), int(float(data_admin[i][2])),int(float(data_admin[i][3])), int(float(data_admin[i][4]))] 
        if data_admin[i][0] == "anpha_scan_an_toan_re_phai":
            anpha_scan_an_toan_re_phai = [int(float(data_admin[i][1])), int(float(data_admin[i][2])),int(float(data_admin[i][3])), int(float(data_admin[i][4]))] 


class kiem_tra_vat_can:
    def __init__(self):
        self.scan_an_toan_tien = np.array([[0], [0], [0]])
        self.scan_an_toan_re_trai = np.array([[0], [0], [0]])
        self.scan_an_toan_re_phai = np.array([[0], [0], [0]])
        self.closest_point = []


    def detect(self, scan, check_tien, check_trai, check_phai, rotation, x_goc, y_goc, huong_x, huong_y, 
               scaling_factor, window_size_x_all, window_size_y_all, 
               khoang_canh_an_toan_tien, khoang_cach_an_toan_re, khoang_cach_tim_duoi_di):
        output = "none"
        closest_point_0 = []
        closest_point_1 = []
        closest_point_2 = []

        scan_1 = scan[(scan[:, 2] < 1500)]
        scan_2 = scan_1[(scan_1[:, 2] < 500)]
        if check_tien == 1:
            self.scan_an_toan_tien = scan[(((scan_1[:, 1] >= anpha_scan_an_toan_tien[0]) & (scan_1[:, 1] <= anpha_scan_an_toan_tien[1])) | \
                                        ((scan_1[:, 1] >= anpha_scan_an_toan_tien[2]) & (scan_1[:, 1] <= anpha_scan_an_toan_tien[3])))]
            
            px_0_tien = np.array(np.cos(self.scan_an_toan_tien[:, 1] / 180 * math.pi - rotation) * self.scan_an_toan_tien[:, 2] * scaling_factor + x_goc)
            py_0_tien = np.array(np.sin(self.scan_an_toan_tien[:, 1] / 180 * math.pi - rotation) * self.scan_an_toan_tien[:, 2] * scaling_factor + y_goc)
            mask0_tien = (px_0_tien > 0) & (px_0_tien < window_size_x_all) & (py_0_tien > 0) & (py_0_tien < window_size_y_all)
            px0_tien = px_0_tien[mask0_tien]
            py0_tien = py_0_tien[mask0_tien]

            closest_point_0, closest_point_1, closest_point_2 = self.callback_tien(x_goc, y_goc, px0_tien, py0_tien, 
                                                    khoang_canh_an_toan_tien,[khoang_cach_tim_duoi_di[0],khoang_cach_tim_duoi_di[1]+20],khoang_cach_tim_duoi_di,
                                                    huong_x,huong_y)

        if check_trai == 1:
            self.scan_an_toan_re_trai = scan_2[(((scan_2[:, 1] > anpha_scan_an_toan_re_trai[0]) & (scan_2[:, 1] < anpha_scan_an_toan_re_trai[1])) | \
                                                ((scan_2[:, 1] > anpha_scan_an_toan_re_trai[2]) & (scan_2[:, 1] < anpha_scan_an_toan_re_trai[3])))]
            
            px_0_re_trai = np.array(np.cos(self.scan_an_toan_re_trai[:, 1] / 180 * math.pi - rotation) * self.scan_an_toan_re_trai[:, 2] * scaling_factor + x_goc)
            py_0_re_trai = np.array(np.sin(self.scan_an_toan_re_trai[:, 1] / 180 * math.pi - rotation) * self.scan_an_toan_re_trai[:, 2] * scaling_factor + y_goc)
            mask0_re_trai = (px_0_re_trai > 0) & (px_0_re_trai < window_size_x_all) & (py_0_re_trai > 0) & (py_0_re_trai < window_size_y_all)
            px0_re_trai = px_0_re_trai[mask0_re_trai]
            py0_re_trai = py_0_re_trai[mask0_re_trai]

            closest_point_0 = self.callback_re(x_goc, y_goc, px0_re_trai, py0_re_trai, khoang_cach_an_toan_re)
        if check_phai == 1:
            self.scan_an_toan_re_phai = scan_2[(((scan_2[:, 1] > anpha_scan_an_toan_re_phai[0]) & (scan_2[:, 1] < anpha_scan_an_toan_re_phai[1])) | \
                                                ((scan_2[:, 1] > anpha_scan_an_toan_re_phai[2]) & (scan_2[:, 1] < anpha_scan_an_toan_re_phai[3])))]
            
            px_0_re_phai = np.array(np.cos(self.scan_an_toan_re_phai[:, 1] / 180 * math.pi - rotation) * self.scan_an_toan_re_phai[:, 2] * scaling_factor + x_goc)
            py_0_re_phai = np.array(np.sin(self.scan_an_toan_re_phai[:, 1] / 180 * math.pi - rotation) * self.scan_an_toan_re_phai[:, 2] * scaling_factor + y_goc)
            mask0_re_phai = (px_0_re_phai > 0) & (px_0_re_phai < window_size_x_all) & (py_0_re_phai > 0) & (py_0_re_phai < window_size_y_all)
            px0_re_phai = px_0_re_phai[mask0_re_phai]
            py0_re_phai = py_0_re_phai[mask0_re_phai]

            closest_point_0 = self.callback_re(x_goc, y_goc, px0_re_phai, py0_re_phai, khoang_cach_an_toan_re)
        if len(closest_point_0) != 0:
            output = "stop"
        else:
            if len(closest_point_1) != 0:
                output = "slow"
        self.closest_point = closest_point_0 + closest_point_1
        return output


    def calculate_distance_and_angle(self, A, B, C, distan_can_vat_can_0,distan_can_vat_can_1,distan_can_vat_can_2, beta=90):
        output_0 = "OK"
        output_1 = "OK"
        output_2 = "OK"
        alpha_check_0 = math.atan(distan_can_vat_can_0[0]/distan_can_vat_can_0[1])*180/np.pi
        alpha_check_1 = math.atan(distan_can_vat_can_1[0]/distan_can_vat_can_1[1])*180/np.pi
        alpha_check_2 = math.atan(distan_can_vat_can_2[0]/distan_can_vat_can_2[1])*180/np.pi
        distance = 0
        # Tính vector AB và AC
        AB = np.array([B[0] - A[0], B[1] - A[1]])
        AC = np.array([C[0] - A[0], C[1] - A[1]])

        # Tính độ dài của vector AB và AC
        AB_length = np.linalg.norm(AB)
        AC_length = np.linalg.norm(AC)

        if A != B and A != C and B != C and C != [0,0]:
            # Tính góc alpha giữa AB và AC
            cos_alpha = np.dot(AB, AC) / (AB_length * AC_length)
            alpha = np.arccos(cos_alpha) * 180 / np.pi  # Chuyển đổi từ radian sang độ

            if abs(alpha) < abs(alpha_check_0):
                distance_check_0 = distan_can_vat_can_0[1]/np.cos(alpha*np.pi/180)
            else:
                distance_check_0 = distan_can_vat_can_0[0]/np.sin(alpha*np.pi/180)
            if abs(alpha) < abs(alpha_check_1):
                distance_check_1 = distan_can_vat_can_1[1]/np.cos(alpha*np.pi/180)
            else:
                distance_check_1 = distan_can_vat_can_1[0]/np.sin(alpha*np.pi/180)
            if abs(alpha) < abs(alpha_check_2):
                distance_check_2 = distan_can_vat_can_2[1]/np.cos(alpha*np.pi/180)
            else:
                distance_check_2 = distan_can_vat_can_2[0]/np.sin(alpha*np.pi/180)

            # print(distance_check_0, distance_check_1, distance_check_2, alpha_check_0,alpha_check_1, alpha_check_2, "------------")
            # Kiểm tra góc alpha có nằm trong dải [-55, 55] hay không
            if -beta <= alpha <= beta:
                # Tính khoảng cách AB * cos(alpha)
                distance = AB_length
                if abs(distance) < abs(distance_check_0):
                    output_0 = "NG"
                if abs(distance) < abs(distance_check_1):
                    output_1 = "NG"
                if abs(distance) < abs(distance_check_2):
                    output_2 = "NG"
        return output_0,output_1,output_2
    def callback_tien(self, x_goc, y_goc, px, py, distan_can_vat_can_0, distan_can_vat_can_1, distan_can_vat_can_2, huong_x, huong_y):
        A = [x_goc, y_goc]
        C = [huong_x,huong_y]
        point_vat_can_0 = []
        point_vat_can_1 = []
        point_vat_can_2 = []
        # print(distan_can_vat_can_0, distan_can_vat_can_1, distan_can_vat_can_2)
        for i in range(0, len(px)):
            B = [px[i], py[i]]
            kq_0,kq_1,kq_2 = self.calculate_distance_and_angle(A, B, C, distan_can_vat_can_0, distan_can_vat_can_1, distan_can_vat_can_2)
            if kq_0 == "NG":
                point_vat_can_0 = [px[i], py[i]]
            if kq_1 == "NG":
                point_vat_can_1 = [px[i], py[i]]
            if kq_2 == "NG":
                point_vat_can_2 = [px[i], py[i]]
        return point_vat_can_0, point_vat_can_1, point_vat_can_2
    def callback_re(self, x_goc, y_goc, px, py, distan_can_vat_can):
        min_dist = 1000
        closest_point = []
        point_vat_can = []
        for i in range(0, len(px)):
            dist = np.sqrt((px[i] - x_goc) ** 2 + (py[i] - y_goc) ** 2)
            if dist < min_dist:
                min_dist = dist
                closest_point = [px[i], py[i], min_dist, x_goc, y_goc]
        if len(closest_point) != 0:
            # Nếu khoảng cách nhỏ hơn 10 thì bật loa nói "có vật cản" trong một luồng riêng biệt
            if min_dist < distan_can_vat_can:
                point_vat_can = [closest_point[0], closest_point[1]]
        return point_vat_can