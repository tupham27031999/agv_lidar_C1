import socket
import time
import path
from support_main.lib_main import edit_csv_tab
import numpy as np
import pyperclip
import struct
import os
# pyperclip.copy('The text to be copied to the clipboard.')
# import pyqtgraph as pg
from PyQt6 import QtCore, QtGui, QtWidgets


UDP_IP = "192.168.1.10" # Nhận từ tất cả các địa chỉ IP
UDP_PORT = 2368

path_phan_mem = path.path_phan_mem
path_admin = path_phan_mem + "/setting/admin_window.csv"
if os.name == "nt":
    print("Hệ điều hành là Windows")
    # Đọc file cài đặt cho Windows
    path_admin = path_phan_mem + "/setting/admin_window.csv"
elif os.name == "posix":
    print("Hệ điều hành là Ubuntu (Linux)")
    # Đọc file cài đặt cho Ubuntu
    path_admin = path_phan_mem + "/setting/admin_ubuntu.csv"
data_admin = edit_csv_tab.load_all_stt(path_admin)


for i in range(0,len(data_admin)):
    if len(data_admin[i]) > 1:
        if data_admin[i][0] == "host_lidar":
            host_lidar = data_admin[i][1]
        if data_admin[i][0] == "port_lidar":
            port_lidar = int(float(data_admin[i][1]))
print("host_lidar", host_lidar)
print("port_lidar", port_lidar, port_lidar == 2368)
UDP_IP = host_lidar
UDP_PORT = port_lidar


class LidarP:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.timer = QtCore.QTimer()
        self.final_data = np.array([[0, 0, 0]])
        self.final_data_old = np.array([[0, 0, 0]])
        self.final_data_new = []
        self.connect = True
        self.data_ok = 0
        self.setup()
        self.start = False


    def setup(self):
        self.sock.bind((UDP_IP, UDP_PORT))
        self.timer.start(40)

    def decode_data(self, data):
        """Giải mã dữ liệu từ LR-1BS3/5."""
        if len(data) != 8:
            raise ValueError("Dữ liệu đầu vào phải là 8 byte.")
        
        angle_raw, distance_raw, signal_raw, _ = struct.unpack("<HHHH", data)
        angle = angle_raw * 0.01
        distance = distance_raw  # Giả sử tỷ lệ khoảng cách là 1mm
        signal = signal_raw
        return angle, distance, signal, _
    def get_data(self):
        output = self.final_data_old
        check = False 
        if self.data_ok == 1:
            output = self.final_data
            self.final_data_old = self.final_data
            self.data_ok = 0
            check = True
        return output, check
    def process_data(self):
        while self.connect:
            try: 
                data, addr = self.sock.recvfrom(1240)
            except socket.timeout:
                self.connect = False
                print("connect False")
                break
            body = data[40:]
            data_array = []

            # Chuyển toàn bộ body (1200 bytes) thành mảng NumPy
            data0 = np.frombuffer(body, dtype=np.uint16).reshape(-1, 4)

            data = data0[data0[:, 1] != 0]

            # Giải mã dữ liệu
            angles0 = data0[:, 0] * 0.01  # Góc quét (angle)
            angles = data[:, 0] * 0.01  # Góc quét (angle)
            # print(angles)
            distances = data[:, 1]      # Khoảng cách đo được (distance)
            signals = data[:, 2]        # Cường độ tín hiệu (signal)

            data_array = np.column_stack((signals, angles, distances))

            if int(angles0[0]) == 0:
                if self.data_ok == 0:
                    self.final_data = self.final_data_new  # Ensure it's a NumPy array
                    self.data_ok = 1
                self.final_data_new = []
            else:
                self.final_data_new = [*self.final_data_new, *data_array]
