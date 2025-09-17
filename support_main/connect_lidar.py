import numpy as np
from rplidar import RPLidar
import time
import serial.tools.list_ports
import threading
import path
from support_main.lib_main import edit_csv_tab
import os



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
com = "COM4"
bau = 115200
data_admin = edit_csv_tab.load_all_stt(path_admin)
for i in range(0,len(data_admin)):
    if len(data_admin[i]) > 1:
        if data_admin[i][0] == "cong_lidar":
            com = data_admin[i][1]
            bau = int(float(data_admin[i][2]))


def get_com_ports():
    """
    Lấy danh sách các cổng COM hiện có.
    
    Returns:
    list: Danh sách các cổng COM.
    """
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def check_com_port(port_name):
    """
    Kiểm tra xem cổng COM đầu vào có tồn tại hay không.
    
    Parameters:
    port_name (str): Tên cổng COM cần kiểm tra.
    
    Returns:
    bool: True nếu cổng COM tồn tại, False nếu không tồn tại.
    """
    return port_name in get_com_ports()



class main_lidar:
    def __init__(self):
        self.com = com
        self.bau = bau
        self.lidar = ""
        self.connect_lidar = False
        try:
            self.lidar = RPLidar(self.com, baudrate=self.bau)
            self.connect_lidar = True
        except:
            self.connect_lidar = False
            self.load_data = 0
        
        self.scan = np.array([[0, 0, 0]])
        self.ouput = np.array([[0, 0, 0]])
        self.check_scan = 0
        self.load_data = 0
        self.close_lidar = 0
        self.time_close = time.time()

    def connect(self):
        if self.close_lidar == 0:
            if check_com_port(self.com):
                if self.connect_lidar == False:
                    try:
                        self.lidar = RPLidar(self.com, baudrate=self.bau)
                        self.connect_lidar = True
                    except:
                        self.connect_lidar = False
                        self.load_data = 0
            if self.connect_lidar == True:
                if self.load_data == 0:
                    self.load_data = 1
                    threading.Thread(target=self.load_data_lidar).start()
    def check_close(self):
        if time.time() - self.time_close > 10:
            self.close_lidar = 1
            self.connect_lidar = False
    def disconnect(self):
        if self.connect_lidar == True:
            self.close_lidar = 1
            self.connect_lidar = False
    def load_data_lidar(self):
        if self.connect_lidar == True:
            try:
                for i, scan0 in enumerate(self.lidar.iter_scans()):
                    if len(scan0) > 0 and self.check_scan == 0:
                        self.upload_scan(scan0)
                    self.check_close()
                    if self.close_lidar == 1:
                        self.lidar.stop()
                        self.lidar.stop_motor()
                        self.lidar.disconnect()
                        break
            except:
                print("RPLidar exception:")
                self.connect_lidar = False
    def return_data(self):
        check = False
        if self.check_scan == 1:
            self.check_scan = 0
            self.ouput = self.scan
            check = True
        return self.ouput, check

    def upload_scan(self,data):
        self.scan = data
        self.check_scan = 1

# # Ví dụ sử dụng
if __name__ == "__main__":
    scan_lidar = main_lidar()
    scan_lidar.connect()

#     # Xử lý dữ liệu trong thread chính
    while True:
        data, check = scan_lidar.return_data()
        if data is not None:
            # Xử lý dữ liệu từ LIDAR
            print(data)