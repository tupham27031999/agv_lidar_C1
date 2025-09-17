import numpy as np
from rplidar import RPLidar
import time
import serial.tools.list_ports
import threading
import path
from support_main.lib_main import edit_csv_tab
import os
import cv2
import serial



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
com = "COM5"
bau = 460800
data_admin = edit_csv_tab.load_all_stt(path_admin)
for i in range(0,len(data_admin)):
    if len(data_admin[i]) > 1:
        if data_admin[i][0] == "cong_lidar":
            com = data_admin[i][1]
            bau = int(float(data_admin[i][2]))

# Khóa để bảo vệ biến 
scan_lock = threading.Lock()

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
        self.ser = None
        self.lidar = ""
        self.connect = False

        try:
            self.ser = serial.Serial(self.com, self.bau, timeout=1)
            # STOP trước khi START
            self.ser.write(bytes([0xA5, 0x25]))
            time.sleep(0.05)
            # START
            self.ser.write(bytes([0xA5, 0x20]))
            time.sleep(0.05)

            # Đồng bộ đầu gói
            self._wait_sync()
            desc = self.ser.read(7)
            print("Mô tả:", desc.hex())

            self.connect = True
            print("hhhhhhhhhhhhhhhhhhhhhhhhhhh")

        except Exception as e:
            print("Lỗi khi kết nối LIDAR:", e)
            self.connect = False
            self.load_data = 0
        
        self.scan = np.array([[0, 0, 0]])
        self.load_data = 0
        self.close_lidar = 0
        self.time_close = time.time()

        self.buffer = bytearray()
        print("Bắt đầu đọc dữ liệu (Ctrl+C để thoát)")

    def connect_lidar(self):
        self.time_close = time.time()
        if self.close_lidar == 0:
            if check_com_port(self.com):
                if self.connect == False:
                    try:
                        self.ser = serial.Serial(self.com, self.bau, timeout=1)
                        # STOP trước khi START
                        self.ser.write(bytes([0xA5, 0x25]))
                        time.sleep(0.05)
                        # START
                        self.ser.write(bytes([0xA5, 0x20]))
                        time.sleep(0.05)

                        # Đồng bộ đầu gói
                        self._wait_sync()
                        desc = self.ser.read(7)
                        print("Mô tả:", desc.hex())

                        self.connect = True

                    except Exception as e:
                        print("Lỗi khi kết nối LIDAR:", e)
                        self.connect = False
                        self.load_data = 0
                    # try:
                    #     self.lidar = RPLidar(self.com, baudrate=self.bau)
                    #     self.connect_lidar = True
                    # except:
                    #     self.connect_lidar = False
                    #     self.load_data = 0
            if self.connect == True:
                if self.load_data == 0:
                    self.load_data = 1
                    threading.Thread(target=self.load_data_lidar).start()
    def _wait_sync(self):
        sync = b''
        while True:
            b = self.ser.read(1)
            if not b:
                continue
            sync += b
            if len(sync) > 2:
                sync = sync[-2:]
            if sync == b'\xA5\x5A':
                print("Đã tìm thấy sync header")
                return
    def _parse_point(self, data):
        if len(data) != 5:
            return None
        b = data
        quality = b[0] >> 2
        check_bit = (b[0] & 0x01)
        start_bit = (b[0] & 0x02) >> 1
        angle = ((b[1] >> 1) | (b[2] << 7)) / 64.0
        distance = ((b[3]) | (b[4] << 8)) / 4.0
        return angle, distance, quality, start_bit, check_bit
    def check_close(self):
        if time.time() - self.time_close > 10:
            self.close_lidar = 1
            self.connect = False
    def disconnect(self):
        if self.connect == True:
            self.close_lidar = 1
            self.connect = False
            time.sleep(1)
            try:
                # STOP
                self.ser.write(bytes([0xA5, 0x25]))
                time.sleep(0.05)
                print("disconnect")
                self.ser.close()
            except:
                pass
            
    def load_data_lidar(self):
        if self.connect == True:
            # Sử dụng biến cục bộ cho luồng này để code rõ ràng hơn
            local_scan_buffer = []
            max_ang_local = 0.0
            try:
                while self.close_lidar == 0:
                    self.buffer += self.ser.read(512)
                    while len(self.buffer) >= 5:
                        point = self._parse_point(self.buffer[:5])
                        self.buffer = self.buffer[5:]
                        if point:
                            angle, distance, quality, start_bit, check_bit = point
                            if 0 <= angle <= 360 and distance > 0:
                                # Khi góc quay về giá trị nhỏ hơn, tức là đã hoàn thành một vòng quét
                                if angle < max_ang_local:
                                    if local_scan_buffer:
                                        # Cập nhật dữ liệu scan dùng chung một cách an toàn
                                        self._update_scan(np.array(local_scan_buffer))
                                    # Reset bộ đệm cục bộ cho vòng quét tiếp theo
                                    local_scan_buffer = []
                                    max_ang_local = 0.0

                                max_ang_local = max(max_ang_local, angle)
                                local_scan_buffer.append([quality, angle, distance])
            except Exception as e:
                print("Lỗi đọc dữ liệu:", e)

    def _update_scan(self, new_scan_data):
        """Phương thức private để cập nhật an toàn biến scan được chia sẻ."""
        with scan_lock:
            self.scan = new_scan_data

    def return_data(self):
        """Phương thức public để lấy ra một bản sao của dữ liệu scan mới nhất."""
        with scan_lock:
            scan_copy = self.scan.copy()
        is_data_valid = scan_copy.shape[0] > 1
        return scan_copy, is_data_valid

# # Ví dụ sử dụng
if __name__ == "__main__":
    scan_lidar = main_lidar()
    img_size = 700
    img0 = np.zeros((img_size, img_size, 3), dtype=np.uint8)
    
    scale = 0.1
    import math

#     # Xử lý dữ liệu trong thread chính
    while True:
        img = img0.copy()
        scan_lidar.connect()
        data_all, check = scan_lidar.return_data()
        # print(scan_lidar.connect_lidar)
        # if data_all is not None:
        #     # Xử lý dữ liệu từ LIDAR
        #     print(data)

        center = img_size // 2
        for data in data_all:
            x = int(center + (data[1] * math.cos(math.radians(data[0]))) * scale)
            y = int(center - (data[1] * math.sin(math.radians(data[0]))) * scale)
            if 0 <= x < img_size and 0 <= y < img_size:
                cv2.circle(img, (x, y), 1, (0, 255, 0), -1)
        cv2.circle(img, (center, center), 3, (0, 0, 255), -1)

        cv2.imshow("Lidar Scan", img)
        key = cv2.waitKey(1)
        if key == 27 or cv2.getWindowProperty('Lidar Scan', cv2.WND_PROP_VISIBLE) < 1:
            scan_lidar.disconnect()
            break
    cv2.destroyAllWindows()