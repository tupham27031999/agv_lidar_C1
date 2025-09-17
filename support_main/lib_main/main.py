from support_main.lib_main import remove, load_data_csv, edit_csv_tab, convert
import numpy as np
import cv2, os, time
import threading
import path
from pynput import keyboard
from support_main import connect_lidar_sick, connect_lidar




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

host = "172.26.76.151"
port = "5000"

data_admin = edit_csv_tab.load_all_stt(path_admin)
for i in range(0,len(data_admin)):
    if len(data_admin[i]) > 1:
        if data_admin[i][0] == "loai_lidar":
            loai_lidar = data_admin[i][1]

