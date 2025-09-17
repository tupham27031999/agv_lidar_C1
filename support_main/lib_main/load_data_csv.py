import path
from support_main.lib_main import edit_csv_tab
import shutil
from tkinter.messagebox import showerror, showwarning, showinfo
import os

def load_file_csv(path_input,so_dong_min = 1):
    ds_input = []
    ten_input = []
    tt_input = []
    if os.path.exists(path_input) == False:
        showerror(title='Error',message='Thiếu file setting: ' + str(path_input))
    else:
        ds_input = edit_csv_tab.load_all_stt(path_input)
        ten_input = []
        tt_input = []
        if len(ds_input) >= so_dong_min:
            for i in range(0,len(ds_input)):
                if len(ds_input[i]) >= 1:
                    ten_input.append(ds_input[i][0])
                else:
                    ten_input.append("")
                if len(ds_input[i]) >= 2:
                    tt_input.append(ds_input[i][1:])
                else:
                    tt_input.append([""])
        else:
            showerror(title='Error',message='Kiểm tra lại file: ' + str(path_input))
        return ds_input,ten_input,tt_input

def giao_dien_goc():
    ds_giao_dien,ten_label,tt_label = load_file_csv(path.path_giao_dien,3)
    return ds_giao_dien,ten_label,tt_label
# print(giao_dien_goc()[2])



# load danh sach setting
def ds_admin():
    ds_admin,ten_admin,tt_admin = load_file_csv(path.path_admin)
    return ds_admin,ten_admin,tt_admin

# load danh sach setting
def ds_khung(path_data):
    ds_data,ten_data,tt_data = load_file_csv(path_data)
    return ds_data,ten_data,tt_data

# load danh sach setting
def ds_data(path_data):
    ds_data,ten_data,tt_data = load_file_csv(path_data)
    return ds_data,ten_data,tt_data




