import os
from tkinter.messagebox import showerror, showwarning, showinfo
import csv


# chuyen dia chi co dau \ sang /
def edit_path(input):
    new_path = ""
    for i in list(input):
        if i == str("\\"):
            new_path = new_path + "/"
        if i != str("\\"):
            new_path = new_path + i
    return new_path
path_phan_mem = edit_path(os.path.dirname(os.path.realpath(__file__)))
# print(path_phan_mem)

# tao file csv moi
def new_csv_no_replace(path_csv, in_p="",): # in_p: tieu de
    if os.path.exists(path_csv) == False:
        with open(path_csv, 'w',newline='',encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=in_p)
            writer.writeheader()
            f.close()
    else:
        showerror(title='Error',message="Đã tồn tại file: "+str(path_csv))

def new_csv_replace(path_csv, in_p="",): # in_p: tieu de
    try:
        with open(path_csv, 'w',newline='',encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=in_p)
            writer.writeheader()
            f.close()
    except OSError as e:
        showerror(title='Error',message="Không truy cập được file: "+str(path_csv))
# them dong moi và gia tri vao dong moi
def append_csv(path_csv,input_log):
    try:
        with open(path_csv, 'a', newline='',encoding="utf-8") as f_object:  
            writer_object = csv.writer(f_object)
            writer_object.writerow(input_log)  
            f_object.close()
    except OSError as e:
        showerror(title='Error',message="Không truy cập được file: "+str(path_csv))
# phan cach bang dau "," cot bat dau tu 1
def return_cot_tru_hang(path,hang = 0, cot = 0):
    list_output = []
    if os.path.exists(path) == True:
        with open( path, encoding="utf8") as csv_file:
            w = csv.reader(csv_file, delimiter=',')
            line_count = 1
            for row in w:
                # print(row)
                if line_count != int(hang):
                    if len(row) == 0:
                        row = [""]
                    if len(row) >= int(cot) and len(row) >= 1:
                        # print("l  =",row[int(cot-1)])
                        list_output.append(row[int(cot-1)])
                line_count +=1
            csv_file.close()
    return list_output
# print(return_cot_tru_hang("test.csv",3,1))
#load value csv
def return_value(path,hang,cot):
    list = ""
    if os.path.exists(path) == True:
        with open( path, encoding="utf8") as csv_file:
            w = csv.reader(csv_file, delimiter=',')
            line_count = 1
            for row in w:
                if line_count == int(hang):
                    if len(row) >= int(cot) and len(row) >= 1:
                        list = row[int(cot-1)]
                    break
                line_count +=1
            csv_file.close()
    return list

# return hang tu phan tu cot tro di (load value csv)
def return_hang_tu_cot(path,hang = 0,cot = 0):
    cot = cot - 1
    list = []
    if os.path.exists(path) == True:
        with open(path, encoding="utf8") as csv_file:
            w = csv.reader(csv_file, delimiter=',')
            line_count = 1
            for row in w:
                # if len(row) == 0:
                #     row = [""]
                if line_count == int(hang):
                    if len(row) >= int(cot) and len(row) >= 1:
                        list = row[int(cot):]
                    break
                line_count +=1
            csv_file.close()
    return list

# return cot tu phan tu hang tro di (load value csv)
def return_cot_tu_hang(path,hang = 0,cot = 0):
    cot = cot - 1
    list = []
    if os.path.exists(path) == True:
        with open(path, encoding="utf8") as csv_file:
            w = csv.reader(csv_file, delimiter=',')
            line_count = 1
            for row in w:
                if line_count >= int(hang):
                    if len(row) == 0:
                        row = [""]
                    if len(row) >= int(cot) and len(row) >= 1:
                        list.append(row[int(cot)])
                line_count +=1
            csv_file.close()
    return list
def load_so_hang_cot(path):
    so_cot = len(return_cot_tu_hang(path,1,1))
    so_hang = len(return_hang_tu_cot(path,1,1))
    return so_cot,so_hang
# print(load_so_hang_cot("test.csv"))


# print(return_cot_tu_hang("new_file.csv",1,1))
# load all file csv (output ra tat ca cac gi tri tron file csv)
# print(return_cot_tu_hang("new_file.csv",1,1))
def load_all_so_nguyen(path_input):
    mang = []
    so_hang = len(return_cot_tu_hang(path_input,1,1))
    for i in range(2,so_hang+1):
        danh_sach = return_hang_tu_cot(path_input,i,1)
        danh_sach_new = [float(l) for l in danh_sach]
        mang.append(danh_sach_new)
    return mang
def load_all(path_input):
    mang = []
    so_hang = len(return_cot_tu_hang(path_input,1,1))
    for i in range(1,so_hang+1):
        danh_sach = return_hang_tu_cot(path_input,i,1)
        if len(danh_sach) != 0:
            mang.append(danh_sach[0:])
    return mang
# bo dong dau tien, lay gia tri tu cot 2 tro di
def load_all_stt(path_input):
    mang = []
    so_hang = len(return_cot_tu_hang(path_input,1,1))
    for i in range(2,so_hang+1):
        danh_sach = return_hang_tu_cot(path_input,i,1)
        if len(danh_sach) != 0:
            mang.append(danh_sach[1:])
    return mang
# print(load_all_stt("test.csv"))
# chinh sua file csv (phan cach bang dau tab)
def edit_csv(path_input,hang,cot,value):
    # hang = hang + 2
    # cot = cot + 3
    path_2 = path_phan_mem + "/" + "edit.csv"
    if os.path.exists(path_input) == False:
        showerror(title='warning',message='Không có file: '+ path_input)
    if os.path.exists(path_input) == True:
        line_hang = 1
        mang = []
        with open(path_input, encoding="utf8") as csv_file:
            w = csv.reader(csv_file, delimiter=',')
            for row in w:
                if line_hang == hang:
                    line_cot = 1
                    l = 0
                    h = 0
                    text = ""
                    for i in list(row[0]):
                        if h == 1 and i == "\t":
                            line_cot = line_cot + 1
                        if i == "\t" and l == 0:
                            l = 1
                            h = 0
                        if l == 1 and i != "\t":
                            line_cot = line_cot + 1
                            l = 2
                        if line_cot != cot:
                            text = text + i
                        if i != "\t":
                            l = 0
                            if line_cot == cot and h == 0:
                                h = 1
                                text = text + str(value)
                    mang.append([text])
                if line_hang != hang:
                    mang.append(row)
                line_hang =line_hang + 1
            if len(mang) > 0:
                new_csv_replace(path_2,mang[0])
                for p in range(0,len(mang)):
                    if p != 0:
                        append_csv(path_2,mang[p])
            csv_file.close()
    if len(mang) > 0:
        os.remove(path_input)
        os.rename(path_2,path_input)
# edit_csv("test.csv",4,3,"hhh")
# xoa value trong file csv (phan cach bang dau tab)
def del_csv(path_input,hang,value=""):
    hang = hang + 2
    path_2 = path_phan_mem + "/" + "del.csv"
    if os.path.exists(path_input) == False:
        showerror(title='warning',message='Không có file: '+ path_input)
    if os.path.exists(path_input) == True:
        line_hang = 1
        mang = []
        with open(path_input, encoding="utf8") as csv_file:
            w = csv.reader(csv_file, delimiter=',')
            mang_0 = []
            const_co_ma = 0
            for row in w:
                if line_hang == hang:
                    line_cot = 1
                    l = 0
                    h = 0
                    text = ""
                    mang_0.append(del_tab(row[0]))
                    for i in range(0,len(mang_0[0])):
                        if mang_0[0][i] == str(value):
                            const_co_ma = 1
                            break
                    const_cot_moi = 0
                    if const_co_ma == 1:
                        for lt in list(row[0]):
                            if lt == "\t" and const_cot_moi == 0 :
                                const_cot_moi = 1
                            if const_cot_moi == 1 and lt != "\t":
                                line_cot = line_cot +1
                                const_cot_moi = 0
                            if line_cot != (i+1):
                                text = text + lt
                            else:
                                pass
                    else:
                        for lt in list(row[0]):
                            text = text + lt
                    mang.append([text])
                if line_hang != hang:
                    mang.append(row)
                line_hang =line_hang + 1
            if len(mang) > 0:
                new_csv_replace(path_2,mang[0])
                for p in range(0,len(mang)):
                    if p != 0:
                        append_csv(path_2,mang[p])
            csv_file.close()
    if len(mang) > 0:
        os.remove(path_input)
        os.rename(path_2,path_input)
# them gia tri vao hang cot tuong ung (phan cach bang dau tab)
def add_csv(path_input,hang,value = ""):
    hang = hang + 2
    path_2 = path_phan_mem + "/" + "add.csv"
    if os.path.exists(path_input) == False:
        showerror(title='warning',message='Không có file: '+ path_input)
    if os.path.exists(path_input) == True:
        line_hang = 1
        mang = []
        with open(path_input, encoding="utf8") as csv_file:
            w = csv.reader(csv_file, delimiter=',')
            for row in w:
                if line_hang == hang:
                    line_cot = 1
                    l = 0
                    h = 0
                    text = ""
                    for i in list(row[0]):
                        text = text + i
                    text = text +"\t" + value
                    mang.append([text])
                if line_hang != hang:
                    mang.append(row)
                line_hang =line_hang + 1
            if len(mang) > 0:
                new_csv_replace(path_2,mang[0])
                for p in range(0,len(mang)):
                    if p != 0:
                        append_csv(path_2,mang[p])
            csv_file.close()
    if len(mang) > 0:
        os.remove(path_input)
        os.rename(path_2,path_input)
# gan stt
def form_csv(path_input,so_cot = 20):
    stt_cot = ""
    stt_cot = stt_cot + ("0\t")
    stt_cot = stt_cot + ("0\t")
    for i in range(0,so_cot+1):
        stt_cot = stt_cot + "\t" + str(i)
    new_csv_replace("new_file.csv",[stt_cot])
    so_hang = len(return_cot_tu_hang(path_input,1,1))
    # mang = []
    for i in range(2,so_hang+1):
        danh_sach = return_hang_tu_cot(path_input,i,1)
        if len(danh_sach) != 0:
            danh_sach_new = list(danh_sach[0])
            stt = len(danh_sach_new)
            for ds_new in range(0,stt):
                if danh_sach_new[0] == "\t":
                    break
                del danh_sach_new[0]
            # mang = danh_sach_new
            # mang[0] = str(i-2)
            ap_csv = ""
            ap_csv = ap_csv + str(i-2)
            for k in danh_sach_new:
                ap_csv = ap_csv + k
            # if len(danh_sach_new) >5:
            #     ap_csv = ap_csv + "\t0\t0\t0"
            # print(ap_csv)
            append_csv("new_file.csv",[ap_csv])
        else:
            append_csv("new_file.csv",[""])
# form_csv("test.csv")