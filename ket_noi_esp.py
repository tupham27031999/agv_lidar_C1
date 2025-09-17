import serial
from support_main.lib_main import load_data_csv
import time
import shutil,os
import path
from support_main.lib_main import remove

path_phan_mem = path.path_phan_mem
path_data_input_output = path_phan_mem + "/data_input_output"
path_esp_sent_py = remove.tao_folder(path_data_input_output + "/esp_sent_py")
path_py_sent_esp = remove.tao_folder(path_data_input_output + "/py_sent_esp")

sent_data = ""
sent_data_new = ""
connect_esp = 1
check_connect_esp = False
input_esp =  {"IN1":0,"IN2":0,"IN3":0,"IN4":0,"IN5":0,"IN6":0,"IN7":0,"IN8":0,"IN9":0,"IN10":0,"IN11":0,"IN12":0}
remove.remove_all_folder_in_folder(path_esp_sent_py)

path_admin = path_phan_mem + "/setting/admin_window.csv"
if os.name == "nt":
    print("Hệ điều hành là Windows")
    # Đọc file cài đặt cho Windows
    path_admin = path_phan_mem + "/setting/admin_window.csv"
elif os.name == "posix":
    print("Hệ điều hành là Ubuntu (Linux)")
    # Đọc file cài đặt cho Ubuntu
    path_admin = path_phan_mem + "/setting/admin_ubuntu.csv"
data_admin = load_data_csv.ds_data(path_admin)[0]
for i in range(0,len(data_admin)):
    if len(data_admin[i]) != 0:
        if data_admin[i][0] == "cong_esp32":
            com = data_admin[i][1]
            hz = int(data_admin[i][2])
print("esp",com,hz)

def thap_phan_sang_nhi_phan(n): 
    return list(bin(n).replace("0b", ""))[::-1]

class Python_Esp:
    def __init__(self):
        self.connected = True
        # try:
        self.serial = serial.Serial()
        # except OSError as e:
        #     self.connected = False
        
        self.data_esp_sent = []

        self.start            = ""
        self.reset            = ""
        self.stop             = ""
        self.cam_bien_vat     = ""
        self.cam_bien_vi_tri  = ""
        self.poka_1_xuong     = ""
        self.cam_bien_barcode = ""
        self.poka_1_len       = ""
        self.poka_2_xuong     = ""
        self.poka_2_len       = ""
        self.barcode_ve       = ""
        self.barcode_ra       = ""

        self.time_reset = 0
        self.data = ""
        self.data_sent = ""
        self.list_ok = ["0","1","2","3","4","5","6","7","8","9"]   
        self.data_move = ""
        self.data_read_ouput = ""
        self.load_angle = 0
        self.angle = 0
        self.input_8 = 0

        self.load_data_esp = 0
        self.close_all = 0
    def khai_bao_serial(self):
        # if self.connected == True:
        self.serial.port = str(com)
        self.serial.baudrate = int(hz)
        self.serial.bytesize = serial.EIGHTBITS #number of bits per bytes
        self.serial.timeout = 3            #non-block read
        self.serial.writeTimeout = 2     #timeout for write
        self.serial.open()
        self.connected = True
        self.out = ""
        self.lay_du_lieu = 0
        self.data_old = ""
        self.time_sent = 0

        self.alpha_servo = 90
        self.name_data = ""

        self.input_esp = {"IN1":0,"IN2":0,"IN3":0,"IN4":0,"IN5":0,"IN6":0,"IN7":0,"IN8":0,"IN9":0,"IN10":0,"IN11":0,"IN12":0}
            
            
        # except OSError as e:
        #     self.connected = False
        #     print(self.connected,"esp32")
    def check_connect(self):
        try:
            self.serial.inWaiting()
            self.load_data_esp = 0
        except:
            self.connected = False
    def thread_load_data(self):
        if self.load_data_esp == 0 and self.close_all == 0:
            self.load_data_esp = 1
            self.load_data()
    def check_data_angle(self,data):
        list_ok = ["0","1","2","3","4","5","6","7","8","9","-","."]
        list_data = list(data)
        output = True
        for i in range(0,len(list_data)):
            output = False
            for i2 in range(0,len(list_ok)):
                if list_data[i] == list_ok[i2]:
                    output = True
                    break
            if output == False:
                break
        # print(output)
        return output
    def load_data(self):
        # print(self.connected == True and self.close_all == 0)
        if self.connected == True and self.close_all == 0:
            while self.serial.inWaiting() > 0:
                if self.close_all == 1:
                    break
                self.out = ""
                self.out = self.read_data()
                if self.out != "":
                    # if self.load_angle == 1:
                    # and self.out != "8_1" and self.out != "8_0" and self.out != "3_1" and self.out != "3_0":
                    # print(self.out, "------ttt-----")
                    if len(str(self.out).split("#")) >= 2:
                        if str(self.out).split("#")[0] == "data":
                            #  "data#" + data_load + "#" + "\r\n";
                            self.data_move = str(self.out).split("#")[1] +"\r\n"
                    if len(str(self.out).split("#")) >= 3:
                        input_esp = bin(int(float(str(self.out).split("#")[2]))).replace("0b", "")
                        for i in range(1,len(list(input_esp))):
                            self.input_esp["IN" + str(13-i)] = int(float(input_esp[i]))
                        # print("self.input_esp", self.input_esp)
                    if (self.data_sent != "" and self.data_sent != self.data_move) or time.time() - self.time_sent > 1:
                        self.time_sent = time.time()
                        if self.time_sent != "":
                            try:
                                self.name_data = str(self.data_sent).split("#")[0]
                                self.serial.write(self.data_sent.encode())
                            except:
                                self.connected = False
                                break
                        else:
                            data = "connect#384\r\n"
                            try:
                                self.serial.write(data.encode())
                            except:
                                self.connected = False
                                break

            if self.close_all == 1:
                self.serial.close()
            self.load_data_esp = 0
        # if len(self.data_esp_sent) != 0:
        #     self.start            = str(self.data_esp_sent[10]) #1 - start
        #     self.reset            = str(self.data_esp_sent[1]) #2 - reset
        #     self.stop             = str(self.data_esp_sent[2]) #3 - stop
        #     self.cam_bien_vat     = str(self.data_esp_sent[3]) #4
        #     self.cam_bien_vi_tri  = str(self.data_esp_sent[4]) #5
        #     self.poka_1_xuong     = str(self.data_esp_sent[5]) #6 
        #     self.cam_bien_barcode = str(self.data_esp_sent[6]) #7
        #     self.poka_1_len       = str(self.data_esp_sent[7]) #8
        #     self.poka_2_xuong     = str(self.data_esp_sent[8]) #9
        #     self.poka_2_len       = str(self.data_esp_sent[9]) #10
        #     self.barcode_ve       = str(self.data_esp_sent[0]) #11
        #     self.barcode_ra       = str(self.data_esp_sent[11]) #12


    def read_data(self):
        load = 0
        self.data = self.serial.readline()
        if self.data != "":
            for ot in list(str(self.data)):
                if ot == str("\\"):
                    break
                if load == 1 and ot != str("'") and ot != "b":
                    self.out = self.out + ot
                if ot == str("'"):
                    load = 1
        return self.out
    def check_data(self,data):
        ds = list(data)
        ok = "0"
        for i in range(0,len(ds)):
            for i2 in range(0,len(self.list_ok)):
                if self.list_ok[i2] == ds[i]:
                    ok = "1"
                    break
            if ok == "0":
                break
        return ok

    def sent_data(self,data):
        # if self.data_move == "" or self.data_move == self.data_sent:
        self.data_sent = data
    def close_serial(self):
        self.close_all = 1
def py_sent_esp(data = "",reset = 0):
    global sent_data,sent_data_new
    sent_data = data
    if reset == 1:
        sent_data_new = ""
        sent_data = ""
def close_serial():
    global connect_esp
    connect_esp = 0
def esp_sent_py():
    ds_esp_sent_py = os.listdir(path_esp_sent_py)
    connected = False
    start,reset,stop,cam_bien_vat,cam_bien_vi_tri,poka_1_xuong,cam_bien_barcode,poka_1_len,\
        poka_2_xuong,poka_2_len,barcode_ra,barcode_ve = ["","","","","","","","","","","",""]
    for i in ds_esp_sent_py:
        if str(i) == "connected" and len(ds_esp_sent_py) > 1:
            connected = True
        if str(i) == "stop":
            stop = "1"
            start = "0"
        elif stop == "":
            stop = "0"

        if str(i) == "start" and stop == "0":
            start = "1"
        elif start == "":
            start = "0"

        if str(i) == "reset":
            reset = "1"
        elif reset == "":
            reset = "0"

        if str(i) == "cam_bien_vi_tri":
            cam_bien_vi_tri = "1"
        elif cam_bien_vi_tri == "":
            cam_bien_vi_tri = "0"

        if str(i) == "poka_1_xuong":
            poka_1_xuong = "1"
        elif poka_1_xuong == "":
            poka_1_xuong = "0"

        if str(i) == "cam_bien_vat":
            cam_bien_vat = "1"
        elif cam_bien_vat == "":
            cam_bien_vat = "0"

        if str(i) == "cam_bien_barcode":
            cam_bien_barcode = "1"
        elif cam_bien_barcode == "":
            cam_bien_barcode = "0"

        if str(i) == "poka_1_len":
            poka_1_len = "1"
        elif poka_1_len == "":
            poka_1_len = "0"

        if str(i) == "poka_2_len":
            poka_2_len = "1"
        elif poka_2_len == "":
            poka_2_len = "0"

        if str(i) == "poka_2_xuong":
            poka_2_xuong = "1"
        elif poka_2_xuong == "":
            poka_2_xuong = "0"

        if str(i) == "barcode_ra":
            barcode_ra = "1"
        elif barcode_ra == "":
            barcode_ra = "0"

        if str(i) == "barcode_ve":
            barcode_ve = "1"
        elif barcode_ve == "":
            barcode_ve = "0"

    return connected,start,reset,stop,cam_bien_vat,cam_bien_vi_tri,poka_1_xuong,cam_bien_barcode,poka_1_len,poka_2_xuong,poka_2_len,barcode_ra,barcode_ve


def python_esp32():
    global sent_data,sent_data_new, connected, input_esp, check_connect_esp
    py_esp = Python_Esp()
    py_esp.khai_bao_serial()

    while True:
        input_esp = py_esp.input_esp
        check_connect_esp = py_esp.connected
        if connect_esp == 0:
            py_esp.close_serial()
            break
        py_esp.check_connect()
        py_esp.load_data()
        if py_esp.connected == True:
            connected = True
        if py_esp.connected == False:
            py_esp.khai_bao_serial()
        if sent_data != "":
            py_esp.sent_data(sent_data)
# py_esp = Python_Esp()
# py_esp.khai_bao_serial()

# def while_esp():
#     global sent_data,sent_data_new, connected, input_esp, py_esp
#     input_esp = py_esp.input_esp
#     if connect_esp == 0:
#         py_esp.close_serial()
#         # break
#     py_esp.check_connect()
#     py_esp.load_data()
#     if py_esp.connected == True:
#         connected = True
#     if py_esp.connected == False:
#         py_esp.khai_bao_serial()
#     if sent_data != "":
#         py_esp.sent_data(sent_data)
# python_esp32()
