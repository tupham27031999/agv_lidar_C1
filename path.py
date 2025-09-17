import os
from support_main.lib_main import edit_csv_tab


# chuyen dia chi co dau \ sang /
def edit_path(input):
    new_path = ""
    for i in list(input):
        if i == str("\\"):
            new_path = new_path + "/"
        if i != str("\\"):
            new_path = new_path + i
    return new_path
def folder_goc(path_input):
    list_path_phan_mem = list(path_input)
    path_ouput = ""
    for i in list_path_phan_mem:
        if list_path_phan_mem[-1] != "/":
            del list_path_phan_mem[-1]
        else:
            break
    for i in list_path_phan_mem:
        path_ouput = path_ouput + i
    return path_ouput
path_phan_mem = edit_path(os.path.dirname(os.path.realpath(__file__)))
if path_phan_mem.split("/")[-1] == "_internal":
    path_phan_mem = path_phan_mem.replace("/_internal","")
path_path = path_phan_mem + "/setting/path.csv"
new_path = ""
dau_gach_cheo = list(path_phan_mem)[2]
ds_path = edit_csv_tab.load_all_stt(path_path)
# dia chi thu muc chua phan mem path_software (thua 1 dau "/")
path_software = folder_goc(path_input=path_phan_mem)

for i in range(0,len(ds_path)):
    if len(ds_path[i]) >=3 :
        if int(ds_path[i][2]) == 1:
            globals()[str(ds_path[i][0])] = edit_path(path_phan_mem + "/" + ds_path[i][1])
        if int(ds_path[i][2]) == 2:
            globals()[str(ds_path[i][0])] = edit_path(ds_path[i][1])
        if int(ds_path[i][2]) == 3:
            globals()[str(ds_path[i][0])] = edit_path(path_software + ds_path[i][1])
