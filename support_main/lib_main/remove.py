import shutil,os

# remove all trong folder
def remove_all(path):
    if os.path.exists(path) == True:
        l = 0
        for i in list(path):
            if i == ".":
                l = 1
                try:
                    os.remove(path) 
                except OSError as e:
                    print('Không xóa được file/folder: ' + path)
                break
        if l == 0:
            try:
                shutil.rmtree(path)
            except OSError as e:
                print('Không xóa được file/folder: ' + path)
    else:
        print('Không tồn tại file/folder: ' + path)

#remove file trong folder
def remove_file(path):
    if os.path.exists(path) == True:
        try:
            os.remove(path) 
        except OSError as e:
            print('Không xóa được file: ' + path)
#remove folder
def remove_folder(path):
    if os.path.exists(path) == True:
        try:
            shutil.rmtree(path)
        except OSError as e:
            print('Không xóa được folder: ' + path)
def remove_all_in_folder(path):
    if os.path.exists(path) == True:
        ds = os.listdir(path)
        for i in range(0,len(ds)):
            remove_all(path+"/"+ds[i])
# remove file in folder
def remove_all_file_in_folder(path):
    if os.path.exists(path) == True:
        ds = os.listdir(path)
        for i1 in range(0,len(ds)):
            for i2 in ds[i1]:
                if i2 == ".":
                    remove_all(path+"/"+ds[i1])
                    break
# remove folder in folder
def remove_all_folder_in_folder(path):
    if type(path) == type([1,2]):
        for i in range(0,len(path)):
            if os.path.exists(path[i]) == True:
                ds = os.listdir(path[i])
                for i1 in range(0,len(ds)):
                    if len(list(ds[i1])) > 4:
                        if ds[i1][-4] != "." and  ds[i1][-5] != ".":
                            remove_folder(path[i] + "/" + ds[i1])
                    else:
                        remove_folder(path[i] + "/" + ds[i1])
    else:
        if os.path.exists(path) == True:
            ds = os.listdir(path)
            for i1 in range(0,len(ds)):
                if len(list(ds[i1])) > 4:
                    if ds[i1][-4] != "." and  ds[i1][-5] != ".":
                        remove_folder(path + "/" + ds[i1])
                else:
                    remove_folder(path + "/" + ds[i1])
#tao folder
def tao_folder(path_folder):
    if type(path_folder) == type([1,2]):
        for i in range(0,len(path_folder)):
            if os.path.exists(path_folder[i]) == False:
                try:
                    # os.mkdir(path_folder)
                    os.makedirs(path_folder[i])
                except OSError as e:
                    print("không tạo được folder: " + path_folder[i])
            else:
                # print("Folder đã tồn tại: " + path)
                pass
    else:
        if os.path.exists(path_folder) == False:
            try:
                # os.mkdir(path_folder)
                os.makedirs(path_folder)
            except OSError as e:
                print("không tạo được folder: " + path_folder)
        else:
            # print("Folder đã tồn tại: " + path)
            pass
    return path_folder

