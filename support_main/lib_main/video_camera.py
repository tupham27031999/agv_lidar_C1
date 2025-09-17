import cv2
import os
import remove
import time
#Capture video from webcam
def edit_path(input):
    new_path = ""
    for i in list(input):
        if i == str("\\"):
            new_path = new_path + "/"
        if i != str("\\"):
            new_path = new_path + i
    return new_path
path_phan_mem = edit_path(os.path.dirname(os.path.realpath(__file__)))

path_videos = path_phan_mem+"/videos"

remove.tao_folder(path_videos)
vid_capture =  cv2.VideoCapture("rtsp://admin:agv1_agv1@192.168.0.24")
# vid_capture =  cv2.VideoCapture(0)
vid_cod = cv2.VideoWriter_fourcc(*'XVID')
print("gõ tên video và nhấn enter (không cần .mp4)")
name_video = str(input())
ds_vieo = os.listdir(path_videos)
print("tên vừa nhập: " + name_video)
while True:
    name_ok = 1
    for i in range(0,len(ds_vieo)):
        if ds_vieo[i] == name_video+".mp4":
            print("trùng tên video, hãy nhập lại")
            print("gõ tên video và nhấn enter (không cần .mp4)")
            name_video = str(input())
            name_ok = 0
            break
    if name_ok == 1:
        print("đang mở video")
        print("nhấn phím x để thoát và lưu video")
        break

stt = 0
output = cv2.VideoWriter(path_videos+"/"+name_video+str(stt)+".mp4", vid_cod, 20.0, (1920,1080))
# output = cv2.VideoWriter(path_videos+"/"+name_video+str(stt)+".mp4", vid_cod, 20.0, (640,480))
t0 = time.time()
print("outvideo")
while(True):
    # Capture each frame of webcam video
    ret,frame = vid_capture.read()
    if ret == True:
        cv2.imshow("video", frame)
        output.write(frame)
        # Close and break the loop after pressing "x" key
        if cv2.waitKey(1) &0XFF == ord('x') or cv2.getWindowProperty('video', cv2.WND_PROP_VISIBLE) < 1:
            break
        if time.time() - t0 > 10 and stt == 0:
            t0 = time.time()
            stt = stt + 1
            output.release()
            output = cv2.VideoWriter(path_videos+"/"+name_video+str(stt)+".mp4", vid_cod, 20.0, (1920,1080))
            # output = cv2.VideoWriter(path_videos+"/"+name_video+str(stt)+".mp4", vid_cod, 20.0, (640,480))
        if time.time() - t0 > 3600 and stt != 0:
            t0 = time.time()
            stt = stt + 1
            output.release()
            output = cv2.VideoWriter(path_videos+"/"+name_video+str(stt)+".mp4", vid_cod, 20.0, (1920,1080))
            # output = cv2.VideoWriter(path_videos+"/"+name_video+str(stt)+".mp4", vid_cod, 20.0, (640,480))
# close the already opened camera
vid_capture.release()
# close the already opened file
output.release()
# close the window and de-allocate any associated memory usage
cv2.destroyAllWindows()