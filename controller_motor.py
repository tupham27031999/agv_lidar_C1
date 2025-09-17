
# giải thuật tính vận tốc 2 bánh cho agv


import numpy as np

class LQRController:
    def __init__(self, dt=0.8):
        self.Q = np.diag([5, 5, 0, 0.1])  # Trọng số trạng thái [
                                             # x, y (độ lệch vận tốc tăng khi càng tăng, tỷ lệ thuận với delta_v)
                                             # yaw - delta angle, góc lệch so với mong muốn (độ lệch vận tốc tăng khi càng nhỏ, tỷ lệ nghịch với delta_v)
                                             # v - giảm tốc độ khi agv di chuyển cong]
        self.R = np.diag([0.001, 0.001])     # Trọng số điều khiển [a, delta]
        self.dt = dt                         # Thời gian lấy mẫu (giây) (dt*1000 = van toc cong them)

    def lqr_control(self, state, ref):
        x_err = np.array([
            ref[0] - state[0],    # sai số x
            ref[1] - state[1],    # sai số y
            ref[2] - state[2],    # sai số góc
            ref[3] - state[3]     # sai số tốc độ
        ])
        A = np.eye(4)
        A[0,2] = -state[3]*np.sin(state[2])*self.dt
        A[0,3] = np.cos(state[2])*self.dt
        A[1,2] = state[3]*np.cos(state[2])*self.dt
        A[1,3] = np.sin(state[2])*self.dt
        A[2,3] = 0
        B = np.zeros((4,2))
        B[3,0] = self.dt
        B[2,1] = self.dt
        P = np.copy(self.Q)
        for _ in range(10):
            P = A.T @ P @ A - A.T @ P @ B @ np.linalg.inv(self.R + B.T @ P @ B) @ B.T @ P @ A + self.Q
        K = np.linalg.inv(self.R + B.T @ P @ B) @ (B.T @ P @ A)
        u = K @ x_err
        a = np.clip(u[0], -1, 1)  # Giới hạn gia tốc chuẩn hóa, sẽ scale sau
        delta = u[1]
        return a, delta

    def compute_wheel_speeds(self, v, w, wheel_base):
        v_left = v - w * wheel_base / 2.0
        v_right = v + w * wheel_base / 2.0
        return v_left, v_right
    def update_Q_by_angle(self, angle_error):
        """
        Tự động điều chỉnh Q theo độ lệch góc (angle_error, radian).
        """
        # Giá trị lớn nhất và nhỏ nhất cho x, y

        
        q_xy = 2*angle_error
        q_xy = 2*angle_error

        # Khi lệch góc lớn, giảm trọng số x, y, tăng yaw
        

        self.Q = np.diag([q_xy, q_xy, 0, 0.1])
        # print("self.Q",self.Q, q_xy, q_xy)

def agv_lqr_control(max_speed, min_speed, 
                    point_start, point_end,
                    v_l, v_r, 
                    v_ref_l, v_ref_r, 
                    yaw, yaw_ref, 
                    wheel_base, max_accel, ty_le_pixel_mm):
    """
    Điều khiển AGV 2 bánh chủ động + 1 bánh đa hướng bằng LQR.
    """
    
    x, y = point_start
    x_ref, y_ref = point_end
    lqr_controller = LQRController()
    lqr_controller.update_Q_by_angle(abs(int(float(yaw *180/np.pi))))
    v = (v_l + v_r) / 2         # tốc độ tiến hiện tại (mm/s)
    v_ref = (v_ref_l + v_ref_r) / 2  # tốc độ tiến mong muốn (mm/s)
    x_m = x * ty_le_pixel_mm / 1000.0
    y_m = y * ty_le_pixel_mm / 1000.0
    v_m = v / 1000.0
    x_ref_m = x_ref * ty_le_pixel_mm / 1000.0
    y_ref_m = y_ref * ty_le_pixel_mm / 1000.0
    v_ref_m = v_ref / 1000.0
    wheel_base_m = wheel_base / 1000.0

    state = [x_m, y_m, yaw, v_m]
    ref = [x_ref_m, y_ref_m, yaw_ref, v_ref_m]
    a, w = lqr_controller.lqr_control(state, ref)
    # print(f"Gia tốc: {a}, Góc quay: {w}")

    # Scale gia tốc theo max_accel
    a = np.clip(a, -max_accel/1000.0, max_accel/1000.0)
    v_new = np.clip(v_m + a * lqr_controller.dt, -max_speed/1000.0, max_speed/1000.0)

    v_left, v_right = lqr_controller.compute_wheel_speeds(v_new * 1000, w, wheel_base)
    # print(f"Tốc độ bánh trái: {v_left}, Tốc độ bánh phải: {v_right}")
    # Giới hạn vận tốc bánh xe nhưng giữ nguyên tỷ lệ
    max_abs = max(abs(v_left), abs(v_right), max_speed)
    if max_abs > max_speed:
        scale = max_speed / max_abs
        v_left *= scale
        v_right *= scale

    # Đảm bảo vận tốc không nhỏ hơn min_speed (theo trị tuyệt đối), giữ tỷ lệ và dấu
    min_abs = min(abs(v_left), abs(v_right))
    if 0 < min_abs < min_speed:
        scale = min_speed / min_abs
        v_left *= scale
        v_right *= scale

    # Giới hạn độ lệch vận tốc giữa 2 bánh không vượt quá 1000, vẫn giữ tỷ lệ và dấu
    diff = abs(v_right - v_left)
    max_diff = 1000
    if diff > max_diff:
        # Scale lại hai vận tốc để độ lệch đúng bằng max_diff, giữ tỷ lệ và dấu
        if v_right != v_left:
            scale = max_diff / diff
            v_left = v_left * scale
            v_right = v_right * scale
        # Nếu v_right == v_left thì không cần scale

    return int(v_left), int(v_right)

# ty_le_pixel_mm = 10
# van_toc_max = 5000
# point_start = [0, 0]    # pixel
# point_end = [100, 100]

# yaw0 = 5 * np.pi / 180  # rad
# yaw_1 = 10 * np.pi / 180  # rad

# v0_l = 100
# v0_r = 100
# v1_l = 1000
# v1_r = 1000

# wheel_base = 1000
# max_accel = 2000 # gia toc max

# print(agv_lqr_control(max_speed = van_toc_max,
#                       point_start = point_start, point_end = point_end,
#                       v_l = v0_l, v_r = v0_r, 
#                       v_ref_l = v1_l, v_ref_r = v1_r, 
#                       yaw = yaw0, yaw_ref = yaw_1, 
#                       wheel_base = wheel_base, max_accel = max_accel, ty_le_pixel_mm = ty_le_pixel_mm))