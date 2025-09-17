
import cv2
import numpy as np

def bresenham_line(x0, y0, x1, y1):
    """Thuật toán Bresenham để tìm các điểm trên đường thẳng nối hai điểm (x0, y0) và (x1, y1)"""
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        points.append((int(x0), int(y0)))
        if x0 == x1 and y0 == y1:
            break
        e2 = err * 2
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

    return points
def find_farthest_point_in_radius(points, center, radius):
    max_point = []
    max_distance = -1
    cx, cy = center
    for point in points:
        px, py = point
        distance = np.sqrt((px - cx) ** 2 + (py - cy) ** 2)
        if distance <= radius and distance > max_distance:
            max_distance = distance
            max_point = [point[0], point[1]]
    return max_point
def crop_square_image_to_points(image, point1, point2, distance):
    x1, y1 = point1
    x2, y2 = point2
    x_min = min(x1, x2)
    x_max = max(x1, x2)
    y_min = min(y1, y2)
    y_max = max(y1, y2)
    
    # Tính toán kích thước của hình vuông bao quanh hai điểm
    # print(x_max - x_min, y_max - y_min, distance)
    side_length = max(x_max - x_min, y_max - y_min, distance)
    
    # Tính toán tọa độ của hình vuông
    x_center = (x_min + x_max) // 2
    y_center = (y_min + y_max) // 2
    x_min_square = max(0, x_center - side_length // 2)
    x_max_square = min(image.shape[1], x_center + side_length // 2)
    y_min_square = max(0, y_center - side_length // 2)
    y_max_square = min(image.shape[0], y_center + side_length // 2)
    
    number = 10
    cropped_image = image[y_min_square-number:y_max_square+number, x_min_square-number:x_max_square+number]
    x_min = x_min_square-number
    y_mind = y_min_square-number
    x_max = x_max_square+number
    y_max = y_max_square+number
    return cropped_image,x_min,y_mind,x_max,y_max
def img_crop(image, start_point, end_point, distance = 70):
    # Vẽ đường thẳng sử dụng thuật toán Bresenham
    line_points = bresenham_line(start_point[0], start_point[1], end_point[0], end_point[1])

    max_point = find_farthest_point_in_radius(line_points, start_point, distance)
    # max_point = end_point

    cropped_image,x_min,y_min,x_max,y_max = crop_square_image_to_points(image, start_point, max_point,distance)

    return cropped_image,max_point,x_min,y_min,x_max,y_max

# line_points = [(2720, 3048), (2721, 3048), (2722, 3048), (2723, 3048), (2724, 3048), (2725, 3048), (2726, 3048), (2727, 3048), (2728, 3048), (2729, 3048), (2730, 3048), (2731, 3048), (2732, 3048), (2733, 3048), (2734, 3048), (2735, 3048), (2736, 3048), (2737, 3048), (2738, 3048), (2739, 3048), (2740, 3048), (2741, 3048)]
# start_point =  [2741, 3048]
# distance = 50
# a = find_farthest_point_in_radius(line_points, start_point, distance)
# print(a)
# # Ví dụ sử dụng
# if __name__ == "__main__":
#     # Đường dẫn đến tệp ảnh
#     image = np.ones((500, 500, 3), dtype=np.uint8) * 255
#     start_point = (50, 50)
#     end_point = (450, 450)
    
#     cv2.line(image, (start_point[0], start_point[1]), (end_point[0], end_point[1]), (255, 0, 0), 2)
#     cv2.circle(image, (start_point[0], start_point[1]), 5, (0, 255, 0), -1)
#     cv2.circle(image, (end_point[0], end_point[1]), 5, (0, 0, 255), -1)
#     # Vẽ đường thẳng sử dụng thuật toán Bresenham
#     line_points = bresenham_line(start_point[0], start_point[1], end_point[0], end_point[1])
#     for point in line_points:
#         cv2.circle(image, point, 1, (255, 0, 0), -1)
#     max_point = find_farthest_point_in_radius(line_points, start_point, 50)
#     cropped_image = crop_square_image_to_points(image, start_point, max_point)

#     # Hiển thị ảnh trong một cửa sổ
#     while True:
#         cv2.imshow('cropped_image', cropped_image)
#         cv2.imshow('Image', image)
#         # Chờ người dùng nhấn phím bất kỳ để đóng cửa sổ
#         cv2.waitKey(1)
#         if cv2.getWindowProperty('Image', cv2.WND_PROP_VISIBLE) < 1:
#             break
#     cv2.destroyAllWindows()