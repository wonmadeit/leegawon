import cv2
import numpy as np
import rospy
from std_msgs.msg import Int16

# def region_of_interest(image, points):
#     img_mask = np.zeros_like(image, dtype=np.uint8)
#     cv2.fillPoly(img_mask, [points], (255, 255, 255))
#     masked_img = cv2.bitwise_and(image, img_mask)
#     return masked_img

# def region_of_interest_crop(image, points):
#     r = (points[0][0], points[0][1], points[2][0], points[2][1])
#     img_roi_crop = image[points[0][1]:points[2][1], points[0][0]:points[2][0]]
#     return img_roi_crop

# def canny_edge_detection(img):
#     mat_blur_img = cv2.blur(img, (3, 3))
#     mat_canny_img = cv2.Canny(mat_blur_img, 70, 170)
#     return mat_canny_img


def DetectLineSlope(src):
    # 흑백화
    gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

    # 모서리 검출
    can = cv2.Canny(gray, 50, 200, None, 3)

    # 관심 구역 설정
    height = can.shape[0]
    rectangle = np.array([[(0, height), (120, 300), (520, 300), (640, height)]])
    mask = np.zeros_like(can)
    cv2.fillPoly(mask, rectangle, 255)
    masked_image = cv2.bitwise_and(can, mask)
    ccan = cv2.cvtColor(masked_image, cv2.COLOR_GRAY2BGR)

    # 직선 검출
    line_arr = cv2.HoughLinesP(masked_image, 1, np.pi / 180, 20, minLineLength=10, maxLineGap=10)

    #line color
    # color = [0, 0, 255] #BGR
    # thickness = 5
    # for line in line_arr:
    #   for x1, y1, x2, y2 in line:
    #        cv2.line(ccan, (x1, y1), (x2, y2), color, thickness)

    #중앙을 기준으로 오른쪽, 왼쪽 직선 분리
    line_R = np.empty((0, 5), int)
    line_L = np.empty((0, 5), int)
    if line_arr is not None:
        line_arr2 = np.empty((len(line_arr), 5), int)
        for i in range(0, len(line_arr)):
            temp = 0
            l = line_arr[i][0]
            line_arr2[i] = np.append(line_arr[i], np.array((np.arctan2(l[1] - l[3], l[0] - l[2]) * 180) / np.pi))
            if line_arr2[i][1] > line_arr2[i][3]:
                temp = line_arr2[i][0], line_arr2[i][1]
                line_arr2[i][0], line_arr2[i][1] = line_arr2[i][2], line_arr2[i][3]
                line_arr2[i][2], line_arr2[i][3] = temp
            if line_arr2[i][0] < 320 and (abs(line_arr2[i][4]) < 170 and abs(line_arr2[i][4]) > 95):
                line_L = np.append(line_L, line_arr2[i])
            elif line_arr2[i][0] > 320 and (abs(line_arr2[i][4]) < 170 and abs(line_arr2[i][4]) > 95):
                line_R = np.append(line_R, line_arr2[i])
    line_L = line_L.reshape(int(len(line_L) / 5), 5)
    line_R = line_R.reshape(int(len(line_R) / 5), 5)

    #중앙과 가까운 오른쪽, 왼쪽 선을 최종 차선으로 인식
    try:
        line_L = line_L[line_L[:, 0].argsort()[-1]]
        degree_L = line_L[4]
        cv2.line(ccan, (line_L[0], line_L[1]), (line_L[2], line_L[3]), (255, 0, 0), 10, cv2.LINE_AA)
    except:
        degree_L = 0
    try:
        line_R = line_R[line_R[:, 0].argsort()[0]]
        degree_R = line_R[4]
        cv2.line(ccan, (line_R[0], line_R[1]), (line_R[2], line_R[3]), (255, 0, 0), 10, cv2.LINE_AA)
    except:
        degree_R = 0

    #원본에 합성
    mimg = cv2.addWeighted(src, 1, ccan, 1, 0)
    return mimg, degree_L, degree_R


cap = cv2.VideoCapture(2)

while cap.isOpened():

    ret, frame = cap.read()
    # 상하좌우 반전
    # frame = cv2.flip(frame, 0)

    if ret:
        frame = cv2.resize(frame, (640, 360))
        cv2.imshow('ImageWindow', DetectLineSlope(frame)[0])
        l, r = DetectLineSlope(frame)[1], DetectLineSlope(frame)[2]

        if abs(l) <= 155 or abs(r) <= 155:
            if l == 0 or r == 0:
                if l < 0 or r < 0:
                    print('left')
                elif l > 0 or r > 0:
                    print('right')
            elif abs(l - 15) > abs(r):
                print('right')
            elif abs(r + 15) > abs(l):
                print('left')
            else:
                print('go')
        else:
            if l > 155 or r > 155:
                print('hard right')
            elif l < -155 or r < -155:
                print('hard left')

        if cv2.waitKey(1) & 0xff == ord('q'):
            break

# def main():
#     img_width = 640
#     img_height = 360
#     USE_CAMERA = 1

#     if USE_CAMERA == 0:
#         img_height = 480

#     cap = cv2.VideoCapture(2)  # Replace 0 with the camera index if using a different camera

#     points = np.array([[0, 300 - 60], [0, 300 + 60], [img_width, 300 + 60], [img_width, 300 - 60]], np.int32)

#     while not rospy.is_shutdown():
#             ret, mat_image_org_color = cap.read()

#             if not ret:
#                 break

#             mat_image_org_gray = cv2.cvtColor(mat_image_org_color, cv2.COLOR_RGB2GRAY)
#             mat_image_roi = region_of_interest_crop(mat_image_org_gray, points)
#             mat_image_canny_edge = canny_edge_detection(mat_image_roi)

#             # Rest of the code for processing and publishing the data goes here

#             cv2.imshow("Camera Image", mat_image_org_color)

#             if cv2.waitKey(25) & 0xFF == ord('q'):
#                 break

#         cap.release()

cv2.destroyAllWindows()