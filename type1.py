import cv2
import numpy as np

def filter_colors(img_frame):
    """
    흰색/노란색 색상의 범위를 정해 해당되는 차선을 필터링한다.
    """
    lower_white = np.array([200, 200, 200])  # 흰색 차선 (RGB)
    upper_white = np.array([255, 255, 255])
    lower_yellow = np.array([10, 100, 100])  # 노란색 차선 (HSV)
    upper_yellow = np.array([40, 255, 255])

    white_mask = cv2.inRange(img_frame, lower_white, upper_white)
    white_image = cv2.bitwise_and(img_frame, img_frame, mask=white_mask)

    img_hsv = cv2.cvtColor(img_frame, cv2.COLOR_BGR2HSV)
    yellow_mask = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
    yellow_image = cv2.bitwise_and(img_frame, img_frame, mask=yellow_mask)

    output = cv2.addWeighted(white_image, 1.0, yellow_image, 1.0, 0.0)
    return output


def canny(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(blur, 50, 150)
    return canny

def display_lines(image, lines):
    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
    return line_image

def region_of_interest(image):
    height = image.shape[0]
    polygons = np.array([
    [(200, height), (1100, height), (550, 250)]
    ])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

cap = cv2.VideoCapture('/home/wonmadeit/catkin_ws/src/test/driving1.mp4')

 # 테스트 파일 경로설정

while(cap.isOpened()):
    _, frame = cap.read()
    lane_image = np.copy(frame)
    lane_canny = canny(lane_image)
    cropped_canny = region_of_interest(lane_canny)
    lines = cv2.HoughLinesP(cropped_canny, 2, np.pi/180, 100, np.array([]), minLineLength=40,maxLineGap=5)
    line_image = display_lines(lane_image, lines)
    combo_image = cv2.addWeighted(lane_image, 0.8, line_image, 1, 1)
    cv2.imshow('result', combo_image)
    
    if cv2.waitKey(50) & 0xFF == ord('q'):  # 'q' 키를 누르면 루프에서 빠져나옵니다.
        break

cap.release()
cv2.destroyAllWindows()
