#20230729 gawpnlee 차선인식 mark1 가즈아
#파라미터라고 주석달린 부분 위주로 변경해가면서 트랙에 맞는 환경 설정해야돼
import cv2
import numpy as np
import matplotlib.pyplot as plt
# from sklearn.preprocessing import PolynomialFeatures
# from sklearn.linear_model import LinearRegression


cap = cv2.VideoCapture(2) 

def grayscale(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

def gaussian_blur(img, kernel_size):
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def canny(img, low_threshold, high_threshold):
    return cv2.Canny(img, low_threshold, high_threshold)

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    if len(img.shape) >2:
        channel_count = img.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
        
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def draw_lines(img, lines, color=[0, 255, 0], thickness=20):
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), color, thickness)

def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]),
                            minLineLength=min_line_len,
                            maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    draw_lines(line_img, lines)
    return line_img

#차선라인과 차선배경이미지 합치기
def weighted_img(img, initial_img, α=0.8, β=1., λ=0.):
    return cv2.addWeighted(initial_img, α, img, β, λ)

while True:
    ret, img = cap.read()
    
    gray = grayscale(img)
    
    #파라미터1
    #반드시 홀수입력(블러정도)
    kernel_size = 13
    blur_gray = gaussian_blur(gray, kernel_size)
    
    #파라미터2
    #흰차선 밝기 조정
    low_threshold = 50
    high_threshold = 200
    edges = canny(blur_gray, low_threshold, high_threshold)
    imshape = img.shape

    #파라미터3
    #roi 영역(사다리꼴) 꼭지점 좌표 
    #여기 수정해가면서 ROI영역 설정해주면 돼 하나씩 바꿔보면서 어떻게 달라지는지 봐~ :)
    vertices = np.array([[(0,imshape[0]), (200, 280), (1000,280), (imshape[1]-0,imshape[0])]], dtype=np.int32)
    mask = region_of_interest(edges, vertices)
    
    #파라미터4
    rho = 2             #r값, r = 원점에서 직선까지 거리
    theta = np.pi/180   #각도, 직선의 기울기
    threshold = 30      #직선으로 판단할 최소한
    min_line_len = 10   #차선으로 판단할 최소 길이
    max_line_gap = 300  #하나의 차선으로 판단할 최대 간격(점선들 하나의 선으로 이으려면 이거 수정)
    
    lines = hough_lines(mask, rho, theta, threshold, min_line_len, max_line_gap)
    lines_edges = weighted_img(lines, img, α=0.8, β=1., λ=0. )

    # 파라미터3에서 지정한 좌표로 선을 그리는 부분
    cv2.line(lines_edges, tuple(vertices[0][0]), tuple(vertices[0][1]), [255, 0, 0], 2)
    cv2.line(lines_edges, tuple(vertices[0][1]), tuple(vertices[0][2]), [255, 0, 0], 2)
    cv2.line(lines_edges, tuple(vertices[0][2]), tuple(vertices[0][3]), [255, 0, 0], 2)
    cv2.line(lines_edges, tuple(vertices[0][3]), tuple(vertices[0][0]), [255, 0, 0], 2)

    cv2.imshow('frame', lines_edges)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
