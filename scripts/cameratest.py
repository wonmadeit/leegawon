import cv2
import numpy as np

capture=cv2.VideoCapture(0) #노트북 내장캠은 0, USB캠은 2로 연결
capture.set(cv2.CAP_PROP_FRAME_WIDTH,640)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

while True:
    ret, frame = capture.read()

    # noise=np.uint8(np.random.normal(loc=0,scale=0.4,size=[480,640,3]))
    # noise_img=cv2.add(frame, noise)
    # median_filter=cv2.medianBlur(noise_img,5)

    cv2.imshow("original",frame)
    # cv2.imshow("noised",noise_img)
    # cv2.imshow("median",median_filter)
    if cv2.waitKey(1) == ord('q'):
        break

    capture.release()
    cv2.destroyAllWindows()
