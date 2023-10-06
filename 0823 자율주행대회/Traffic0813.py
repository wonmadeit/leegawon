#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String

class CameraCtrl:
    def __init__(self):
        rospy.init_node("camera_control_node")
        self.bridge = CvBridge()
        self.color_pub = rospy.Publisher("color", String, queue_size=1)
        rospy.Subscriber("/image_raw", Image, self.camera_callback)
        self.color = ""

    def gaussian_blur(self, image, kernel_size):
        return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)

    def canny(self, image, color):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # red 범위 설정
        lower_red = np.array([0, 50, 50], dtype=np.uint8)  # H, S, V
        upper_red = np.array([5, 255, 255], dtype=np.uint8)
        # red에 해당하는 범위의 픽셀 값만 마스킹hi
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        cv2.imshow('red', mask_red)
        
        # green 범위 설정
        #lower_green = np.array([60, 50, 50], dtype=np.uint8)  # H, S, V
        #upper_green = np.array([80, 255, 255], dtype=np.uint8)
        # green에 해당하는 범위의 픽셀 값만 마스킹
        #mask_green = cv2.inRange(hsv, lower_green, upper_green)
        #cv2.imshow('green', mask_green)
        
        #combined_mask = cv2.bitwise_or(mask_red, mask_green)

        #canny = cv2.Canny(combined_mask, 130, 130)
        canny = cv2.Canny(mask_red, 130, 130)

  
        # Check if green is detected
        #if np.any(mask_green):
        #    self.color = 'green'

        # Check if red is detected
        #if np.any(mask_red):
        #    self.color = 'red'
        
        cv2.imshow('canny', canny)
        return canny

    def display_circles(self, image, circles):
        circles_image = np.zeros_like(image)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in range(circles.shape[1]):
                x, y, z = circles[0, i]
                cv2.circle(circles_image, (x, y), radius=z, color=(255, 0, 0), thickness=1)
                
             #print('red')  
            self.color = 'red'

        else:
            self.color = ""      

        cv2.imshow('circle', circles_image)
        
        return circles_image

    def region_of_interest(self, image):
        polygons = np.array([
            [(0, 200), (320, 200), (320, 300), (0, 300)]
        ])
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, polygons, 255)
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image

    def draw_roi(self, image):
        polygons = np.array([
            [(0, 200), (320, 200), (320, 300), (0, 300)]
        ])
        image_copy = np.copy(image)
        cv2.polylines(image_copy, [polygons], True, (0, 255, 0), thickness=2)
        return image_copy

    def camera_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Apply Gaussian blur
        kernel_size = 15
        blur_image = self.gaussian_blur(cv_img, kernel_size)

        # Apply Canny edge detection and color detection
        circles_canny = self.canny(blur_image, self.color)

        # Apply region of interest
        cropped_canny = self.region_of_interest(circles_canny)

        # Detect circles using Hough Circle Transform
        circles = cv2.HoughCircles(cropped_canny, cv2.HOUGH_GRADIENT, 2, 15, param1=200, param2=30, minRadius=5, maxRadius=25)

        # Display circles on the image
        circles_image = self.display_circles(cv_img, circles)

        # Combine images and draw ROI
        combo_image = cv2.addWeighted(cv_img, 0.8, circles_image, 1, 2)
        combo_image = self.draw_roi(combo_image)

        # Resize image and display color
        scale_factor = 1
        large_image = cv2.resize(combo_image, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_LINEAR)
        print(self.color)
        self.color_pub.publish(self.color)

        # Display the final image
        cv2.imshow('Result Image', large_image)
        cv2.waitKey(1)

def publish_image():
    image_pub = rospy.Publisher("image_raw", Image, queue_size=10)
    bridge = CvBridge()
    capture = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        ret, img = capture.read()
        if not ret:
            rospy.logerr("Could not grab a frame!")
            break
        try:
            img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
            image_pub.publish(img_msg)
        except CvBridgeError as error:
            print(error)

def main():
    camera_ctrl = CameraCtrl()
    publish_image()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!")

if __name__ == "__main__":
    main()
