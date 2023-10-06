#!/usr/bin/env python3
import cv2
import numpy as np
import math
import logging
import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from math import *
import matplotlib.pyplot as plt

class Lane_Detector:    

    def __init__(self, height, width):
        self.midpoint = 0
        self.w = width
        self.h = height
        self.color = ""
        self.kernel_size = 13
        self.low_threshold = 100
        self.high_threshold = 200
        self.a = 0.7
        self.b = 1.
        self.theta_w = 0.
        self.horizontal_line_y = float(height * 0.5)
        self.intersection_points =[]
        self.isZebra = 0
        self.stopZebra = 0
        self.numContourZebra = 0
        
        
    def wrapping(self, image):
        image = cv2.resize(image, (640,480))
        (self.h, self.w) = (image.shape[0], image.shape[1])
        offsetx=40
        offsety=50

        
        source = np.float32([[165, self.h/2 + 5],               #왼쪽위
                             [self.w-165, self.h/2 + 5],        #오른쪽위
                             [40, self.h+40],                   #왼쪽아래
                             [self.w-40, self.h+40]])           #오른쪽아래 
        
  
        #2D view
        destination = np.float32([[35, 0],
                                  [self.w-45,0],
                                  [63, self.h],
                                  [self.w-75, self.h]]) 
        
    
        self.transform_matrix = cv2.getPerspectiveTransform(source, destination)
        self.minv = cv2.getPerspectiveTransform(destination, source)
        self._image = cv2.warpPerspective(image, self.transform_matrix, (self.w, self.h))
        
        cv2.circle(image, (int(source[0][0]), int(source[0][1])), 10, (255,0,0),-1)
        cv2.circle(image, (int(source[1][0]), int(source[1][1])), 10, (0,255,0),-1)
        cv2.circle(image, (int(source[2][0]), int(source[2][1])), 10, (0,0,255),-1)
        cv2.circle(image, (int(source[3][0]), int(source[3][1])), 10, (255,255,255),-1)
        
        cv2.imshow("wrap", self._image)
        cv2.imshow("original", image)
        
        return self._image

    def color_filter(self, frame):
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        lower = np.array([0, 200, 0])   #흰색 최소
        upper = np.array([40, 255, 255]) #흰색 최대

        any_lower = np.array([100, 200, 0])        #초록 제외 최소
        any_upper = np.array([255, 255, 255])    #초록 제외 최대

        any_mask = cv2.inRange(hls, any_lower, any_upper)
        white_mask = cv2.inRange(hls, lower, upper)
        mask = cv2.bitwise_or(any_mask, white_mask)
        masked = cv2.bitwise_and(frame, frame, mask = mask)
        #cv2.imshow('masked', masked)
        
        return masked

    def roi(self, image):
        x = int(image.shape[1])
        y = int(image.shape[0])

        # 한 붓 그리기
        _shape = np.array(
            [[int(5),      int(y)],           
             [int(5),      int(0.5*y)],
             [int(0.40*x), int(0.5*y)], 
             [int(0.40*x), int(y)], 
             [int(0.55*x), int(y)], 
             [int(0.55*x), int(0.1*y)],
             [int(0.95*x), int(0.1*y)], 
             [int(0.95*x), int(y)], 
             [int(0.2*x),  int(y)]])

        mask = np.zeros_like(image)

        if len(image.shape) > 2:
            channel_count = image.shape[2]
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255

        cv2.fillPoly(mask, np.int32([_shape]), ignore_mask_color)
        masked_image = cv2.bitwise_and(image, mask)

        return masked_image

    def gaussian_blur(self, img, kernel_size):
        return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

    def calculate_histogram(self,frame,plot):
    # Generate the histogram
        histogram = np.sum(frame[int(
                frame.shape[0]/2):,:], axis=0)

        midpoint = np.int16(histogram.shape[0]/2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        # if abs(leftx_base - rightx_base) < 20:
        #     return
        #======================================================================
        
        self.midpoint = int((leftx_base + rightx_base) / 2)
        # Sliding window width is +/- margin
        margin = int((1/10) * self.w)  # Window width is +/- margin
        warped_frame = frame.copy()
        frame_sliding_window = warped_frame.copy()
        no_of_windows = 10
        # Set the height of the sliding windows
        window_height = np.int16(warped_frame.shape[0]/ (7 + no_of_windows))       
        no_of_windows = no_of_windows + 7
        # Find the x and y coordinates of all the nonzero 
        # (i.e. white) pixels in the frame. 
        nonzero = warped_frame.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1]) 

        # Store the pixel indices for the left and right lane lines
        left_lane_inds = []
        right_lane_inds = []
            
        # Current positions for pixel indices for each window,
        # which we will continue to update
        leftx_current = leftx_base
        rightx_current = rightx_base

        # Go through one window at a time
        minpix = int((1/40) * self.w)
        num_index_left = 0
        num_index_right = 0
        height_left_temp = 0
        height_right_temp = 0
        num_window_left = 0
        num_window_right = 0
        for window in range(no_of_windows):
            
            # Identify window boundaries in x and y (and right and left)
            win_y_low = warped_frame.shape[0] - (window + 1) * window_height
            win_y_high = warped_frame.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            cv2.rectangle(frame_sliding_window,(win_xleft_low,win_y_low),(
                win_xleft_high,win_y_high), (255,255,255), 2)
            cv2.rectangle(frame_sliding_window,(win_xright_low,win_y_low),(
                win_xright_high,win_y_high), (255,255,255), 2)

            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low)
                              & (nonzeroy < win_y_high) 
                              & (nonzerox >= win_xleft_low)
                              & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                                (nonzerox >= win_xright_low) & (
                                    nonzerox < win_xright_high)).nonzero()[0]
            
            
            if (num_index_left < len(good_left_inds)):
                num_index_left = len(good_left_inds)
                height_left_temp = (win_y_low + win_y_high)/2
            if (num_index_right < len(good_right_inds)):
                num_index_right = len(good_right_inds)
                height_right_temp = (win_y_low + win_y_high)/2
                
            if len(good_left_inds) > 2: num_window_left = num_window_left + 1
            if len(good_right_inds) > 2: num_window_right = num_window_right + 1
            if (num_window_right > 9) and (num_window_left > 9): break
            
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            
            cv2.imshow('sliding_window', frame_sliding_window) #sliding window 출력 ***
            
            # If you found > minpix pixels, recenter next window on mean position
            minpix = minpix
            if len(good_left_inds) > minpix:
                leftx_current = np.int16(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:        
                rightx_current = np.int16(np.mean(nonzerox[good_right_inds]))

        
        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
        
        # Extract the pixel coordinates for the left and right lane lines
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds] 
        righty = nonzeroy[right_lane_inds]
        
        print("Left Right")
        print(height_left_temp, height_right_temp)
        
        if (height_left_temp > height_right_temp): 
            height_left_temp = height_right_temp
        else:
            height_right_temp = height_left_temp
            
        if (height_left_temp < 100):
            height_right_temp = 0.8 * self.h
            height_left_temp = height_right_temp
        
        self.midpoint = abs(self.h - height_left_temp)
        
        self.midpoint = self.h * 0.8
        height_left_temp = self.h * 0.8
        height_right_temp = self.h * 0.9
        
        try:
            # Fit a second order polynomial curve to the pixel coordinates for
            # the left and right lane lines
            left_fit = np.polyfit(lefty, leftx, 2)
            self.left_fit = left_fit
            bottom_left = self.left_fit[0]*height_left_temp**2 + self.left_fit[1]*height_left_temp + self.left_fit[2]
        except:
            bottom_left = 0
            left_fitx = [0,0,0]
            print("Cannot detect left line")
            
        try:
            # Fit a second order polynomial curve to the pixel coordinates for
            # the left and right lane lines
            right_fit = np.polyfit(righty, rightx, 2)
            self.right_fit = right_fit
            #print(right_fit)
            bottom_right = self.right_fit[0]*height_right_temp**2 + self.right_fit[1]*height_right_temp + self.right_fit[2]
        except:
            bottom_right = 0
            right_fit = [0,0,0]
            print("Cannot detect right line")
        
        if (bottom_left > (self.w * 0.9)): bottom_left = 0
        if (bottom_right > self.w): bottom_right = self.w
        if (bottom_left < 0): bottom_left = 60
        if (bottom_right < 0): bottom_right = self.w
        
        if (abs(bottom_left - bottom_right) < 400) and (bottom_right != 0) and (bottom_left != 0):
            # if bottom_right < int(self.w/2) and bottom_left < int(self.w/2):
            #     print("1==================================")
            #     bottom_right = int(self.w)
            # elif bottom_right > int(self.w/2) and bottom_left > int(self.w/2):
                print("2============")
                bottom_left = 60
                bottom_right = self.w#bottom_right + 60
            #bottom_left = 40
            
        if bottom_right == 0:
            bottom_right = self.w
        
        print("Bottom")
        print(bottom_left)
        print(bottom_right)
        # Get position of car in centimeters
        #cv2.waitKey(100)
        car_location = self.w / 2

        # Fine the x coordinate of the lane line bottom
        center_lane = (bottom_right - bottom_left)/2 + bottom_left 
        
        if plot==True:
            # Create the x and y values to plot on the image  
            ploty = np.linspace(
                0, frame_sliding_window.shape[0]-1, frame_sliding_window.shape[0])
            left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
            right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
        
            # Generate an image to visualize the result
            out_img = np.dstack((
                frame_sliding_window, frame_sliding_window, (
                frame_sliding_window))) * 255
                    
            # Add color to the left line pixels and right line pixels
            out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
            out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [
                0, 0, 255]
                            
            # Plot the figure with the sliding windows
            figure, (ax1, ax2, ax3) = plt.subplots(3,1) # 3 rows, 1 column
            figure.set_size_inches(10, 10)
            figure.tight_layout(pad=3.0)
            orig_frame = frame.copy()
            ax1.imshow(cv2.cvtColor(orig_frame, cv2.COLOR_BGR2RGB))
            ax2.imshow(frame_sliding_window, cmap='gray')
            ax3.imshow(out_img)
            ax3.plot(left_fitx, ploty, color='yellow')
            ax3.plot(right_fitx, ploty, color='yellow')
            ax1.set_title("Original Frame")  
            ax2.set_title("Warped Frame with Sliding Windows")
            ax3.set_title("Detected Lane Lines with Sliding Windows")
            plt.show()
            
            #plt.close('all')
                

        # clear
        
        if plot==True:
            # Draw both the image and the histogram
            figure, (ax1, ax2) = plt.subplots(2,1) # 2 row, 1 columns
            figure.set_size_inches(10, 5)
            ax1.imshow(frame, cmap='gray')
            ax1.set_title("Warped Binary Frame")
            ax2.plot(histogram)
            ax2.set_title("Histogram Peaks")
            plt.show()
            
        return center_lane
    

def main():
    test = 414+10
    cap = cv2.VideoCapture(2)
    
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    lane_detector = Lane_Detector(width, width)
    
    steer_angle_old = 0
    steer_angle_new = 0
    steering_conversion = 0.35
    rospy.init_node("urrc_node1")
    ctrl_pub = rospy.Publisher("cameraAngle",Int16,queue_size=1)
    camera_msg = Int16()
    camera_msg.data = 12
    ctrl_pub.publish(camera_msg)
    bridge = CvBridge()
    rate = rospy.Rate(30)  # Set the desired loop rate (10 Hz in this case)
    print("inited")
    angle_send_old = 0
    zebra = False
    while not rospy.is_shutdown():
        ret, img = cap.read()
        imageshow = img.copy()
        frame = img.copy()        
        frame = cv2.resize(frame, (640,480))
        
        # Convert the video frame from BGR (blue, green, red) 
        # color space to HLS (hue, saturation, lightness).
        
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    
        ################### Isolate possible lane line edges ######################
        
        # Perform Sobel edge detection on the L (lightness) channel of 
        # the image to detect sharp discontinuities in the pixel intensities 
        # along the x and y axis of the video frame.             
        # sxbinary is a matrix full of 0s (black) and 255 (white) intensity values
        # Relatively light pixels get made white. Dark pixels get made black.
        # 흰색 차선 판단 파라미터(210 이상일경우 255로 변환)
        
        thresh=(210, 255)
        _, sxbinary = cv2.threshold(hls[:, :, 1], thresh[0], thresh[1], cv2.THRESH_BINARY)
        
        img = cv2.bitwise_and(frame, frame, mask=sxbinary)
        
        #cv2.imshow('mask',img) 
        
        #img_color = lane_detector.color_filter(img_wrap)
        img_mask = lane_detector.gaussian_blur(img, 1)
        #kernel_size = 1

        img_wrap = lane_detector.wrapping(img_mask)
        #cv2.imshow('wrap', img_wrap)

        
        img_roi = img_wrap.copy()#
        #cv2.imshow('roi', img_roi)
        
        if not ret:
            print("웹캠에서 프레임을 수신할 수 없습니다. 종료 중 ...")
            break
        
        # white_extracted = lane_detector.white_color_extraction_hsv(img)
        gray = cv2.cvtColor(img_roi, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)
        
        #img_roi = lane_detector.roi(thresh)
        #cv2.imshow('roi', img_roi)
        
        
        thre,th3 = cv2.threshold(thresh,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        zebraImage = th3.copy()
        th3 = lane_detector.roi(th3)
        contours, hierarchy = cv2.findContours(th3, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        # Initialize variables to keep track of the contour with maximum area
        # Iterate over the contours and find the contour with maximum area
        index = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            #print(area)
            if area < 500 or area > 25000:
                cv2.drawContours(th3, contours, index, 0, -1)
                cv2.drawContours(img, contours, index, 0, -1)
              
            index = index + 1 
        print('ang', index)
        cv2.imshow("Find contour result", th3); #컨투어 출력
        cv2.waitKey(1)
        
        contours, hierarchy = cv2.findContours(th3, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        
        numCountEnterZebra = 0
        # numCountEscapeZebra = 0
        # numCountStopZebra = 0
        for i in range(1,th3.shape[1]-2):
            for j in range(int(th3.shape[0]/2)-60, int(th3.shape[0]/2) + 60):
                if (th3[int(th3.shape[0]/2), i]):
                    numCountEnterZebra = numCountEnterZebra + 1
        numContour = 0
        print("FDSFFFFFFFFFFFFFF")
        print(numCountEnterZebra)
        #cv2.waitKey(200)
        if numCountEnterZebra > 18000:# and numCountEnterZebra < 200:#and lane_detector.isZebra==0:
            lane_detector.isZebra = 1
        if numCountEnterZebra > 90 and numCountEnterZebra < 7800:# and numCountEnterZebra < 200:#and lane_detector.isZebra==0:
            lane_detector.isZebra = 0
        if lane_detector.isZebra > 0:# and lane_detector.stopZebra == 0:
            print("ZEBRA _____________________________________________")
            maxAngle = 0
            maxContourIndex = 0
            maxContour = 10
            indexTemp = 0
            contours1, hierarchy = cv2.findContours(zebraImage, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            for c in contours1:
                area = cv2.contourArea(c)
                if maxContour < area and area < 20000:
                    maxContourIndex = indexTemp
                    maxContour = area
                indexTemp = indexTemp + 1
            
            [vx,vy,x,y] = cv2.fitLine(contours1[maxContourIndex], cv2.DIST_L2,0,0.01,0.01)
            
            rect = cv2.minAreaRect(contours1[maxContourIndex])
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            # print("Box Index")
            # print(box)
            cv2.drawContours(zebraImage,[box],0, 100,2)
            x1 = vx             # box[0][0] - box[1][0]
            y1 = vy             #box[0][1] - box[1][1]
            x2 = 0
            y2 = 1
            # Compute angle using atan2
            angle_radians = math.atan2(y1, x1)
            # 0 89 90 -89 0
            # Convert angle from radians to degrees
            angle_degrees = math.degrees(angle_radians)
            if angle_degrees < 5 or (angle_degrees > -5 and angle_degrees < 0): continue
            steer_angle_send = 0
            steer_angle_new = float(angle_degrees)
            sensorStart = 430
            sensorStep = 20
            angleStep = 2.5
            if steer_angle_new > 0 and steer_angle_new < (90 - angleStep):#left1
                steer_angle_send = sensorStart + 5 * sensorStep + 700
                print("Sent 6")
            elif steer_angle_new >= (90 - angleStep) or steer_angle_new < -(90-angleStep) or steer_angle_new == 0:#center
                steer_angle_send = sensorStart + 3 * sensorStep + 700
                print("Sent 4")
            elif steer_angle_new < 0 and steer_angle_new > -(90-angleStep):# right1
                steer_angle_send = sensorStart + 1 * sensorStep + 700
                print("Sent 2")
            
            if steer_angle_old == 0:
                steer_angle_old = steer_angle_new
            if 1:#(abs(steer_angle_new - steer_angle_old) <= 3):# from 0 to 8
                steer_angle_old = steer_angle_new
                if 1:#(angle_send_old==0) or (steer_angle_send != angle_send_old):
                    # if (steer_angle_send < 490):
                    #     steer_angle_send = 490
                    # elif (steer_angle_send > 530):
                    #     steer_angle_send = 530
                    print('Zebra')
                    camera_msg.data = steer_angle_send
                    angle_send_old = steer_angle_send
                    
                    print(steer_angle_send)
                    ctrl_pub.publish(camera_msg)
            
            
            cv2.imshow('MAIN CAM', zebraImage)
            # cv2.waitKey(10)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
            rate.sleep()  # Sleep to maintain the desired loop rate
            continue
        print("No Zebra __________________________________________")
        if lane_detector.stopZebra == 1 and lane_detector.isZebra > 0:
            print("Reset value")
            lane_detector.isZebra = 0
            lane_detector.stopZebra = 0
        center_lane = lane_detector.calculate_histogram(th3, 0) #histogram ***
        
        if center_lane != 0:
            imageshow = cv2.resize(imageshow,(640, 480))
            cv2.line(imageshow, (int(lane_detector.w/2), lane_detector.h), (int(center_lane), int(lane_detector.midpoint)), (0, 255, 255), 3)
            intersection_y = 0.1 * lane_detector.h #lane_detector.midpoint
            intersection_x = center_lane 
            
            #print(lane_detector.midpoint)
            angle_radians = math.atan2(intersection_y, (-int(lane_detector.w/2) + intersection_x))
            angle_degrees = math.degrees(angle_radians)
             
            if angle_degrees < 0: angle_degrees = 0
            #print(angle_degrees)
            
            if not math.isnan(angle_degrees):
                steer_angle_new = int(angle_degrees)
            else:
                steer_angle_new = 0
            start = 60 
            if (steer_angle_new > 175 or steer_angle_new < 5): continue     #직선 skip
            if (steer_angle_new <= start): steer_angle_new = start
            if (steer_angle_new > 120): steer_angle_new = 120
            steer_angle_send = 0
            sensorStart = 430
            sensorStep = 20
            angleStep = 4.29
            
            print("Steering angle new-------------------------------------------------")
            print(steer_angle_new)
            
            if steer_angle_new >= (90-angleStep*4) and steer_angle_new < (90-angleStep*3): #right3
                steer_angle_send = sensorStart
                print("Sent 1")
            elif steer_angle_new >= (90-angleStep*3) and steer_angle_new < (90-angleStep*2):#right2
                steer_angle_send = sensorStart + 1 * sensorStep
                print("Sent 2")
            elif steer_angle_new >= (90 - angleStep*2) and steer_angle_new < (90-angleStep):#right1
                steer_angle_send = sensorStart + 2 * sensorStep
                print("Sent 3")
            elif steer_angle_new >= (90 - angleStep) and steer_angle_new < (90+angleStep):#center
                steer_angle_send = sensorStart + 3 * sensorStep
                print("Sent 4")
            elif steer_angle_new >= (90+angleStep) and steer_angle_new < (90+angleStep*2):#left1
                steer_angle_send = sensorStart + 4 * sensorStep
                print("Sent 5")
            elif steer_angle_new >= (90+angleStep*2) and steer_angle_new < (90+angleStep*3):#left2 
                steer_angle_send = sensorStart + 5 * sensorStep
                print("Sent 6")
            elif steer_angle_new >= (90+angleStep*3) and steer_angle_new <= (90+angleStep*4):#left3
                steer_angle_send = sensorStart + 6 * sensorStep # 6->6.5
                print("Sent 7")
            # elif steer_angle_new >= (start + 7 * step) and steer_angle_new <= (start + 8 * step):  
            #     steer_angle_send = sensorStart + 7 * sensorStep
                #print("Sent 8") # 544부터 546까지 추가
            if steer_angle_old == 0:
                steer_angle_old = steer_angle_new
            if (abs(steer_angle_new - steer_angle_old) <= 7):# from 0 to 8
                steer_angle_old = steer_angle_new
                if 1:#(angle_send_old==0) or (steer_angle_send != angle_send_old):
                    camera_msg.data = steer_angle_send
                    angle_send_old = steer_angle_send
                    
                    print(steer_angle_send)
                    ctrl_pub.publish(camera_msg)
                    
            elif steer_angle_new==0:
                print("Error")
            
            steer_angle_old = steer_angle_new
        
        cv2.imshow('MAIN CAM', imageshow)
        # cv2.waitKey(10)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
        rate.sleep()  # Sleep to maintain the desired loop rate
    cap.release()

if __name__ == "__main__":
    main()