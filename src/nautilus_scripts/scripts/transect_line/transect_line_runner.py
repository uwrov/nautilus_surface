#!/usr/bin/env python3
"""
ROS driver for the transect line task

Authored by Kelvin Ng 
Used: https://stackoverflow.com/questions/60150480/opencv-detect-only-a-particular-line-in-an-image
"""
import cv2
import numpy as np

def process_image(filename):
    # read image as grayscale
    img = cv2.imread(filename)
    img_height = img.shape[0]
    img_width = img.shape[1]

    # threshold on red color
    lowcolor = (0,0,75)
    highcolor = (50,50,135)
    thresh = cv2.inRange(img, lowcolor, highcolor)


    # apply morphology close
    kernel = np.ones((5,5), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    # get contours and filter on area
    result = img.copy()
    contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]
    result = img.copy()
    side_counts = {"left":0, "right":0, "top":0, "bottom":0}
    for c in contours:
        for point_list in c:
            for x,y in point_list:
                count_point(x, y, img_height, img_width, side_counts)

    print(side_counts)
    # show thresh and result    
    cv2.imshow("thresh", thresh)
    cv2.imshow("result", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # save resulting images
    cv2.imwrite('red_line_thresh.png',thresh)
    cv2.imwrite('red_line_extracted.png',result)


def count_point(x,y, img_height, img_width, side_counts):
    if x < 100:
        side_counts["left"]+=1
    if x > img_width-100:
        side_counts["right"]+=1
    if y < 100:
        side_counts["top"]+=1
    if y > img_height -100:
        side_counts["bottom"]+=1

if __name__ == "__main__":
    filename = './transect_images/Transect1_copy.jpg'
    process_image(filename)
