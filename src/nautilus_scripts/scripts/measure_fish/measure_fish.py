import cv2 as cv
import numpy as np
import matplotlib as plt

IMG_OFFSET = 50

def contour_transform(img):
    """
    Takes in an image from the video feed, finds and draws
    the contour of the fish, then transforms into a rectangular
    image and return
    """
    # find the largest rectangle
    img_copy = img.copy()
    img_gray = cv.cvtColor(img_copy, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(img_gray, 127, 255, 0)
    contours, heirarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

    # sort and find largest contour
    sorted_contours = sorted(contours, key=cv.contourArea, reverse=True)
    fish = sorted_contours[1]
    rect = cv.minAreaRect(fish)
    box = cv.boxPoints(rect).astype('int')

    # For testing:
    # cv.drawContours(img_copy, [box], 0, (0, 255, 0), 3)
    # image = cv.circle(img_copy, box[0], radius=0, color=(0, 0, 255), thickness=20)
    # image = cv.circle(img_copy, box[1], radius=0, color=(0, 0, 255), thickness=20)
    # image = cv.circle(img_copy, box[2], radius=0, color=(0, 0, 255), thickness=20)
    # image = cv.circle(img_copy, box[3], radius=0, color=(0, 0, 255), thickness=20)

    """
    Requires Testing: Does box order always end up like this?
    box[0] -> (0, 0)
    box[1] -> (length, 0)
    box[2] -> (length, height)
    box[3] -> (0, height)
    """
    
    # find the dimensions to transform into
    height = int((box[2][1] - box[1][1] + box[3][1] - box[0][1]) / 2)
    length = int((box[1][0] - box[0][0] + box[2][0] - box[3][0]) / 2)
    pts1 = np.float32(box)
    pts2 = np.float32([[0, 0], [length, 0], [length, height], [0, height]])

    matrix = cv.getPerspectiveTransform(pts1, pts2)
    output = cv.warpPerspective(img_copy, matrix, (length, height))
    cv.imshow('out', output)
    return output
    

# might not have to do this, just measure the height since its the same for all fishes
def segment_measure(transformed_img):
    """
    Takes in a transformed image, slices the image in its halfway point
    and uses the values to calculate the size of the fish and returns it.
    """
    # segment image and find pvc height
    height, width = transformed_img.shape[:2]
    start_row, start_col = int(height * .30), int(width * .45)
    end_row, end_col = int(height * .70), int(width * .55)
    cropped = transformed_img[start_row:end_row, start_col:end_col]
    
    # measure the distance between the edges
    img_gray = cv.cvtColor(cropped, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(img_gray, 127, 255, 0)
    contours, heirarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    sorted_contours = sorted(contours, key=cv.contourArea, reverse=True)
    pipe = sorted_contours[1]
    rect = cv.minAreaRect(pipe)
    box = cv.boxPoints(rect).astype('int')
    cv.drawContours(cropped, [box], 0, (0, 255, 0), 3)

    # find pipe height in pixels
    pipe_height = (box[3][1] - box[0][1] + box[2][1] - box[1][1]) / 2

    # pipe_height = 0.5 inches
    return 0.5 / pipe_height * width

def calculate_biomass(avg_length, N, a, b):
    """
    Calculates the biomass given the variables
    """
    return N * a * (avg_length ** b)


def main():
    """
    Main function
    """
    img = cv.imread('fish1.jpg')
    # might have to do some background cleaning like zoom and then put on white canvas?
    transformed_img = contour_transform(img)
    length = segment_measure(transformed_img)
    print(length)
    # next step: find length 2 more times using camera input, then find the average
    # plug in the average length into calculate biomass after inputting N, a, b
    return 0


if __name__ == "__main__":
    main()
