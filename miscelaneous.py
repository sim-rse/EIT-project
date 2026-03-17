import cv2

def image_threshold(image, threshold, maxval = 255):
    threshold_Val = int(threshold*maxval)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, image = cv2.threshold(image, threshold_Val,maxval,cv2.THRESH_BINARY)
    return image