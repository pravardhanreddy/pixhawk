import cv2
import numpy as np

vid = cv2.VideoCapture(42)
while True:
    ret, img = vid.read()
    cv2.circle(img, (320, 180), 3, (0,0,255), -1)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.medianBlur(gray, 19)
    sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    sharpen = cv2.filter2D(blur, -1, sharpen_kernel)

    thresh = cv2.threshold(sharpen,100,255, cv2.THRESH_BINARY_INV)[1]
    # cv2.imshow('thresh', thresh)


    # apply morphology
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11,11))
    clean = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (17,17))
    clean1 = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    blur = cv2.medianBlur(clean1, 19)
    sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    sharpen = cv2.filter2D(blur, -1, sharpen_kernel)

    clean = cv2.threshold(sharpen,100,255, cv2.THRESH_BINARY_INV)[1]
    # cv2.imshow('clean', clean)
    contours = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]

    for c in contours:
        rot_rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rot_rect)
        box = np.int0(box)
        # draw rotated rectangle on copy of img
        cv2.drawContours(img,[box],0,(0,0,0),2)
        cv2.circle(img, np.int0(rot_rect[0]), 3, (255,0,0), -1)
        cv2.putText(img, str(np.int0(rot_rect[0])), np.int0(rot_rect[0]), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.5, (0,255,0), 2, cv2.LINE_AA)



    cv2.imshow('shape', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()