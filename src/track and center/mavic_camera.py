import cv2

# rtmp_url = "rtmp://localhost/live/testing"
# video_capture = cv2.VideoCapture(rtmp_url)
video_capture = cv2.VideoCapture(2)

while True:
    # Capture frame-by-frame
    ret, frames = video_capture.read()
    cv2.imshow('Video', frames)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video_capture.release()
cv2.destroyAllWindows()