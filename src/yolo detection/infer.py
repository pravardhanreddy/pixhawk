import torch
import cv2


cap = cv2.VideoCapture('/home/pravardhan/Downloads/DJI_20220621160832_0002_S.MP4.mp4')


model = torch.hub.load('/home/pravardhan/Documents/yolo/yolov5', 'custom', path='/home/pravardhan/Documents/yolo/best.pt', source='local')
model.conf = 0.75

classes = ['House', 'Tarp']

def plot_boxes(results, frame):

    labels, cord = results.xyxyn[0][:, -1].cpu().numpy(), results.xyxyn[0][:, :-1].cpu().numpy()
    n = len(labels)
    x_shape, y_shape = frame.shape[1], frame.shape[0]
    for i in range(n):
        row = cord[i]
        if row[4] >= 0.2:
            x1, y1, x2, y2 = int(row[0]*x_shape), int(row[1]*y_shape), int(row[2]*x_shape), int(row[3]*y_shape)
            bgr = (0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), bgr, 2)
            cv2.putText(frame, classes[int(labels[i])], (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, bgr, 2)

    return frame


while cap.isOpened():
    ret, frame = cap.read()

    if not ret:
        break

    # frame = cv2.resize(frame, (640,480))

    result = model(frame)
    frame = plot_boxes(result, frame)

    cv2.imshow('Result', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()