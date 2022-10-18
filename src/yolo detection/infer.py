import torch
import cv2
import pickle

cap = cv2.VideoCapture(6)

outfile = open('/dev/shm/center.pkl', 'wb')

model = torch.hub.load('/home/pravardhan/Documents/yolo/yolov5', 'custom', path='/home/pravardhan/Documents/yolo/yolov5/yolov5s.pt', source='local')
model.conf = 0.70

# classes = ['House', 'Tarp']
classes = model.names

def plot_boxes(results, frame):
    cy, cx = frame.shape
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

    return frame, ((x1+x2)//2, (y1+y2)//2, cx//2, cy//2, abs((x2-x1)*(y2-y1)))


while cap.isOpened():
    ret, frame = cap.read()

    if not ret:
        break

    result = model(frame)
    frame, center = plot_boxes(result, frame)

    pickle.dump(center, outfile)

    cv2.imshow('Result', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

outfile.close()
cap.release()
cv2.destroyAllWindows()