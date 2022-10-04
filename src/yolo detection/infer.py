import torch
import cv2



img = cv2.imread('/home/pravardhan/Downloads/nadir.jpg')
cv2.imshow('Image', img)


model = torch.hub.load('/home/pravardhan/Documents/yolo/yolov5', 'custom', path='/home/pravardhan/Documents/yolo/best.engine', source='local')
result = model(img)
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


final_img = plot_boxes(result, img)
cv2.imshow('Result', final_img)

cv2.waitKey(0)
cv2.destroyAllWindows()