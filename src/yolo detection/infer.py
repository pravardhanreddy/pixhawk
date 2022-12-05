import torch
import cv2
import pickle
import socket


MAX_VEL = 0.3
UDP_IP = '192.168.0.101'
UDP_PORT = 2001

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cap = cv2.VideoCapture(22)

outfile = open('/dev/shm/center.pkl', 'wb')

model = torch.hub.load('/home/pravardhan/Documents/yolo/yolov5', 'custom', path='/home/pravardhan/Documents/yolo/best.pt', source='local')
model.conf = 0.3

# classes = ['Tarp', 'Tarp']
classes = model.names

def plot_boxes(results, frame):
    cy, cx, _ = frame.shape
    cv2.circle(frame, (cx//2, cy//2), 3, (255,0,0), -1)
    labels, cord = results.xyxyn[0][:, -1].cpu().numpy(), results.xyxyn[0][:, :-1].cpu().numpy()
    n = len(labels)
    x_shape, y_shape = frame.shape[1], frame.shape[0]
    x1, y1, x2, y2 = None, None, None, None
    for i in range(n):
        row = cord[i]
        if row[4] >= 0.2:
            x1, y1, x2, y2 = int(row[0]*x_shape), int(row[1]*y_shape), int(row[2]*x_shape), int(row[3]*y_shape)
            bgr = (0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), bgr, 2)
            cv2.putText(frame, classes[int(labels[i])], (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, bgr, 2)
    res = ((x1+x2)//2, (y1+y2)//2, cx//2, cy//2, abs((x2-x1)*(y2-y1))) if x1 else None
    return frame, res if (x1 and n == 1) else None


while cap.isOpened():
    ret, frame = cap.read()

    if not ret:
        break

    result = model(frame)
    ans = plot_boxes(result, frame)

    # print(len(ans))
    frame, center = ans

    if center:
        r,c, cntrx, cntry, area = center 
        vel_y = max(min((cntrx - r) / 100 , MAX_VEL), -MAX_VEL)
        vel_x = max(min((cntry - c) / 100 , MAX_VEL), -MAX_VEL)

        close_enough = 0
        if abs(cntrx - r) < 30 and abs(cntry - c) < 30:
            close_enough = 1
        else:
            close_enough = 0
 
        msg = str(vel_x) + ',' + str(vel_y) + ',' + str(close_enough)
        print('Velx:',vel_x, 'vely:', vel_y, 'close:', close_enough)
        sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))

        pickle.dump(center, outfile)
    else:
        msg = 'None'
        sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))

    cv2.imshow('Result', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

outfile.close()
cap.release()
cv2.destroyAllWindows()