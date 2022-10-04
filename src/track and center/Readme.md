# Camera to OpenCV with H480

Launch the simulation as
```bash
roslaunch px4 h480.launch
```

### QGroundControl
The camera stream is sent over udp to port `5600`. It can be viewed in QGroundControl directly

### Gstreamer
To view with gstreamer via terminal, run the following command
```bash
gst-launch-1.0  -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=1000 sync=false
```

### V4L2
First create a video device
```bash
sudo modprobe v4l2loopback video_nr="42" \
'card_label=virtcam'
```
or (incase the above command doesn't work)
```bash
sudo modprobe v4l2loopback video_nr="42"\
    'card_label=virtcam'\
    exclusive_caps=1 max_buffers=2
```
Sink to it with gstreamer
```bash
gst-launch-1.0  -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
! rtph264depay ! avdec_h264 ! videoconvert ! v4l2sink device=/dev/video42
```
reset the created device by running
```bash
sudo modprobe -r v4l2loopback
```
