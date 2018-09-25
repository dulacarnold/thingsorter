import cv2
import scipy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from IPython import embed

VIDEO_DEVS = [0,1,2]
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480


def open_cam_usb(dev, width, height):
    # We want to set width and height here, otherwise we could just do:
    #     return cv2.VideoCapture(dev)
    gst_str = ('v4l2src device=/dev/video{} ! '
               'video/x-raw, width=(int){}, height=(int){}, '
               'format=(string)RGB ! '
               'videoconvert ! appsink').format(dev, width, height)
    return cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)


def main():
    caps = []
    # for video_dev in VIDEO_DEVS:
    #     caps.append(open_cam_usb(video_dev, VIDEO_WIDTH, VIDEO_HEIGHT))
        # caps.append(cv2.VideoCapture(video_dev))
        # cv2.VideoWriter_fourcc('M','J', 'P', 'G')


    for i in range(60):
        for video_dev in VIDEO_DEVS:
            # cap = open_cam_usb(video_dev, VIDEO_WIDTH, VIDEO_HEIGHT)
            cap = cv2.VideoCapture(video_dev)
            ret, frame = cap.read()
            # print(frame)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            # plt.show()
            cap.release()
            # plt.imshow(frame)


    # for cap in caps:
    #     cap.release()


if __name__ == '__main__':
    main()
