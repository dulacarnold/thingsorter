#!/usr/bin/env python3
import cv2
import scipy
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from IPython import embed

VIDEO_DEVS = [0, 1, 2]
VIDEO_WIDTH = 1280
VIDEO_HEIGHT = 720


def open_cam_usb(dev, width, height):
    # We want to set width and height here, otherwise we could just do:
    #     return cv2.VideoCapture(dev)
    # gst_str1 = ('v4l2src device=/dev/video{} ! '
    #            'video/x-raw, width=(int){}, height=(int){}, '
    #            'format=(string)RGBA ! '
    #            'nvvidconv ! appsink').format(dev, width, height)
    gst_str = ('v4l2src device=/dev/video{} ! '
               'image/jpeg,widh=(int){},height=(int){},framerate=10/1,'
               'format=(string)RGB ! '
               'jpegdec ! videoconvert ! appsink').format(dev, width, height)
    # gst_str = 'v4l2src ! image/jpeg,width=1280,height=720 ! jpegdec ! videoconvert ! appsink name=sink caps=video/x-raw, format=(string){BGR, GRAY8}; video/x-bayer,format=(string){rggb,bggr,grbg,gbrg}''
    return cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

def main_stream():
    caps = []

    for video_dev in VIDEO_DEVS:
        cap = open_cam_usb(video_dev, VIDEO_WIDTH, VIDEO_HEIGHT)
        # cap = cv2.VideoCapture(video_dev)
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)
        # caps.append(cv2.VideoCapture(video_dev))
        caps.append(cap)

        # cv2.VideoWriter_fourcc('M','J', 'P', 'G')

    for i in range(1000):
        for cap in caps:
            ret, frame = cap.read()
            # print(frame)
            print('.')
            # time.sleep(0.05)
            # print(ret, frame)
            if frame is not None:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            else:
                print('!')
            # plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            # plt.show()

    for cap in caps:
        cap.release()


# def main():
#     caps = []
#     for video_dev in VIDEO_DEVS:
#         # caps.append(open_cam_usb(video_dev, VIDEO_WIDTH, VIDEO_HEIGHT))
#         caps.append(cv2.VideoCapture(video_dev))
#         # cv2.VideoWriter_fourcc('M','J', 'P', 'G')

#     for i in range(60):
#         for video_dev in VIDEO_DEVS:
#             # cap = open_cam_usb(video_dev, VIDEO_WIDTH, VIDEO_HEIGHT)
#             cap = cv2.VideoCapture(video_dev)
#             # ret, frame =
#             ret, frame = cap.read()
#             print(frame)
#             frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#             # plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
#             # plt.show()
#             cap.release()
#             # plt.imshow(frame)

#     # for cap in caps:
#     #     cap.release()

def display_main():
    caps = []

    for video_dev in VIDEO_DEVS:
        cap = open_cam_usb(video_dev, VIDEO_WIDTH, VIDEO_HEIGHT)
        # cap = cv2.VideoCapture(video_dev)
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)
        # caps.append(cv2.VideoCapture(video_dev))
        caps.append(cap)

        # cv2.VideoWriter_fourcc('M','J', 'P', 'G')

    for i in range(1000):
        for idx, cap in enumerate(caps):
            ret, frame = cap.read()
            # print(frame)
            print('.')
            # time.sleep(0.05)
            # print(ret, frame)
            if frame is not None:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                cv2.imshow('cam{}'.format(idx), frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print('!')
            # plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            # plt.show()

    for cap in caps:
        cap.release()

# def setup():
#     init_cameras()

# def step():


# def main():


if __name__ == '__main__':
    display_main()
