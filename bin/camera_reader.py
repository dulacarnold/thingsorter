#!/usr/bin/env python3
import cv2
import json
import zmq
import argparse
import logging
import signal
import sys
import os
import time
import configparser

from IPython import embed


def get_sig_handler(cap, master, sink, context):
    def signal_handler(sig, frame):
        print("You pressed Ctrl+C!")
        cap.release()
        master.close()
        sink.close()
        context.term()
        sys.exit(0)

    return signal_handler


def open_cam_usb(dev, width, height):
    gst_str = (
        "v4l2src device={} ! "
        "image/jpeg,widh=(int){},height=(int){},framerate=10/1,"
        "format=(string)RGB ! "
        "jpegdec ! videoconvert ! appsink"
    ).format(dev, width, height)
    return cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)


def send_array(
    socket, timestamp, camera_number, A, flags=0, copy=True, track=False
):
    """send a numpy array with metadata"""
    md = dict(
        dtype=str(A.dtype),
        shape=A.shape,
        timestamp=timestamp,
        video_device=int(camera_number),
    )
    socket.send_json(md, flags | zmq.SNDMORE)
    return socket.send(A, flags, copy=copy, track=track)


def main(args, logger):

    # Set up the zmq sockets:
    context = zmq.Context()
    master = context.socket(zmq.SUB)
    master.connect(args.master_address)
    master.setsockopt(zmq.SUBSCRIBE, b"")

    sink = context.socket(zmq.PUSH)
    sink.connect(args.sink_address)
    sink.setsockopt(zmq.LINGER, 0)

    cap = open_cam_usb(args.video_device, args.width, args.height)
    signal.signal(signal.SIGINT, get_sig_handler(cap, master, sink, context))

    while True:
        ret, frame = cap.read()
        if ret is False:
            logger.error("Missed a frame.")
            break
        logger.debug("Captured frame.")
        try:
            msg = master.recv_json(flags=zmq.NOBLOCK)
        except zmq.Again:
            continue

        if msg["control"] == "QUIT":
            break
        logger.info("Sending image for timestamp: {}".format(msg["timestamp"]))
        send_array(sink, msg["timestamp"], args.camera_number, frame.copy())

    cap.release()


def parse_args():

    # p = configargparse.ArgParser(default_config_files=['/etc/thingsorter/thingsorter.conf'])

    # p.add('-c', '--config', required=True, is_config_file=True, help="config file")
    # p.add('--video_device', help="video device path")
    # p.add('--camera_list')
    # p.add('--video_id', required=True, help="camera number")
    # p.add('--master_address')
    # p.add('--sink_address')
    # p.add('--width')
    # p.add('--height')

    # return p.parse_args()
    parser = argparse.ArgumentParser(
        description="Grab camera images and forward to sink."
    )
    optional = parser._action_groups.pop()
    required = parser.add_argument_group("required arguments")
    required.add_argument("--camera_number", type=int, required=True)
    required.add_argument("-c", "--config_file", default="/home/gabriel/Src/TechnoTrieuse/thingsorter.ini")
    # optional.add_argument("--master_address", default="tcp://localhost:5556")
    # optional.add_argument("--sink_address", default="tcp://localhost:5558")
    optional.add_argument("--log_level", default="DEBUG")
    # optional.add_argument("--width", default=1280)
    # optional.add_argument("--height", default=720)
    parser._action_groups.append(optional)

    args = parser.parse_args()
    config = configparser.ConfigParser()
    config.read(args.config_file)
    args.video_device = (
        config["cameras"]["camera_prefix"]
        + config["cameras"]["camera_list"]
        .split(",")[args.camera_number]
        .strip()
    )
    args.width = config["cameras"]["width"]
    args.height = config["cameras"]["height"]
    args.master_address = config["addresses"]["master_address"]
    args.sink_address = config["addresses"]["sink_address"]
    return args


if __name__ == "__main__":
    args = parse_args()
    logging.basicConfig()
    logger = logging.getLogger("camera_reader_{}".format(args.camera_number))
    numeric_level = getattr(logging, args.log_level.upper())
    if not isinstance(numeric_level, int):
        raise ValueError("Invalid log level: {}".format(args.log_level))
    try:
        import coloredlogs

        coloredlogs.install(level=numeric_level)
    except ImportError:
        logging.basicConfig(level=numeric_level)
    logger.info("Starting main...")
    main(args, logger)
