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
import numpy as np
import scipy.misc
import pathlib
from collections import defaultdict

from IPython import embed


logging.basicConfig()
logger = logging.getLogger("image_sink")
NUM_CAMS = 3


class FixSizeDefaultStrDict(defaultdict):
    def __init__(self, *args, max_len=0, **kwargs):
        self._max = max_len
        super().__init__(*args, **kwargs)

    def __setitem__(self, key, value):
        if type(key) != str:
            raise ValueError("Only string keys accepted.")
        defaultdict.__setitem__(self, key, value)
        if self._max > 0:
            if len(self) > self._max:
                min_key = min(self.keys(), key=float)  # Remove oldest key
                old_item = self.pop(min_key)
                logger.debug("Removed key {} from {}.".format(min_key, self))


def get_sig_handler(master, sink, monitor, context):
    def signal_handler(sig, frame):
        master.close()
        sink.close()
        monitor.close()
        context.term()
        sys.exit(0)

    return signal_handler


def recv_array(socket, flags=0, copy=True, track=False):
    """recv a numpy array"""
    md = socket.recv_json(flags=flags)
    msg = socket.recv(flags=flags, copy=copy, track=track)
    A = np.frombuffer(msg, dtype=md["dtype"])
    return md["timestamp"], int(md["video_device"]), A.reshape(md["shape"])


def get_oldest_unprocessed(image_queue):
    """ Get the oldest key in the queue which references a full unprocessed
    series of photos.  A full unprocessed series is one where all the cameras have
    taken their picture.
    The key will corresond to the chain starting with the zero-th camera, so
    a key K will reference image_queue[K][0], image_queue[K+1][1], image_queue[K][2]...

    """
    # Check if the oldest key describes an unprocessed chain, and move back from there
    oldest_unprocessed = None
    sorted_keys = sorted(image_queue.keys(), key=float)
    cam_id = 2
    # For each timestamp, check if it ends with a valid picture chain
    for idx, key in enumerate(sorted_keys):
        idx_good = True
        # Check that image_queue[i][j] is valid for cam_id = 1..NUM_CAMS and idx = t..t-NUM_CAMS
        # if that is the case then we have a valid image chain we can extract later
        for j, cam_id in enumerate(range(0, NUM_CAMS)):
            try:
                if idx_good and (
                    image_queue[sorted_keys[idx + j]][cam_id] is not None
                ):
                    idx_good = True
                else:
                    idx_good = False
            except (KeyError, IndexError):
                idx_good = False
        if idx_good:
            oldest_unprocessed = key
            break
    return oldest_unprocessed


def extract_image_chain(image_queue, image_key):
    """ Extract a list of images of the same chestnut, going backwards.
    The key will corresond to the chain starting with the zero-th camera, so
    a key K will reference image_queue[K][0], image_queue[K+1][1], image_queue[K][2]...
    """
    images = []
    sorted_keys = sorted(image_queue.keys(), key=float)
    idx = sorted_keys.index(image_key)
    for j, cam_id in enumerate(range(NUM_CAMS)):
        try:
            images.append(image_queue[sorted_keys[idx + j]][cam_id])
            del image_queue[sorted_keys[idx + j]][cam_id]
        except IndexError:
            raise ValueError(
                "Couldn't generate chain for key: {}".format(timestamp)
            )
    return images


def save_image_chain(save_dir, key, image_chain, label):
    dir_name = os.path.join("./", save_dir, label)
    logger.info("Saving files to {}".format(dir_name))
    pathlib.Path(dir_name).mkdir(parents=True, exist_ok=True)
    for idx, img in enumerate(image_chain):
        scipy.misc.imsave(
            os.path.join(dir_name, "{}_{}.jpg".format(key, idx)), img
        )


def main(args, logger):

    # Set up the zmq sockets:
    context = zmq.Context()
    master = context.socket(zmq.SUB)
    master.connect(args.master_address)
    master.setsockopt(zmq.SUBSCRIBE, b"")

    sink = context.socket(zmq.PULL)
    sink.bind(args.sink_address)

    monitor = context.socket(zmq.PUSH)
    monitor.connect(args.monitor_address)
    monitor.setsockopt(zmq.LINGER, 0)

    poller = zmq.Poller()
    poller.register(master, zmq.POLLIN)
    poller.register(sink, zmq.POLLIN)

    signal.signal(
        signal.SIGINT, get_sig_handler(master, sink, monitor, context)
    )

    image_queue = FixSizeDefaultStrDict(dict, max_len=10)
    label_queue = FixSizeDefaultStrDict(None, max_len=20)

    logger.debug("Entering main loop.")
    do_quit = False

    while True:
        events = dict(poller.poll())

        if sink in events:
            timestamp, video_device, img = recv_array(sink)
            image_queue[timestamp][video_device] = img
            logger.info("Received image for ts {}".format(timestamp))

        if master in events:
            m_msg = master.recv_json()
            logger.info("Received master for ts={}".format(m_msg["timestamp"]))
            label_queue[m_msg["timestamp"]] = m_msg["label"]
            if m_msg["control"] == "QUIT":
                break

        save_key = get_oldest_unprocessed(image_queue)
        if save_key is not None:
            logging.info("Saving images for ts={}".format(save_key))
            image_chain = extract_image_chain(image_queue, save_key)
            try:
                label = label_queue[save_key]
            except KeyError:
                label = "UNK"
            save_image_chain(args.save_dir, save_key, image_chain, label)
            monitor.send_json({"timestamp": save_key, "label": "SAVE"})

    # while True:

    #     # Store received images in the image queue
    #     timestamp, video_device, img = recv_array(sink)
    #     image_queue[timestamp][video_device] = img
    #     logger.info("Received image for ts {}".format(timestamp))
    #     # We can do an infinite loop as long as the master freq < timeout
    #     while True:
    #         try:
    #             m_msg = master.recv_json()
    #             logger.info(
    #                 "Received master message for ts={}".format(
    #                     m_msg["timestamp"]
    #                 )
    #             )
    #             label_queue[m_msg["timestamp"]] = m_msg["label"]
    #             if m_msg["control"] == "QUIT":
    #                 do_quit = False
    #         except zmq.Again:
    #             logger.debug("No messages from master.")
    #             # time.sleep(0.1)
    #             break
    #     if do_quit:
    #         break

    #     # Get the oldest key with valid chain and save it to disk
    #     save_key = get_oldest_unprocessed(image_queue)

    #     logging.info("Saving images for ts={}".format(save_key))
    #     if save_key is not None:
    #         image_chain = extract_image_chain(image_queue, save_key)
    #         try:
    #             label = label_queue[save_key]
    #         except KeyError:
    #             label = "UNK"
    #         save_image_chain(args.save_dir, save_key, image_chain, label)

    #     # Tell the monitor we've processed this timestamp
    #     monitor.send_json({"timestamp": save_key})


def parse_args():
    parser = argparse.ArgumentParser(
        description="Grab camera images and forward to sink."
    )
    optional = parser._action_groups.pop()
    required = parser.add_argument_group("required arguments")
    required.add_argument("--save_dir", required=True)
    optional.add_argument("--master_address", default="tcp://localhost:5556")
    optional.add_argument("--sink_address", default="tcp://127.0.0.1:5558")
    optional.add_argument("--monitor_address", default="tcp://127.0.0.1:5559")
    optional.add_argument("--log_level", default="DEBUG")
    parser._action_groups.append(optional)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

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
