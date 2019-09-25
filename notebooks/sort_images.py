#!/usr/bin/env python3

"""Take a directory of images and sort it into a set of directories
according to the given model's predictions.
"""

import argparse
import cv2
import os
import imageio
import pathlib
import logging
from multiprocessing import Pool
from functools import partial
import numpy as np
from tensorflow import keras
from IPython import embed


def load_images_from_path(path):
    ds = []
    filenames = []
    for filename in os.listdir(path):
        img = cv2.imread(os.path.join(path, filename))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        filenames.append(filename)
        ds.append(img)
    return filenames, ds


def main(args, logger):
    logger.info(type(args.num_proc))
    model = keras.models.load_model(args.model)
    source_dir = args.source_dir
    target_dir = args.target_dir

    logger.info("Loading images...")
    filenames, images = load_images_from_path(source_dir)
    logger.info("Done loading.")

    # Figure out what dimensions the net is expecting and resize a copy
    t_width = model.layers[0].get_config()['batch_input_shape'][2]
    t_height = model.layers[0].get_config()['batch_input_shape'][1]
    logger.info("Resizing images...")
    pool = Pool(processes=args.num_proc)
    images_resized = pool.map(partial(cv2.resize, dsize=(t_width, t_height)),
                              images)
    pool.close()
    logger.info("Done resizing.")

    # Run the model
    logger.info("Running model...")
    X_pred = np.array(images_resized) / 255.
    y_pred_cat = model.predict(X_pred)
    y_pred = np.argmax(y_pred_cat, axis=1)
    logger.info("Done with predictions.")
    # embed()

    # Create target directories
    logger.info("Writing to disk...")
    out_dirs = []
    for idx in range(y_pred_cat.shape[1]):
        out_dirs.append(os.path.join(target_dir, '{}'.format(idx)))
        pathlib.Path(out_dirs[idx]).mkdir(parents=True, exist_ok=True)

    # Save files to their respective directory
    for idx, filename in enumerate(filenames):
        out_file = os.path.join(out_dirs[y_pred[idx]], filenames[idx])
        imageio.imwrite(out_file, images[idx])
    logger.info("Done.")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Grab camera images and forward to sink."
    )

    parser.add_argument("model", help="hdf5 model file")
    parser.add_argument("source_dir", help="location of images to be sorted")
    parser.add_argument("target_dir", help="location to sort images to")
    parser.add_argument("--log_level", default="DEBUG")
    parser.add_argument("--num_proc", default=30)
    args = parser.parse_args()
    return args


if __name__ == "__main__":
    args = parse_args()
    logger = logging.getLogger("master")
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
