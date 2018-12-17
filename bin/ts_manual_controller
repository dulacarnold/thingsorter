#!/usr/bin/env python3
from sortlib.HardwareInterface import HardwareInterface
# import sys
import logging
import time

T_ELEV_DROP = 0.7
T_PROCESS_TIME = 1

def wait(blocker):
    while not blocker:
        time.sleep(0.05)

def main(args, logger):
    hwi = HardwareInterface(port="/dev/ttyACM0")
    hwi.start()
    time.sleep(2)
    label = 1
    # Initialize wheels
    hwi.sort_and_advance(0)
    hwi.sorter_ready = True
    logger.info("Initial elevator wait...")
    # while not hwi.elevator_arrived:
    #     time.sleep(0.05)
    logger.info("Starting main loop:")
    while True:
        label = 0
        time.sleep(T_PROCESS_TIME)
        logger.info("Trigger sort and advance")
        hwi.sort_and_advance(label)
        logger.info("Waiting on servo arrival..")
        # Wait for servos to finish turning
        while not hwi.servos_arrived: 
            time.sleep(0.05)
        # In case it hasn't arrived yet, should return immediately
        logger.info("Waiting on elevator...")
        while not hwi.elevator_arrived:
            time.sleep(0.05)
        logger.info("Send sorter ready")
        hwi.sorter_ready = True
        time.sleep(T_ELEV_DROP)


    # while True:

    #     # Turn 

    #     # Simulate labeling
    #     # label = input("--> ")
    #     # print(label)
    #     time.sleep(0.5)
    #     logger.info("Setting sort to {} and advancing line.".format(label))
    #     hwi.sort_and_advance(label)
    #     logger.info("Waiting on servo arrival...")
    #     while not hwi.servos_arrived:
    #         time.sleep(0.05)
    #     logger.info("Servos arrived.")
    #     # Activate elevator
    #     logger.info("Activating elevator and waiting on arrival.")
    #     hwi.sorter_ready = True
    #     while not hwi.elevator_arrived:
    #         time.sleep(0.05)
    #     logger.info("Elevator arrived.")
    #     logger.info(hwi.motor_positions)
    #     time.sleep(0.45)  # Make sure everything is stabilized


if __name__ == "__main__":
    numeric_level = 5
    logger = logging.getLogger("manual_controller")
    try:
        import coloredlogs
        coloredlogs.install(level=numeric_level)
    except ImportError:
        logging.basicConfig(level=numeric_level)
    logger.setLevel(numeric_level)
    logger.info("Starting main...")
    args = []
    main(args, logger)
