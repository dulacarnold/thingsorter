from sort_lib import HardwareInterface
# import sys
import logging
import time


def main(args, logger):
    # hwi = HardwareInterface()
    # hwi.start()

    while True:
        label = input("--> ")
        print(label)
        time.sleep(1)
        logging.info("Setting sort to {} and advancing line.".format(label))
        hwi.sort_and_advance(label)
        while not hwi.servos_arrived:
            time.sleep(0.05)
        # Activate elevator
        logging.info("Activating elevator.")
        hwi.sorter_ready = True
        while not hwi.elevator_arrived:
            time.sleep(0.05)
        logging.info(hwi.motor_positions)
        time.sleep(0.25)  # Make sure everything is stabilized


if __name__ == "__main__":
    numeric_level = 5
    logger = logging.getLogger("manual_controller")
    try:
        import coloredlogs
        coloredlogs.install(level=numeric_level)
    except ImportError:
        logging.basicConfig(level=numeric_level)
    # logger.setLevel(numeric_level)
    logger.info("Starting main...")
    args = []
    main(args, logger)
