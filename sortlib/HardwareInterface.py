import serial
import logging
from multiprocessing import Process, Value, Array, Queue
from queue import Empty
from ctypes import c_bool
import time


class HardwareInterface:

    MOTOR_OFFSETS = {
        0: (315, 50, 135, 225),
        1: (0, 88, 178, 266),
        2: (345, 85, 165, 255),
    }
    NUM_POS = 4

    def __init__(
        self, port="/dev/ttyACM0", baudrate="38400", motor_offsets=None
    ):
        self.port = port
        self.baudrate = baudrate
        self._cur_pos = 0
        self._sorter_ready = Value(c_bool, False)
        self._elevator_arrived = Value(c_bool, False)
        self._servos_arrived = Value(c_bool, False)
        self._motor_positions = Array("i", [0] * len(self.MOTOR_OFFSETS))
        self._msg_queue = Queue()
        self._logger = logging.getLogger(__name__)
        if motor_offsets is not None:
            self.MOTOR_OFFSETS = motor_offsets
        time.sleep(1)
        self._zero_motors()

    def _zero_motors(self):
        self._set_sort(0)
        self._cur_pos = 0
        for motor_id in self.MOTOR_OFFSETS:
            self._set_pos(self.MOTOR_OFFSETS[motor_id][self._cur_pos], motor_id)

    def sort_and_advance(self, label):
        """ Set the sort direction and advance blades. """
        self._set_sort(label)
        self._cur_pos = (self._cur_pos + 1) % (self.NUM_POS)
        for motor_id in self.MOTOR_OFFSETS:
            self._set_pos(self.MOTOR_OFFSETS[motor_id][self._cur_pos], motor_id)

    def _set_pos(self, angle, motor_id=None):
        if motor_id is None:
            self._ser_write("G {}\r\n".format(angle))
        else:
            self._ser_write("G{} {}\r\n".format(motor_id, angle))

    def _set_sort(self, label):
        self._ser_write("S {}\r\n".format(label))

    def _ser_write(self, cmd):
        self._msg_queue.put(cmd)

    @property
    def sorter_ready(self):
        """
        Token variable, gets set to false once consumed.
        """
        if self._sorter_ready.value:
            self.sorter_ready = False
            return True
        else:
            return False

    @sorter_ready.setter
    def sorter_ready(self, value):
        self._sorter_ready.value = value

    @property
    def elevator_arrived(self):
        """
        Token variable, gets set to false once consumed.
        """
        if self._elevator_arrived.value:
            self.elevator_arrived = False
            return True
        else:
            return False

    @elevator_arrived.setter
    def elevator_arrived(self, value):
        self._elevator_arrived.value = value

    @property
    def servos_arrived(self):
        if self._servos_arrived.value:
            self.servos_arrived = False
            return True
        else:
            return False

    @servos_arrived.setter
    def servos_arrived(self, value):
        self._servos_arrived.value = value

    @property
    def motor_positions(self):
        return tuple(self._motor_positions)

    def start(self):
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0)
        self._p = Process(target=self._run)
        self._p.start()

    def stop(self):
        self._p.terminate()
        self._p.join()
        self.ser.close()

    def __del__(self):
        self.stop()

    def _run(self):
        ser = self.ser
        b_line = b""
        while True:
            # If there was already a full line, overwrite it:
            if b"\n" in b_line:
                b_line = ser.readline()
            # Otherwise, append whatever is in the buffer
            else:
                b_line += ser.readline()
            # If we've received or completed a full line, parse it
            if b"\n" in b_line:
                self._logger.debug("Received: {}".format(b_line))
                line = b_line.decode("ascii")
                # self._logger.debug("Decoded: {}".format(line))
                if "SA" in line[0:3]:
                    self.servos_arrived = True
                    els = line.split(":")[1].split(";")
                    for idx, m in enumerate(self._motor_positions):
                        self._motor_positions[idx] = int(els[idx])

                if "ET" in line[0:3]:
                    self.elevator_arrived = True

                if "ST" in line[0:3]:
                    els = line.split(":")[1].split(";")
                    for idx, m in enumerate(self._motor_positions):
                        self._motor_positions[idx] = int(els[idx].split("_")[0])

            # Write out messages from Queue
            try:
                msg = self._msg_queue.get(False)
                ser.write(msg.encode())
                self._logger.debug("Sent {}".format(msg.encode()))
            except Empty:
                pass

            # If flags have changed send updates
            if self.sorter_ready:
                # Declare sorter ready
                # ser.write("R\r\n".encode())
                # Start elevator (set sorter_ready if already active)
                ser.write("A\r\n".encode())
                self._logger.debug("Sent {}".format("A\r\n".encode()))
