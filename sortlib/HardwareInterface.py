import serial
import logging
from multiprocessing import Process, Value, Array, Queue
from ctypes import c_bool


class HardwareInterface:

    MOTOR_OFFSETS = {
        0: (0, 90, 180, 270),
        1: (0, 90, 180, 270),
        2: (0, 90, 180, 270),
    }
    NUM_POS = 4

    def __init__(self, port="/dev/ttyASM0", baudrate="38400"):
        self.ser = serial.Serial(port, baudrate)
        self._cur_pos = 0
        self.sort_and_advance(0)

        self._sorter_ready = Value(c_bool, False)
        self._elevator_arrived = Value(c_bool, False)
        self._servos_arrived = Value(c_bool, False)
        self._motor_positions = Array("i", [0] * len(self.MOTOR_OFFSETS))
        self._msg_queue = Queue()

    def _ser_write(self, cmd):
        self._msg_queue.put(cmd)

    def _set_pos(self, angle, motor_id=None):
        if motor_id is None:
            self._ser_write("G {}".format(angle))
        else:
            self._ser_write("G{} {}".format(motor_id, angle))

    def _set_sort(self, label):
        self._ser_write("G {}\n".format(label))

    @property
    def sorter_ready(self):
        return self._sorter_ready

    @sorter_ready.setter
    def sorter_ready(self, value):
        self._sorter_ready = value

    @property
    def elevator_arrived(self):
        return self._elevator_arrived

    @property
    def servos_arrived(self):
        return self._servos_arrived

    def sort_and_advance(self, label):
        self._set_sort(label)
        self._cur_pos = (self._cur_pos + 1) % (self.NUM_POS - 1)
        for motor_id in self.MOTOR_OFFSETS:
            self._set_pos(self.MOTOR_OFFSETS[motor_id][self.cur_pos], motor_id)

    def start(self):
        self.p = Process(target=self.run)
        self.stop_run = False
        self.p.start()

    def stop(self):
        self.p.terminate()
        self.p.join()

    def _run(self):
        ser = self.ser
        while True:
            line = ser.readline()
            try:
                msg = self._msg_queue.get(False)
                ser.write(msg.encode())
            except Queue.Empty:
                pass

            logging.debug(line)
            if "SA" in line[0:3]:
                self._servos_arrived = True
                els = line.split(";")
                for idx, m in enumerate(self._motor_positions):
                    self._motor_positions[idx] = els[idx]

            if "EA" in line[0:3]:
                self._elevator_arrived = True

            if "ST" in line[0:3]:
                line = line.split(":")[1]
                els = line.split(";")
                for idx, m in enumerate(self._motor_positions):
                    self._motor_positions[idx] = els[idx].split("_")[0]

            if self._sorter_ready:
                ser.write("R".encode())  # Declare sorter ready
                ser.write("A".encode())  # Start elevator (no effect if active)
                self._sorter_ready = False
