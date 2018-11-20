import serial
impor logging

class HardwareInterface:

    def __init__(self, port="/dev/ttyASM0", baudrate="38400"):
        self.ser = serial.Serial(port, baudrate)
        self.cur_pos = 0
        self.servos_arrived = Value('d', 0)
        self.elevator_arrived = Value('d', 0)
        self.motor_positions = Array('i', [0, 0, 0])
        self.sleep(1)

        self._last_pos = 0
        self._set_pos(self._last_pos)


    @staticmethod
    def _round_closest(x, base=10):
        return int(base * round(float(x)/base))

    def _ser_monitor(self, servos_arrived, elevator_arrived):
        ser = self.ser
        while True:
            line = ser.readline()

            logging.debug(line)
            if "SA" in line[0:3]:
                servos_arrived = 1
                els = line.split(';')
                for idx, m in enumerate(motor_positions):
                    motor_positions[idx] = els[idx]

            if "EA" in line[0:3]:
                elevator_arrived = 1

            if "ST" in line[0:3]:
                line = line.split(':')[1]
                els = line.split(';')
                for idx, m in enumerate(motor_positions):
                    motor_positions[idx] = els[idx].split('_')[0]

    def send_raw_command(self, cmd):
        ser.write(cmd)
        return ser.readline()

    def _set_pos(self, label):
        ser.write("G{}\n".format(label))

    def _set_sort(self, label)
    def step(self):
        self._set_pos(self._last_pos + 90)
        self._new_pos =  self._last_pos + 90)
        while self.servos_arrived == 0:
            time.sleep(0.1)

    self has_arrived(self):




