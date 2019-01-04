import struct
import threading
import time

import serial

from pydobot.message import Message


def retry(f):
    def dec(*a, **kw):
        while True:
            try:
                return f(*a, **kw)
            except:
                time.sleep(0.05)
    return dec


MODE_PTP_JUMP_XYZ = 0x00
MODE_PTP_MOVJ_XYZ = 0x01
MODE_PTP_MOVL_XYZ = 0x02
MODE_PTP_JUMP_ANGLE = 0x03
MODE_PTP_MOVJ_ANGLE = 0x04
MODE_PTP_MOVL_ANGLE = 0x05
MODE_PTP_MOVJ_INC = 0x06
MODE_PTP_MOVL_INC = 0x07
MODE_PTP_MOVJ_XYZ_INC = 0x08
MODE_PTP_JUMP_MOVL_XYZ = 0x09

GET_POSE = 10
SET_HOME_CMD = 31
SET_END_EFFECTOR_PARAMS = 60
SET_END_EFFECTOR_SUCTION_CUP = 62
SET_PTP_COORDINATE_PARAMS = 81
SET_PTP_COMMON_PARAMS = 83
SET_PTP_CMD = 84
SET_CP_CMD = 91
GET_QUEUED_CMD_CURRENT_INDEX = 246


class Dobot(threading.Thread):
    on = True
    x = 0.0
    y = 0.0
    z = 0.0
    r = 0.0
    j1 = 0.0
    j2 = 0.0
    j3 = 0.0
    j4 = 0.0

    # joint_angles = [4]

    def __init__(self, port, verbose=False):
        threading.Thread.__init__(self)
        self.verbose = verbose
        self.lock = threading.Lock()
        self.ser = serial.Serial(port,
                                 baudrate=115200,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 bytesize=serial.EIGHTBITS)
        self.closed = threading.Event()
        is_open = self.ser.isOpen()
        if self.verbose:
            print('pydobot: %s open' % self.ser.name if is_open else 'failed to open serial port')
        self._set_ptp_coordinate_params(velocity=200.0, acceleration=200.0)
        self._set_ptp_common_params(velocity=200.0, acceleration=200.0)
        self._get_pose()
        self.start()

    def run(self):
        while self.on:
            try:
                self._get_pose()
            except AttributeError:
                pass  # Don't die on attribute errors in this thread since they should be transient
            time.sleep(0.2)
        self.lock.acquire()
        self.ser.close()
        if self.verbose:
            print('pydobot: %s closed' % self.ser.name)
        self.lock.release()
        self.closed.set()

    def close(self):
        self.on = False
        self.closed.wait(1)

    def wait_for_queue_to_drain(self):
        while self._get_queued_cmd_current_index() < self.queue_pos:
            time.sleep(0.1)

    def _send_command(self, msg, sync=False):
        with self.lock:
            self._send_message(msg)
            response = self._read_message()
        # for some reason HOME returns nothing contrary to documentation, so it's always considered async
        if msg.queued and msg.id != SET_HOME_CMD:
            self.queue_pos = struct.unpack("L", response.params[-8:])[0]
            if sync:
                self.wait_for_queue_to_drain()
        return response

    def _send_message(self, msg):
        time.sleep(0.1)
        if self.verbose:
            print('pydobot: >>', msg)
        self.ser.write(msg.bytes())

    def _read_message(self):
        time.sleep(0.1)
        b = self.ser.read_all()
        if len(b) > 0:
            msg = Message(b)
            if self.verbose:
                print('pydobot: <<', msg)
            return msg
        return

    def _get_pose(self):
        msg = Message()
        msg.id = GET_POSE
        response = self._send_command(msg)
        self.x = struct.unpack_from('f', response.params, 0)[0]
        self.y = struct.unpack_from('f', response.params, 4)[0]
        self.z = struct.unpack_from('f', response.params, 8)[0]
        self.r = struct.unpack_from('f', response.params, 12)[0]
        self.j1 = struct.unpack_from('f', response.params, 16)[0]
        self.j2 = struct.unpack_from('f', response.params, 20)[0]
        self.j3 = struct.unpack_from('f', response.params, 24)[0]
        self.j4 = struct.unpack_from('f', response.params, 28)[0]
        if self.verbose:
            print("pydobot: x:%03.1f y:%03.1f z:%03.1f r:%03.1f j1:%03.1f j2:%03.1f j3:%03.1f j4:%03.1f" %
                  (self.x, self.y, self.z, self.r, self.j1, self.j2, self.j3, self.j4))
        return response

    def _set_cp_cmd(self, x, y, z):
        msg = Message()
        msg.id = SET_CP_CMD
        msg.ctrl = 0x03
        msg.params = bytearray(bytes([0x01]))
        msg.params.extend(bytearray(struct.pack('f', x)))
        msg.params.extend(bytearray(struct.pack('f', y)))
        msg.params.extend(bytearray(struct.pack('f', z)))
        msg.params.append(0x00)
        return self._send_command(msg)

    def _set_ptp_coordinate_params(self, velocity, acceleration):
        msg = Message()
        msg.id = SET_PTP_COORDINATE_PARAMS
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        return self._send_command(msg)

    def _set_ptp_common_params(self, velocity, acceleration):
        msg = Message()
        msg.id = SET_PTP_COMMON_PARAMS
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        return self._send_command(msg)

    def _set_ptp_cmd(self, x, y, z, r, mode, sync):
        msg = Message()
        msg.id = SET_PTP_CMD
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray([mode]))
        msg.params.extend(bytearray(struct.pack('f', x)))
        msg.params.extend(bytearray(struct.pack('f', y)))
        msg.params.extend(bytearray(struct.pack('f', z)))
        msg.params.extend(bytearray(struct.pack('f', r)))
        response = self._send_command(msg, sync=sync)
        return response

    def _set_end_effector_suction_cup(self, suck=False):
        msg = Message()
        msg.id = SET_END_EFFECTOR_SUCTION_CUP
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray([0x01]))
        if suck is True:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))
        return self._send_command(msg)

    def _set_home_cmd(self):
        msg = Message()
        msg.id = SET_HOME_CMD
        msg.ctrl = 0x03
        msg.params = bytearray([])
        self._send_command(msg, sync=False)

    def _set_end_effector_params(self, x, y, z):
        msg = Message()
        msg.id = SET_END_EFFECTOR_PARAMS
        msg.ctrl = 0x02
        msg.params = bytearray([])
        msg.params.extend(struct.pack('f', x))
        msg.params.extend(struct.pack('f', y))
        msg.params.extend(struct.pack('f', z))
        return self._send_command(msg)


    @retry
    def _get_queued_cmd_current_index(self):
        msg = Message()
        msg.id = GET_QUEUED_CMD_CURRENT_INDEX
        response = self._send_command(msg)
        # sometimes (when a command is running?) the returned struct has more than the index, but the index seems to
        # always be the last 8 bytes
        return struct.unpack("L", response.params[-8:])[0]

    def home(self):
        self._set_home_cmd()

    def go(self, x=None, y=None, z=None, r=None, dx=None, dy=None, dz=None, dr=None, sync=True):
        assert not (x and dx) and not (y and dy) and not (z and dz) and not (r and dr), "in any dimension, do not provide both absolute and relative values"
        def update(old, abs, rel):
            if abs is not None:
                return abs
            elif rel is not None:
                return old + rel
            else:
                return old
        x = update(self.x, x, dx)
        y = update(self.y, y, dy)
        z = update(self.z, z, dz)
        r = update(self.r, r, dr)
        self._set_ptp_cmd(x, y, z, r, mode=MODE_PTP_MOVJ_XYZ, sync=sync)

    def set_end_effector_bias(self, x=0, y=0, z=0):
        self._set_end_effector_params(x, y, z)

    def suck(self, suck):
        self._set_end_effector_suction_cup(suck)

    def speed(self, velocity=100., acceleration=100.):
        self._set_ptp_common_params(velocity, acceleration)
        self._set_ptp_coordinate_params(velocity, acceleration)

    # def set_operating_bounds(self, xmin=None, xmax=None, ymin=None, ymax=None, zmin=None, zmax=None, rmin=None, rmax=None):
        # assert xmin < xmax, (xmin, xmax)
        # assert ymin < ymax, (ymin, ymax)
        # assert zmin < zmax, (zmin, zmax)
        # assert rmin < rmax, (rmin, rmax)

        