"""Minimal Linux I2C/BMI160 reader adapted from the existing IMU test driver."""
import ctypes
import fcntl
import os
import struct
import time
from typing import Optional

I2C_RDWR = 0x0707
I2C_M_RD = 0x0001

class _I2CMsg(ctypes.Structure):
    _fields_ = [
        ("addr", ctypes.c_uint16),
        ("flags", ctypes.c_uint16),
        ("len", ctypes.c_uint16),
        ("buf", ctypes.POINTER(ctypes.c_uint8)),
    ]

class _I2CRdwrIoctlData(ctypes.Structure):
    _fields_ = [
        ("msgs", ctypes.POINTER(_I2CMsg)),
        ("nmsgs", ctypes.c_uint32),
    ]

class LinuxI2CBus:
    """Linux combined I2C transactions."""

    def __init__(self, bus_num: int) -> None:
        self.bus_num = bus_num
        self.dev_path = f"/dev/i2c-{bus_num}"
        self.fd: Optional[int] = None

    def open(self) -> None:
        if self.fd is None:
            self.fd = os.open(self.dev_path, os.O_RDWR)

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def __enter__(self) -> "LinuxI2CBus":
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    def write_reg(self, addr: int, reg: int, value: int) -> None:
        if self.fd is None:
            raise RuntimeError("I2C bus is not open")
        wbuf = (ctypes.c_uint8 * 2)(reg & 0xFF, value & 0xFF)
        msg = _I2CMsg(addr=addr, flags=0, len=2, buf=wbuf)
        ioctl_data = _I2CRdwrIoctlData(msgs=ctypes.pointer(msg), nmsgs=1)
        fcntl.ioctl(self.fd, I2C_RDWR, ioctl_data)

    def read_regs(self, addr: int, reg: int, length: int) -> bytes:
        if self.fd is None:
            raise RuntimeError("I2C bus is not open")
        wbuf = (ctypes.c_uint8 * 1)(reg & 0xFF)
        rbuf = (ctypes.c_uint8 * length)()
        msgs = (_I2CMsg * 2)(
            _I2CMsg(addr=addr, flags=0, len=1, buf=wbuf),
            _I2CMsg(addr=addr, flags=I2C_M_RD, len=length, buf=rbuf),
        )
        ioctl_data = _I2CRdwrIoctlData(msgs=msgs, nmsgs=2)
        fcntl.ioctl(self.fd, I2C_RDWR, ioctl_data)
        return bytes(rbuf)

    def read_reg(self, addr: int, reg: int) -> int:
        return self.read_regs(addr, reg, 1)[0]

class BMI160:
    """100 Hz, +/-4 g and +/-500 deg/s; units converted at acquisition."""
    def __init__(self, bus, address):
        self.bus, self.address = bus, address

    def initialize(self):
        if self.bus.read_reg(self.address, 0x00) != 0xD1:
            raise OSError("BMI160 chip ID mismatch")
        for value, delay in ((0xB6, .05), (0x11, .02), (0x15, .1)):
            self.bus.write_reg(self.address, 0x7E, value)
            time.sleep(delay)
        for register, value in ((0x40, 0x28), (0x41, 0x05), (0x42, 0x28), (0x43, 0x02)):
            self.bus.write_reg(self.address, register, value)
        time.sleep(.02)
        if self.bus.read_reg(self.address, 0x03) & 0x3C != 0x14:
            raise OSError("BMI160 accelerometer/gyro not in normal mode")
        if self.bus.read_reg(self.address, 0x02):
            raise OSError("BMI160 configuration error")

    def read(self):
        if self.bus.read_reg(self.address, 0x1B) & 0xC0 != 0xC0:
            return None
        raw = struct.unpack('<6h', self.bus.read_regs(self.address, 0x0C, 12))
        temp, = struct.unpack('<h', self.bus.read_regs(self.address, 0x20, 2))
        return {'accel_m_s2': [v / 8192 * 9.80665 for v in raw[3:]],
                'gyro_deg_s': [v / 65.6 for v in raw[:3]],
                'temperature_c': None if temp == -32768 else 23 + temp / 512,
                'saturated': any(abs(v) >= 32760 for v in raw)}
