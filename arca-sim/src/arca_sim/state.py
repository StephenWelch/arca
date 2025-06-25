import numpy as np
import io
import msgpack
from dataclasses import dataclass


@dataclass
class LowLevelCommand:
    pos: np.ndarray
    vel: np.ndarray
    current: np.ndarray
    kp: np.ndarray
    kd: np.ndarray

    def to_bytes(self):
        buf = io.BytesIO()
        buf.write(msgpack.packb(self.pos.tolist()))
        buf.write(msgpack.packb(self.vel.tolist()))
        buf.write(msgpack.packb(self.current.tolist()))
        buf.write(msgpack.packb(self.kp.tolist()))
        buf.write(msgpack.packb(self.kd.tolist()))
        return buf.getvalue()

    @classmethod
    def from_bytes(cls, data: bytes):
        buf = io.BytesIO(data)
        unpacker = msgpack.Unpacker(buf)
        pos = np.array(unpacker.unpack())
        vel = np.array(unpacker.unpack())
        current = np.array(unpacker.unpack())
        kp = np.array(unpacker.unpack())
        kd = np.array(unpacker.unpack())
        return cls(pos, vel, current, kp, kd)


@dataclass
class LowLevelState:
    gyro: np.ndarray
    quat: np.ndarray
    pos: np.ndarray
    vel: np.ndarray
    current: np.ndarray

    def to_bytes(self):
        buf = io.BytesIO()
        buf.write(msgpack.packb(self.gyro.tolist()))
        buf.write(msgpack.packb(self.quat.tolist()))
        buf.write(msgpack.packb(self.pos.tolist()))
        buf.write(msgpack.packb(self.vel.tolist()))
        buf.write(msgpack.packb(self.current.tolist()))
        return buf.getvalue()

    @classmethod
    def from_bytes(cls, data: bytes):
        buf = io.BytesIO(data)
        unpacker = msgpack.Unpacker(buf)
        gyro = np.array(unpacker.unpack())
        quat = np.array(unpacker.unpack())
        pos = np.array(unpacker.unpack())
        vel = np.array(unpacker.unpack())
        current = np.array(unpacker.unpack())
        return cls(gyro, quat, pos, vel, current)


@dataclass
class HighLevelCommand:
    pos: np.ndarray
    vel: np.ndarray
    torque: np.ndarray


@dataclass
class HighLevelState:
    gyro: np.ndarray
    quat: np.ndarray
    pos: np.ndarray
    vel: np.ndarray
    torque: np.ndarray
