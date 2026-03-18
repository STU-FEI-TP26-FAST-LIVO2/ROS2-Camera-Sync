#!/usr/bin/env python3
import mmap
import os
import struct
from typing import Optional, Tuple

MAGIC = b"LIVSYNC1"
VERSION = 1
PACKET_STRUCT = struct.Struct("<8sIIQQQ")
PACKET_SIZE = PACKET_STRUCT.size


def stamp_to_ns(stamp_msg) -> int:
    return int(stamp_msg.sec) * 1_000_000_000 + int(stamp_msg.nanosec)


def ns_to_sec_nsec(ns: int) -> Tuple[int, int]:
    sec = ns // 1_000_000_000
    nanosec = ns % 1_000_000_000
    return sec, nanosec


class SharedStampWriter:
    def __init__(self, path: str):
        self.path = path
        os.makedirs(os.path.dirname(path), exist_ok=True)
        fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
        try:
            os.ftruncate(fd, PACKET_SIZE)
            self._mmap = mmap.mmap(fd, PACKET_SIZE, access=mmap.ACCESS_WRITE)
        finally:
            os.close(fd)

    def write(self, stamp_ns: int, write_time_ns: int, seq: int = 0):
        payload = PACKET_STRUCT.pack(MAGIC, VERSION, 0, int(stamp_ns), int(write_time_ns), int(seq))
        self._mmap.seek(0)
        self._mmap.write(payload)
        self._mmap.flush()

    def close(self):
        if getattr(self, '_mmap', None) is not None:
            self._mmap.close()
            self._mmap = None


class SharedStampReader:
    def __init__(self, path: str):
        self.path = path
        self._mmap = None
        if not os.path.exists(path):
            return
        fd = os.open(path, os.O_RDONLY)
        try:
            self._mmap = mmap.mmap(fd, PACKET_SIZE, access=mmap.ACCESS_READ)
        finally:
            os.close(fd)

    def read(self) -> Optional[Tuple[int, int, int]]:
        if self._mmap is None:
            return None
        self._mmap.seek(0)
        raw = self._mmap.read(PACKET_SIZE)
        if len(raw) != PACKET_SIZE:
            return None
        magic, version, _reserved, stamp_ns, write_time_ns, seq = PACKET_STRUCT.unpack(raw)
        if magic != MAGIC or version != VERSION:
            return None
        return int(stamp_ns), int(write_time_ns), int(seq)

    def close(self):
        if self._mmap is not None:
            self._mmap.close()
            self._mmap = None
