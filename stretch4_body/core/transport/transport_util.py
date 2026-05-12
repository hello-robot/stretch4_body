import serial
import time
import struct
import array as arr
import stretch4_body.core.transport.cobbs_framing as cobbs_framing
import copy
import fcntl
import logging
import math
import threading
import aioserial
import asyncio

"""

Loop protocol is:



RPC Data is sent over a COBBS encoding with CRC error detection.

The  packet is:

Data can be up to X bytes.

Data is manually packed / unpacked into dictionaries (Python) and C-structs (Arduino).
Care should be taken that the pack/unpack size and types are consistent between the two.
This is not automated.
"""


class TransportError(Exception):
    """Base class for exceptions in this module."""
    pass

# //////////////////////////////  Shared Defines ///////////////////////////////////////////////////
RPC_DATA_MAX_BYTES_SAMD51 = 8194 #SAMD51 needs 8KB of space for 2048 floats + rpc ID
RPC_TRANSPORT_VERSION_0 = 0
RPC_TRANSPORT_VERSION_1 = 1


# //////////////////////////////  Shared Defines ///////////////////////////////////////////////////

def pack_string_t(s, sidx, x):
    n = len(x)
    return struct.pack_into(str(n) + 's', s, sidx, x)


def unpack_string_t(s, n):
    return (struct.unpack(str(n) + 's', s[:n])[0].strip(b'\x00')).decode('utf-8')


def unpack_int32_t(s):
    return struct.unpack('i', s[:4])[0]


def unpack_uint32_t(s):
    return struct.unpack('I', s[:4])[0]


def unpack_int64_t(s):
    return struct.unpack('q', s[:8])[0]


def unpack_uint64_t(s):
    return struct.unpack('Q', s[:8])[0]


def unpack_int16_t(s):
    return struct.unpack('h', s[:2])[0]


def unpack_uint16_t(s):
    return struct.unpack('H', s[:2])[0]


def unpack_uint8_t(s):
    return struct.unpack('B', s[:1])[0]


def unpack_float_t(s):
    return struct.unpack('f', s[:4])[0]


def unpack_double_t(s):
    return struct.unpack('d', s[:8])[0]


def pack_float_t(s, sidx, x):
    return struct.pack_into('f', s, sidx, x)


def pack_double_t(s, sidx, x):
    return struct.pack_into('d', s, sidx, x)


def pack_int32_t(s, sidx, x):
    return struct.pack_into('i', s, sidx, x)


def pack_uint32_t(s, sidx, x):
    return struct.pack_into('I', s, sidx, x)


def pack_int16_t(s, sidx, x):
    return struct.pack_into('h', s, sidx, x)


def pack_uint16_t(s, sidx, x):
    return struct.pack_into('H', s, sidx, x)


def pack_uint8_t(s, sidx, x):
    return struct.pack_into('B', s, sidx, x)



















