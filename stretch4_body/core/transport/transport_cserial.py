import ctypes
import atexit
import logging
import array as arr
import struct
from stretch4_body.core.transport.transport_util import *

import os

class TransportCSerial:
    """
    C based serial transport
    The library libtransport.so handles the transport protocol and serial comms
    """
    _lib_path = os.path.join(os.path.dirname(__file__), 'libtransport.so')
    if not os.path.exists(_lib_path):
        # Fallback for meson-python editable installs
        import glob
        _search_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'build', '*', 'libtransport.so'))
        _matches = glob.glob(_search_path)
        if _matches:
            _lib_path = _matches[0]

    lib = ctypes.CDLL(_lib_path)
    RPC_CALLBACK_TYPE = ctypes.CFUNCTYPE(ctypes.c_bool,ctypes.c_int, ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint16)

    lib.start_serial.argtype=[ctypes.c_uint8, ctypes.c_char_p]
    lib.start_serial.restype = ctypes.c_bool

    lib.start_queue_thread.argtype=[ctypes.c_uint8]
    lib.start_queue_thread.restype = ctypes.c_bool

    lib.shutdown_queue_thread.argtype=[ctypes.c_uint8]
    lib.shutdown_queue_thread.restype = ctypes.c_bool


    lib.do_rpc.argtypes=[ctypes.c_bool,ctypes.c_bool,ctypes.c_uint8,ctypes.POINTER(ctypes.c_uint8),ctypes.c_uint16,RPC_CALLBACK_TYPE]
    lib.do_rpc.restype=ctypes.c_int

    lib.load_rpc_result.argtype=[ctypes.c_int, ctypes.c_int,]
    lib.load_rpc_result.restype = ctypes.c_int

    lib.is_rpc_result_ready.argtype=[ctypes.c_int, ctypes.c_int,]
    lib.is_rpc_result_ready.restype = ctypes.c_bool

    lib.get_rpc_duration_us.argtype = [ctypes.c_int, ctypes.c_int, ]
    lib.get_rpc_duration_us.restype = ctypes.c_double



    def __init__(self, port_name, qid, logger=logging.getLogger()):
        self.connected=False
        self.port_name=port_name
        self.qid=qid
        self.logger = logger
        self.status = {}
        self.empty_payload = arr.array('B', [0] * (RPC_DATA_MAX_BYTES_SAMD51 + 1))  # RPC ID + 1024 bytes of data   
        self.payload_type = ctypes.c_uint8 * (RPC_DATA_MAX_BYTES_SAMD51 + 1)
        self.py_callbacks={}
        self.nonblocking_thread_active = False
        self.blocking_callback_func = self.RPC_CALLBACK_TYPE(self._conversion_blocking_callback) #Hold by strong reference to avoid garbage collection
        self.nonblocking_callback_func = self.RPC_CALLBACK_TYPE(self._conversion_nonblocking_callback)

    def startup(self):
        if self.connected:
             return True
        #Start serial
        self.connected=self.lib.start_serial(ctypes.c_uint8(self.qid),ctypes.c_char_p(self.port_name.encode('utf-8')))
        return self.connected

    def start_nonblocking_thread(self):
        if self.lib.start_queue_thread(ctypes.c_uint8(self.qid)):
            self.nonblocking_thread_active = True
            atexit.register(self.stop)
            return True
        return False
    def stop_nonblocking_thread(self):
        if self.nonblocking_thread_active:
            self.lib.shutdown_queue_thread(ctypes.c_uint8(self.qid))
            self.nonblocking_thread_active = False
            return True
        return False

    def stop(self):
        if not self.connected:
            return
        self.stop_nonblocking_thread()
        self.connected = False

    def _conversion_blocking_callback(self, rpc_id, reply,nb_reply):
        #Make length implicit to array to be compatible with older Pythonic approach
        #Ignore rpc_id as this is for non-blocking C
        self.active_callback(arr.array('B',reply[:nb_reply]))
        return True

    def _conversion_nonblocking_callback(self, rpc_id, reply,nb_reply):
        #Make length implicit to array to be compatible with older Pythonic approach

        if rpc_id in self.py_callbacks:
            func=self.py_callbacks.pop(rpc_id)
            func(arr.array('B',reply[:nb_reply]))
        return True

    def load_rpc_result(self,rpc_id):
        return self.lib.load_rpc_result(ctypes.c_int(self.qid),ctypes.c_int(rpc_id))

    def is_rpc_result_ready(self,rpc_id):
        return self.lib.is_rpc_result_ready(ctypes.c_int(self.qid), ctypes.c_int(rpc_id))

    def get_rpc_duration_us(self,rpc_id):
        """
        Return duration of a completed rpc (from queue to acknowledge), in uS
        """
        return self.lib.get_rpc_duration_us(ctypes.c_int(self.qid), ctypes.c_int(rpc_id))


    def do_rpc(self, blocking, is_push, payload, nb_payload,rpc_callback ):
        array_type = ctypes.c_uint8 * nb_payload
        c_data = array_type(*payload[:nb_payload])
        if blocking:
            self.active_callback = rpc_callback
            rpc_id = self.lib.do_rpc(ctypes.c_bool(blocking),ctypes.c_bool(is_push),
                                      ctypes.c_uint8(self.qid),
                                      c_data, nb_payload, self.blocking_callback_func)
        else:
            rpc_id = self.lib.do_rpc(ctypes.c_bool(blocking), ctypes.c_bool(is_push),
                                     ctypes.c_uint8(self.qid),
                                     c_data, nb_payload, self.nonblocking_callback_func)
            self.py_callbacks[rpc_id]=rpc_callback

        return rpc_id



