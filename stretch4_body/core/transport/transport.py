import array as arr
from stretch4_body.core.transport.transport_util import *
from stretch4_body.core.transport.transport_pyserial import TransportPySerial
from stretch4_body.core.transport.transport_cserial import TransportCSerial
import stretch4_body.core.hello_utils as hello_utils
import logging
import threading
import time

# ##################### TRANSPORT ####################################


class Transport(TransportPySerial):
    """
    Older systems use TransportPySerial. This class wraps TransportPySerial to be
    the default backend (and backwards compatible to older systems). 
    It also exposes alternate, faster communication backends


    """
    BACKEND_PY_SERIAL = 0
    BACKEND_C_SERIAL = 1

    def __init__(self, port_name, logger=logging.getLogger(), default_backend=BACKEND_PY_SERIAL, qid=None):
        """
        Parameters
        ----------
        port_name: device name, eg /dev/ttyACM0
        logger: system logger
        chid: int id of serial channel in C library to use

        Returns
        -------
        None
        """
        self.default_backend=default_backend
        self.use_c_backend = (default_backend==self.BACKEND_C_SERIAL)
        self.qid = qid
        self.rpc_ids=[]
        self.n_rate_log=0
        self.rate_log=[]

        TransportPySerial.__init__(self, port_name, logger=logger)
        if self.use_c_backend:
            self.transport_c = TransportCSerial(port_name, qid, logger=logger)
            self.status['rpc_duration_us']=0
        else:
            self.transport_c = None

        self.device_name=self.port_name[self.port_name.rfind('/')+1:]

    def startup(self):
        if not hello_utils.acquire_transport_filelock(self.device_name):
            self.logger.error(f"""Unable for {self.device_name} to acquire transport_filelock.
Server or other process may already be running.
Try running stretch_body_server --kill""")
            return False

        success = TransportPySerial.startup(self)
        if self.use_c_backend:
            x=self.transport_c.startup()
            success = success and x
            if success:
                y=self.transport_c.start_nonblocking_thread()
                success = success and y
        return success

    def stop(self):
        hello_utils.free_transport_filelock(self.device_name)
        TransportPySerial.stop(self)
        if self.use_c_backend:
            self.transport_c.stop_nonblocking_thread()
            self.transport_c.stop()

    def configure_version(self, firmware_version):
        TransportPySerial.configure_version(self, firmware_version)
        if self.configure_version == RPC_TRANSPORT_VERSION_0 and self.transport_c:
            self.logger.error(
                'Transport: C backend not supported for  RPC_TRANSPORT_VERSION_0. Defaulting to Py backend')
            self.transport_c.stop()
            self.transport_c = None
            self.use_c_backend = False

    def pause(self):
        TransportPySerial.pause(self)

    def unpause(self):
        TransportPySerial.unpause(self)

    ################## PULL/PUSH #######################

    def do_rpc(self, blocking, is_push, payload, rpc_callback, backend=None, exiting=False):
        """
        Return rpc_id if successful, None if fails
        """
        if backend is None:
            backend = self.default_backend
        if backend == self.BACKEND_C_SERIAL and not self.use_c_backend:
            self.logger.error('Transport: do_rpc. Backend not supported: {0} '.format(backend))
            return None
        if blocking is False and not self.use_c_backend:
            self.logger.error('Transport: do_rpc. nonblocking mode requires C backend: {0}'.format(self.port_name))
            return None
        if backend == self.BACKEND_PY_SERIAL:
            TransportPySerial.do_pull_rpc_sync(self, payload, rpc_callback, exiting=exiting)
            return None
        if backend == self.BACKEND_C_SERIAL:
            #Always use the non-blocking function, just wait for it to finish if blocking
            rpc_id = self.transport_c.do_rpc(False, is_push, payload, nb_payload=len(payload), rpc_callback=rpc_callback)
            if blocking:
                time.sleep(.001) #Min 1khz wait time for an RPC to return
                res=self._load_rpc_result(rpc_id=rpc_id, wait_on_result=True)
                if res==0:
                    return None
            self.rpc_ids.append(rpc_id)
            return rpc_id

    def load_rpc_results(self, wait_on_result=True):
        res = []
        for r in self.rpc_ids:
            if r is not None:
                res.append(self._load_rpc_result(r, wait_on_result))  
        self.rpc_ids=[]
        return res

    def is_rpc_result_ready(self, rpc_id):
        if self.use_c_backend and rpc_id is not None:
            return self.transport_c.is_rpc_result_ready(rpc_id)
        return False

    def get_rpc_duration_us(self, rpc_id):
        if self.use_c_backend and rpc_id is not None:
            return self.transport_c.get_rpc_duration_us(rpc_id)
        return 0

    def _load_rpc_result(self, rpc_id, wait_on_result=True):
        """
        This allows the nonblocking RPC calls to load the results at a later time
        Pass in an rpc_id, call the associated callbacks to load data into PY
        Return 1 if loaded succesfully, 0 if invalid id or failed, or -1 if not yet ready / timeout
        """
        if not self.use_c_backend or rpc_id is None:
            return 0
        if self.transport_c is not None:
            res = self.transport_c.load_rpc_result(rpc_id)
            if (wait_on_result and res != 1):
                timeout = 1.0
                ts = time.time()
                while (True):
                    time.sleep(.001)
                    res = self.transport_c.load_rpc_result(rpc_id)
                    if res==1 or res==0:
                        break
                    if time.time() - ts > timeout:
                        res=-1
                        break
        #Handle rate logging
        if res==1:
            self.status['rpc_duration_us']=self.get_rpc_duration_us(rpc_id)
            if self.n_rate_log>0:
                self.rate_log.append(self.status['rpc_duration_us'])
                if len(self.rate_log)>self.n_rate_log: #Store last N rpc durations to get stats
                    self.rate_log.pop(0) 
        return res


    def do_pull_rpc_sync(self, payload, reply_callback, exiting=False):
        # Wrapper for backward compatibility
        return self.do_rpc(blocking=True, is_push=False, payload=payload, rpc_callback=reply_callback, exiting=exiting)

    ################## PUSH #######################

    def do_push_rpc_sync(self, payload, reply_callback, exiting=False):
        # Wrapper for backward compatibility
        return self.do_rpc(blocking=True, is_push=True, payload=payload, rpc_callback=reply_callback, exiting=exiting)





