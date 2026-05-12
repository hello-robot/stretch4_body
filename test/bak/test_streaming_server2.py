#!/usr/bin/env python3

from multiprocessing import  Process, Queue, current_process
import time
import queue # imported for using queue.Empty exception

from stretch4_body.core.device import Device
import stretch4_body.omnibase as base
import stretch4_body.arm as arm
import stretch4_body.lift as lift
import stretch4_body.pimu as pimu
import stretch4_body.core.hello_utils as hello_utils

from serial import SerialException



class Robot(Device):
    """
    API to the Stretch Robot
    """
    def __init__(self):
        Device.__init__(self, 'robot')
        self.status = {'pimu': {}}
        self.pimu=pimu.Pimu()
        self.status['pimu']=self.pimu.status
        self.devices={ 'pimu':self.pimu}
        self.streaming_server=None

    # ###########  Device Methods #############
    def startup(self):
        success=self.pimu.startup()
        return success

    def start_streaming_server(self):
        if self.streaming_server is not None:
            self.streaming_server=RobotStreamingServer(self)

    def stop_streaming_server(self):
        if self.streaming_server is not None:
            self.streaming_server.stop()
            self.streaming_server=None

    def stop(self):
        """
        To be called once before exiting a program
        Cleanly stops down motion and communication
        """
        self.streaming_server.stop()
        self.logger.info('---- Shutting down robot ----')
        self.pimu.stop()
        self.logger.info('---- Shutdown complete ----')

class RobotStreamingServer:
    def __init__(self,robot):
        self.q_cmd = Queue()
        self.q_status = Queue()
        self.robot=robot
        self.wbl_process = Process(target=self.do_whole_body_loop, args=(self.q_cmd, self.q_status))
        self.wbl_process.start()

    def do_whole_body_loop(self,q_cmd, q_status):
        stop=False
        while not stop:

            #Trigger new status reads from uC
            rpc_ids = self.robot.pimu.pull_status(blocking=False)

            # Ingest new commands waiting to be executed
            cmd=None
            try:
                cmd = q_cmd.get_nowait()
                if cmd=='exit':
                    stop=True
                print('New cmd: ',cmd)
            except queue.Empty:
                pass

            self.robot.pimu.stream_execute_commands(cmd['pimu'])

            #Push the commands down to uC
            self.robot.pimu.push_command()

            #Load the status read results
            self.robot.pimu.transport.load_rpc_results(rpc_ids, wait_on_result=True)
            self.robot.pimu.pretty_print()

            self.q_status.put({'pimu':self.robot.pimu.status})

        return True

    def pull_status(self):
        s=None
        while self.q_status.qsize():
            s=self.q_status.get(block=True,timeout=.1)
        return s

    def push_command(self,cmd):
        self.q_cmd.put(cmd)
    # def pause(self):
    #     self.q_cmd.put('pause')
    def stop(self):
        self.q_cmd.put('exit')
        self.wbl_process.join()


if __name__ == '__main__':
    r=RobotStreamingServer()
    r.start()
    r.test()
    r.stop()