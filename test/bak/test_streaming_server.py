#!/usr/bin/env python3
import zmq
import time
import sys
from multiprocessing import Process

class RobotStreamingServer:
    def __init__(self,port_no="1234"):
        self.port_no=port_no

    def start(self):
        Process(target=self.server, args=()).start()
        Process(target=self.client, args=()).start()

    def server(self):
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.bind("tcp://*:%s" % self.port_no)
        print("Running server on port: ", self.port_no)
        # serves only 5 request and dies
        for reqnum in range(5):
            # Wait for next request from client
            message = socket.recv()
            print("Received request #%s: %s" % (reqnum, message))
            socket.send_string("World from %s" % self.port_no)


    def client(self):
        context = zmq.Context()
        print("Connecting to server at port %s" % self.port_no)
        socket = context.socket(zmq.REQ)
        socket.connect("tcp://localhost:%s" % self.port_no)
        for request in range(20):
            print("Sending request ", request, "...")
            socket.send_string("Hello")
            message = socket.recv()
            print("Received reply ", request, "[", message, "]")
            time.sleep(1)



if __name__ == "__main__":
    # Now we can run a few servers
    r=RobotStreamingServer()
    r.start()
    time.sleep(5.0)
    print('DONE!')

#
# # https://medium.com/@jshlbrd/building-distributed-scalable-python-apps-with-pyzmq-and-multiprocessing-ae832f75d1f0
#
# from argparse import ArgumentParser
# from multiprocessing import Process
#
# from random import randint
# import json
# import os
# import signal
# import zmq
#
# class Foo():
#     def __init__(self):
#         print('Foo')
#     def mmb_init_model(self):
#         pass
#     def handle_message(self,message):
#         print("Got message",message)
#         data = {
#             "name": "Alice",
#             "age": 30,
#             "isStudent": False,
#             "courses": ["Math", "Science"]
#         }
#         return data
#     def mmb_prediction_to_json(self,result):
#         return result
#
# class MmbotWorker(Process):
#     def __init__(self, server, backend_port):
#         super(MmbotWorker, self).__init__()
#         self.mmb = Foo()
#         self.identity = "%04X-%04X" % (randint(0, 0x10000), randint(0, 0x10000))
#         self.server = server
#         self.backend_port = backend_port
#
#     def run(self):
#         print("worker %s loading model" % self.identity)
#         self.mmb.mmb_init_model()
#         print("worker %s model loaded" % self.identity)
#         context = zmq.Context()
#         worker = context.socket(zmq.REP)
#         worker.connect("tcp://{}:{}".format(self.server, self.backend_port))
#         while 1:
#             message = worker.recv()
#             result = self.mmb.handle_message(message)
#             worker.send(json.dumps(self.mmb.mmb_prediction_to_json(result)))
#             print("worker %s sending results" % self.identity)
#             print(result)
#
# class MmbotProxy(Process):
#     def __init__(self, frontend_port, backend_port):
#         super(MmbotProxy, self).__init__()
#         self.frontend_port = frontend_port
#         self.backend_port = backend_port
#     def run(self):
#         context = zmq.Context()
#         # Socket facing clients
#         frontend = context.socket(zmq.XREP)
#         frontend.bind("tcp://*:{}".format(self.frontend_port))
#         # Socket facing services
#         backend = context.socket(zmq.XREQ)
#         backend.bind("tcp://*:{}".format(self.backend_port))
#         print("proxy starting up")
#         zmq.proxy(frontend, backend)
#
#
# processes = []
# def signal_handler(_signum, _frame):
#     for process in processes:
#         if process.is_alive():
#             os.kill(process.pid, signal.SIGKILL)
#     for process in processes:
#         process.join()
#
# def main():
#     signal.signal(signal.SIGTERM, signal_handler)
#     signal.signal(signal.SIGINT, signal_handler)
#
#     parser = ArgumentParser(description='utilizes ZMQ to run mmbot as a service.')
#     parser.add_argument('procs', type=int,help='the number of mmbot workers that will be launched. each worker will load the model, which uses approx. 1GB of memory per worker.')
#     parser.add_argument('proxy',help='the proxy server address.')
#     parser.add_argument('frontend_port', help='the frontend proxy port for clients to connect to.')
#     parser.add_argument('backend_port',help='the backend proxy port for workers to connect to.')
#
#     args = parser.parse_args()
#     proxy_proc = MmbotProxy(args.frontend_port, args.backend_port)
#     proxy_proc.start()
#     processes.append(proxy_proc)
#     for _ in range(args.procs):
#         worker_proc = MmbotWorker(args.proxy, args.backend_port)
#         worker_proc.start()
#         processes.append(worker_proc)
# if __name__ == "__main__":
#     main()
