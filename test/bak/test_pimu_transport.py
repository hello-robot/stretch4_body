#!/usr/bin/env python3
from __future__ import print_function

import stretch4_body.core.hello_utils as hu
import stretch4_body.core.transport.transport as transport
from stretch4_body.pimu import Pimu
import time

p = Pimu()
p.startup()

dbg_last=None
ts=time.time()

# print("############################ Pulling status ############################ ")
for i in range(1000):
    rpc_ids=p.pull_status(blocking=False)
    p.transport.load_rpc_results(rpc_ids,wait_on_result=True)
    p.pretty_print()
    print('ITR', i, 'RPC_ID', rpc_ids, 'duration', p.transport.get_rpc_duration_us(rpc_ids[0]))


# print("############################ Testing Load Blocking  ############################ ")
# for i in range(100):
#     p.push_load_test(blocking=True)
#     p.pull_load_test(blocking=True)


# print("############################ Testing Load Non-Blocking  ############################ ")
# itr=0
# while True:
#     itr=itr+1
#     print('ITR',itr)
#     rpc_ids=p.push_load_test(blocking=False)
#     p.transport.load_rpc_results(rpc_ids, wait_on_result=True)
#     rpc_ids=p.pull_load_test(blocking=False)
#     p.transport.load_rpc_results(rpc_ids, wait_on_result=True)

p.stop()
#print('duration',10000/dt)



