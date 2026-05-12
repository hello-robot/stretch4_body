#!/usr/bin/env python3
from __future__ import print_function

import stretch4_body.core.hello_utils as hu
import stretch4_body.core.transport.transport as transport
from stretch4_body.core.stepper import Stepper
import time

s=[Stepper('/dev/hello-motor-omni-0'),Stepper('/dev/hello-motor-omni-1'),Stepper('/dev/hello-motor-omni-2')]
for ss in s:
    ss.startup()

dbg_last=None
ts=time.time()

# print("############################ Pulling status ############################ ")
for i in range(1000):
    rpc_ids=[]
    rpc_ids.append(s[0].pull_status(blocking=False))
    rpc_ids.append(s[1].pull_status(blocking=False))
    rpc_ids.append(s[2].pull_status(blocking=False))
    s[0].transport.load_rpc_results(rpc_ids[0],wait_on_result=False)
    s[1].transport.load_rpc_results(rpc_ids[1], wait_on_result=False)
    s[2].transport.load_rpc_results(rpc_ids[2], wait_on_result=False)
    # s[0].pretty_print()
    # s[1].pretty_print()
    # s[2].pretty_print()
    #print('ITR', i, 'RPC_ID', rpc_ids)#, 'duration', p.transport.get_rpc_duration_us(rpc_ids[0]))

print('DT',1000/(time.time()-ts))
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

for ss in s:
    ss.stop()
#print('duration',10000/dt)



