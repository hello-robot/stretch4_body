#!/usr/bin/env python3
import stretch4_body.core.hello_utils as hello_utils
import os
import queue
import signal
import logging

LOOP_STATE_INVALID="INVALID"
LOOP_STATE_RUNNING="RUNNING"
LOOP_STATE_PAUSED="PAUSED"

# ###########################################################################################

def worker_loop(loop_name,rate_hz,worker_instance,
                q_admin,q_status,q_cmd,do_exit,
                callback_step,
                callback_pause,
                callback_unpause,
                callback_exit):
    profile_enabled = os.environ.get('STRETCH_PROFILE') == '1'
    if profile_enabled:
        try:
            import yappi
            yappi.start()
        except ImportError:
            profile_enabled = False

    logger = worker_instance.logger if hasattr(worker_instance, 'logger') else logging.getLogger()
    logger.info('-------- Starting %s with PID %d --------'%(loop_name.capitalize(),os.getpid()))
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    loop_mgmt=hello_utils.LoopStats(loop_name,rate_hz)
    state = LOOP_STATE_RUNNING
    while not do_exit.is_set():
        status={}
        # 1. Handle server admin from queue
        try:
            message = q_admin.get_nowait()
            if message == "exit":
                logger.info('%s EXIT!'%loop_name.capitalize())
                state = LOOP_STATE_INVALID
                callback_exit(worker_instance)
                break
            if state is not LOOP_STATE_INVALID:
                if message == "pause":
                    logger.info('%s PAUSE!' % loop_name.capitalize())
                    if callback_pause(worker_instance):
                        state = LOOP_STATE_PAUSED
                if message == 'unpause' and state == LOOP_STATE_PAUSED:
                    logger.info('%s UNPAUSE!' % loop_name.capitalize())
                    if callback_unpause(worker_instance):
                        state = LOOP_STATE_RUNNING
        except queue.Empty:
            pass

        # 2. Wait until ctrl cycle ready
        import time
        while not do_exit.is_set():
            t_now = time.perf_counter()
            target_time = (1.0/loop_mgmt.target_loop_rate) * (loop_mgmt.loop_cycles + 1)
            time_to_wait = target_time - (t_now - loop_mgmt.ts_0)
            if time_to_wait > 0.1:
                do_exit.wait(0.05)
            else:
                break
                
        if do_exit.is_set():
            try:
                msg = q_admin.get_nowait()
                if msg == "exit":
                    logger.info('%s EXIT!'%loop_name.capitalize())
                    callback_exit(worker_instance)
            except queue.Empty:
                pass
            break
            
        loop_mgmt.wait_until_next_cycle(warn_delay=5.0,overrun_thresh_s=0.005,warn_on=False)
        loop_mgmt.mark_loop_start()

        #3. Step the Loop based on q_cmd_in, update status
        if state==LOOP_STATE_RUNNING:
            callback_step(worker_instance,q_cmd,status)

        status['loop']= {'name':loop_name,'stats':loop_mgmt.status, 'state': state}
        q_status.put(status) #These can overflow, make sure consumer keeps near zero
        loop_mgmt.mark_loop_end()
    logger.info('Exiting %s Loop'% loop_name.capitalize())
    
    # Critically prevent the child process from hanging on exit due to pending queue flushes
    q_admin.queue.cancel_join_thread()
    q_cmd.queue.cancel_join_thread()
    q_status.queue.cancel_join_thread()
    
    if profile_enabled:
        try:
            import yappi
            import sys
            import io
            print("\n" + "="*20 + f" PROFILING SUMMARY: {loop_name.upper()} " + "="*20, file=sys.stderr)
            yappi.stop()
            stats = yappi.get_func_stats()
            stats.sort('ttot', 'desc')
            s = io.StringIO()
            stats.print_all(out=s)
            lines = s.getvalue().splitlines()
            print('\n'.join(lines[:25]), file=sys.stderr)
            print("="*80 + "\n", file=sys.stderr)
        except Exception as e:
            print(f"Error printing profile summary for {loop_name}: {e}", file=sys.stderr)

    return True

# ###########################################################################################
#
# if __name__=='__main__':
#     r = RobotServer()
#     if r.start():
#         time.sleep(1.0)
#         r.q_cl_cmd.put(['power_periph|trigger_beep'])
#         loop_mgmt = hello_utils.LoopStats("robot_control_loop", r.params['client_loop_rate_Hz'])
#         try:
#             t_print_last=time.time()
#             exiting=False
#             while not exiting:
#                 loop_mgmt.mark_loop_start()
#                 r.handle_command_messages()
#                 r.publish_status()
#                 r.publish_status_aux()
#                 s=r.latest_status
#                 if time.time()-t_print_last>1.0:
#                     print('RobotServer : Runtime %.8f (s) | Rate %.2f (Hz): '%(s['control_loop']['stats']['execution_time_s'],s['control_loop']['stats']['curr_rate_hz']))
#                     t_print_last=time.time()
#                 exiting = r.handle_admin_messages()
#                 loop_mgmt.mark_loop_end()
#                 loop_mgmt.wait_until_next_cycle()
#         except (SystemExit,KeyboardInterrupt):
#             pass
#         r.stop()
