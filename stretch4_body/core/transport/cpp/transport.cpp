#include "readerwriterqueue/readerwriterqueue.h"
#include "readerwriterqueue/readerwritercircularbuffer.h"
#include <iostream>
#include <thread>
#include <cstdint>
#include <random>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <vector>

#include "transport.h"
#include "transport_framing.h"
#include "crc16.h"

using namespace moodycamel;

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>


extern "C" {



void sigterm_handler(int signal, siginfo_t *info, void *_unused)
{
  fprintf(stderr, "Received SIGTERM from process with pid = %u\n",
      info->si_pid);
  exit(0);
}


#define QUEUED_RPC_BUFFER_SZ 255
class TransportQueue;
void queue_thread_loop(TransportQueue * tq);
bool thread_rpc_callback(int qid, uint8_t * rpc_reply, uint16_t nb_rpc_reply);

/*
TransportQueue enables fast lock-free non-blocking RPC comms with the uCs.
It uses lock free readerwriterqueues for fast unidirectional
passing of data from the Python process to the C++ thread

It works by
* Python side: Nonblocking User RPC --> byte payload --> transmit_q , where the RPC call is queued for consumption by the queue_thread.
 -- The python side RPC call back is stored for future use
 -- An rpc_id (handle) is generated and returned
* C++ side: queue_thread --> dequeue the RPC --> Push/Pull RPC with uC --> uC sends an ack  payload back --> store in ack_queue
* Python side: periodically move payloads from ack_queue --> ack_buf, where thread safe access of ack_buf can be had from Python
* Python side: use the rpc_id to lookup the ack payload and stuff it into the RPC callback
*/

class TransportQueue
{
public:
	TransportQueue(int idx):thread_active(false),exitSignal(false),rpc_id(0),cb_idx(0),ack_overwrites(0),qid(idx),ack_q(100)
	{
	}
	int qid;
    int rpc_id;
    int cb_idx;
    int ack_overwrites;

    RPCCallback  py_rpc_callback[QUEUED_RPC_BUFFER_SZ];
    QueuedRPC    ack_buf[QUEUED_RPC_BUFFER_SZ];
    QueuedRPC    send_rpc;
    std::string port_name;
    
    QueuedRPC thread_send_rpc;  //accessed by thread side


    //Using a circular buffer to avoid memory leaks by bounding queue size
    //However, this can cause producer thread to drop acks if consumer doesn't handle rpc_ids in time
    //This should never happen though,instead the ack_buf will overflow (as dequeue_available_ack_rpcs is called for every rpc sent down)
    BlockingReaderWriterCircularBuffer<QueuedRPC> ack_q;
    BlockingReaderWriterQueue<QueuedRPC> transmit_q;
    bool thread_active; 
    std::thread queue_thread;
    std::atomic<bool> exitSignal;
    TransportFraming tframer;



    ////////////// Queued RPC /////////////////////
    //Non-thread method: Queue new rpc for thread to act on
    int queue_rpc_to_transmit(bool is_push, uint8_t* payload,uint16_t nb_payload, RPCCallback cb )
    {
        if (!tframer.valid_port)
            return 0;

        dequeue_available_ack_rpcs();

        rpc_id=rpc_id+1; //will roll over eventually but that's ok
        memcpy(send_rpc.payload,payload,nb_payload);
        send_rpc.is_push=is_push;
        send_rpc.nb_payload=nb_payload;
        send_rpc.rpc_id=rpc_id;
        send_rpc.cb_idx=cb_idx;
        send_rpc.q_time=std::chrono::duration<double>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

        //reply will be posted at ack_buf[cb_idx], setup struct now
        //, flag as not dirty now so we can monitor when the reply comes in
        if (ack_buf[cb_idx].has_been_written && !ack_buf[cb_idx].has_been_read) //0 indicates an open slot
                ack_overwrites++;
        ack_buf[cb_idx].has_been_written=false;
        ack_buf[cb_idx].has_been_read=false;
        ack_buf[cb_idx].rpc_id=rpc_id;
        ack_buf[cb_idx].cb_idx=cb_idx;
        ack_buf[cb_idx].is_push=is_push;
        ack_buf[cb_idx].q_time= send_rpc.q_time;
        ack_buf[cb_idx].cb_time=0;
        //std::cout<<"Callback PY Q :"<<reinterpret_cast<void*>(cb)<<std::endl;
        py_rpc_callback[cb_idx]=cb; //store the Python call-back so can unpack later once ack is recieved
        transmit_q.enqueue(send_rpc);
        cb_idx=(cb_idx+1)%QUEUED_RPC_BUFFER_SZ;
        return rpc_id;
    }

    //Transfer any ack rpcs in the queue to the buffer for use on Python side
        bool dequeue_available_ack_rpcs()
        {
           QueuedRPC a;
           bool ret=false;
           while(ack_q.try_dequeue(a))
           {
                //sanity check, fail to handle acks on client side in timely fashion
                if (ack_buf[a.cb_idx].rpc_id!=a.rpc_id)
                  std::cout<<"Expired ack RPC in dequeue_available_ack_rpcs. Mismatched rpc_id: "<<ack_buf[a.cb_idx].rpc_id<<" | "<<a.rpc_id<<" | "<<port_name<<std::endl;
                else
                {
                    memcpy(ack_buf[a.cb_idx].payload,a.payload,a.nb_payload);
                    ack_buf[a.cb_idx].nb_payload=a.nb_payload;
                    ack_buf[a.cb_idx].cb_time=a.cb_time;
                    ack_buf[a.cb_idx].has_been_read=false;
                    ack_buf[a.cb_idx].has_been_written=true;
                    ret=true;
                }
           }
           return ret;
        }

    //Thread method: Dequeue an RPC and send to uC
    bool thread_execute_queued_rpc()
    {

        if (!tframer.valid_port)
            return false;

        //if (transmit_q.try_dequeue(thread_send_rpc))
        if (transmit_q.wait_dequeue_timed(thread_send_rpc, std::chrono::milliseconds(5)))
        {
            //std::cout<<"dequeued "<<thread_send_rpc.rpc_id<<std::endl;
            if (thread_send_rpc.is_push)
                tframer.doPushTransactionV1(qid,thread_send_rpc.payload, thread_send_rpc.nb_payload,(RPCCallback)thread_rpc_callback);
            else
                tframer.doPullTransactionV1(qid,thread_send_rpc.payload, thread_send_rpc.nb_payload,(RPCCallback)thread_rpc_callback);
            return true;
        }
        return false;
    }

    int get_idx_of_rpc_id(int rpc_id)
    {
        for (int idx=0;idx<QUEUED_RPC_BUFFER_SZ;idx++)
        {
            //std::cout<<"searching "<<idx<<" "<<ack_buf[idx].rpc_id<<" "<<rpc_id<<std::endl;
            if (ack_buf[idx].rpc_id==rpc_id)
                return idx;
                }
        return -1;
    }



    //Python method: Get the result from the rpc and load into the Python callback
    //Return 0: invalid or expired rpc_id
    //Return -1: valid rpc_id but not ready
    //Return 1: successfully loaded
    int load_rpc_result(int rpc_id)
    {
        dequeue_available_ack_rpcs();

    //TODO: Lock as Thread will access ack_buf via load_rpc_result
        int result=0;
        int idx=get_idx_of_rpc_id(rpc_id);
        if (idx==-1)
        {
            std::cout<<"Idx not found! "<<rpc_id<<std::endl;
            return 0;
            }
        if ( ack_buf[idx].has_been_written)
        {
            (*py_rpc_callback[idx])(ack_buf[idx].rpc_id, (uint8_t *)ack_buf[idx].payload, (uint16_t)ack_buf[idx].nb_payload);
            ack_buf[idx].has_been_read=true;//expired
            return 1;
        }
        //std::cout<<"result not ready : rpc_id: "<<rpc_id<<std::endl;
        return -1;
    }

    bool is_rpc_result_ready(int rpc_id)
    {
        dequeue_available_ack_rpcs();
        int idx=get_idx_of_rpc_id(rpc_id);
        if (idx==-1)
            return 0;
        return (ack_buf[idx].has_been_written);
    }

    double get_rpc_duration_us(int rpc_id)
    {
        dequeue_available_ack_rpcs();
        int idx=get_idx_of_rpc_id(rpc_id);
        if (idx==-1)
            return 0;
        return  (ack_buf[idx].cb_time-ack_buf[idx].q_time)*1000000;
    }

    ////////////// THREAD ADMIN /////////////////////
    bool start_thread()
    {
        if (!thread_active)
        {
            exitSignal=false;
            queue_thread = std::thread(queue_thread_loop,this);
            thread_active=true;
            return true;
        }
        return false;
    }

    bool shutdown_thread()
    {
        exitSignal=true;
        if (thread_active)
        {
            if (queue_thread.joinable())
            {
                std::cout<<"Shutting down transmit_q thread "<<port_name<<std::endl;
                queue_thread.join();
            }
            thread_active=false;
            return true;
        }
        return false;
    }
};
//# ##############################################################################################################3
#define N_QUEUES 8

struct TQArray {
    TransportQueue* arr[N_QUEUES];
    TQArray() { for(int i=0; i<N_QUEUES; i++) arr[i] = new TransportQueue(i); }
    TransportQueue& operator[](int i) { return *arr[i]; }
};
TQArray TQ;

//Thread method: Q the uC reply according so can be later dequed and found by rpc_id
//This is called with result of the RPC comms with uC
bool thread_rpc_callback(int qid, uint8_t * rpc_reply, uint16_t nb_rpc_reply)
{
    QueuedRPC ack_rpc;
    int cb_idx=TQ[qid].thread_send_rpc.cb_idx;
    int rpc_id=TQ[qid].thread_send_rpc.rpc_id;
    memcpy(ack_rpc.payload,rpc_reply,nb_rpc_reply);
    ack_rpc.nb_payload=nb_rpc_reply;
    ack_rpc.rpc_id=rpc_id;
    ack_rpc.cb_idx=cb_idx;
    ack_rpc.cb_time=std::chrono::duration<double>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    //TQ[qid].ack_q.enqueue(ack_rpc);
    if (!TQ[qid].ack_q.wait_enqueue_timed(ack_rpc,std::chrono::milliseconds(1)))
    {
        std::cout<<"Transport_cpp: ack_q overrun. Consumer not calling load_rpc_result quickly enough. Dropping ack for rpc_id: "<<rpc_id<<std::endl;
    }

    return true;
 }

void queue_thread_loop(TransportQueue * tq)
{
    std::cout << "Starting transmit_q thread: "<<tq->port_name<<std::endl;
    while (!tq->exitSignal)
    {
        tq->thread_execute_queued_rpc();
        //std::this_thread::sleep_for(std::chrono::microseconds(100));//avoid starving the cpu
    }
}


////////////////////// Called from Python ///////////////////////////////

double get_rpc_duration_us(int qid,int rpc_id)
{
    return TQ[qid].get_rpc_duration_us(rpc_id);
}
int load_rpc_result(int qid, int rpc_id)
{
    return TQ[qid].load_rpc_result(rpc_id);
}
bool is_rpc_result_ready(int qid, int rpc_id)
{
    return TQ[qid].is_rpc_result_ready(rpc_id);
}

bool start_serial(int qid, const char * port_name)
{
  struct sigaction action;
    action.sa_handler = NULL;
    action.sa_sigaction = sigterm_handler;
    sigemptyset(&action.sa_mask);
    action.sa_flags = SA_SIGINFO;
    action.sa_restorer = NULL;

  sigaction(SIGTERM, &action, NULL);

    TQ[qid].port_name=std::string(port_name);
    return TQ[qid].tframer.startup(port_name);
}


bool start_queue_thread(int qid)
{

    return TQ[qid].start_thread();
}

bool shutdown_queue_thread(int qid)
{
    return TQ[qid].shutdown_thread();
}

int do_rpc(bool blocking, bool is_push, int qid, uint8_t* payload, uint16_t nb_payload, RPCCallback rpc_callback)
{
    if (!TQ[qid].tframer.valid_port)
        return 0;

    if(blocking)
    {
        if (is_push)
            return (int) TQ[qid].tframer.doPushTransactionV1(qid, payload, nb_payload,rpc_callback);
        else
            return (int) TQ[qid].tframer.doPullTransactionV1(qid, payload, nb_payload,rpc_callback);
    }
    else
    {
        return TQ[qid].queue_rpc_to_transmit(is_push,payload,nb_payload,rpc_callback);
    }
    return 0;
}

} // END extern "C"


