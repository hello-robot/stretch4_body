#pragma once

#include <array>
#include <cstdint>
#include <chrono>

#define MAX_BYTES_PAYLOAD 2000
typedef bool (* RPCCallback) (int, uint8_t *, uint16_t);
/*
Layers on an RPC tunnel onto the queue mechanism
1. An RPC data is sent down in push/pull
2. This data is queued and an RPC_ID is returned
3. This ID can be polled on when the RPC is complete

*/
struct QueuedRPC {
    uint8_t payload[MAX_BYTES_PAYLOAD];
    uint16_t nb_payload;
    int rpc_id;
    int cb_idx;
    bool is_push;
    bool has_been_written;
    bool has_been_read;
    double q_time;
    double cb_time;
    QueuedRPC() = default;
};


#ifdef __cplusplus
extern "C" {
#endif

bool start_queue_thread(int qid);
bool shutdown_queue_thread(int qid);
bool start_serial(int qid, const char * port_name);
int do_rpc(bool blocking, bool is_push, int qid, uint8_t* payload, uint16_t nb_payload, RPCCallback rpc_callback);
int load_rpc_result(int qid, int rpc_id);
bool is_rpc_result_ready(int qid, int rpc_id);
double get_rpc_duration_us(int qid,int rpc_id);
#ifdef __cplusplus
}
#endif
