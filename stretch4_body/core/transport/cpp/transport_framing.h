#pragma once

#include <array>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <iostream>

#include "cobs.h"
#include "crc16.h"
#include "SCSerial.h"
#include "transport.h"

#define RPC_V1_MAX_FRAMES 18  // Required to support 1024 bytes
#define RPC_V1_PUSH_FRAME_FIRST_MORE 201
#define RPC_V1_PUSH_FRAME_FIRST_ONLY 202
#define RPC_V1_PUSH_FRAME_MORE 203
#define RPC_V1_PUSH_FRAME_LAST  204
#define RPC_V1_PUSH_ACK  205
#define RPC_V1_PULL_FRAME_FIRST 206
#define RPC_V1_PULL_FRAME_MORE 207
#define RPC_V1_PULL_FRAME_ACK_MORE  208
#define RPC_V1_PULL_FRAME_ACK_LAST 209
#define RPC_V1_FRAME_DATA_MAX_BYTES  58 //63 - 2 (CRC) - 1 (Cobbs Header) - 1 (FRAME CMD) - 1 (Packet Marker)

#define COBBS_FRAME_SIZE_V1 63 //Was seeing issues when transmitting 64 bytes so limiting to 63. Issue resolved.

//////////////////////////////  Shared Defines ///////////////////////////////////////////////////
#define RPC_DATA_MAX_BYTES  1024
#define RPC_MAX_FRAME_SIZE 64 //Arduino and Linux USB Uart has a 64 byte buffer. When frame is >64 have seen issues.


//RPC_DATA_MAX_BYTES_SAMD51 = 8194 #SAMD51 needs 8KB of space for 2048 floats + rpc ID
#define COBBS_PACKET_MARKER 0

class TransportFraming : public SCSerial{
   public:
      bool startup(const char* serialPort);
      bool doPushTransactionV1(int qid, uint8_t * rpc_out, uint16_t nb_rpc_out,RPCCallback rpc_callback);
      bool doPullTransactionV1(int qid, uint8_t * rpc_out, uint16_t nb_rpc_out,RPCCallback rpc_callback);
      inline TransportFraming()
      {
         transactions=0;
         valid_port=false;
         crc_ok=false;
      }
      bool valid_port;
   private:
      bool sendFramedData(uint8_t * frame, uint16_t nb_frame);
      bool receiveFramedData(uint8_t * frame_buf, uint16_t & nb_frame);
      bool handlePushAckV1(bool crc, uint16_t nr, int ack_code);
      bool handlePullAckV1(bool crc, uint16_t nr, int ack_code);

      int transactions;

      bool crc_ok;
      Crc16 crc;
      COBS cobs;


    };


