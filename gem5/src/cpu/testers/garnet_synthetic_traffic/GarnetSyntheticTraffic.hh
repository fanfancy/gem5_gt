/*
 * Copyright (c) 2016 Georgia Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Tushar Krishna
 */

#ifndef __CPU_GARNET_SYNTHETIC_TRAFFIC_HH__
#define __CPU_GARNET_SYNTHETIC_TRAFFIC_HH__

#include <set>


#include "base/statistics.hh"
#include "mem/mem_object.hh"
#include "mem/port.hh"
#include "params/GarnetSyntheticTraffic.hh"
#include "sim/eventq.hh"
#include "sim/sim_exit.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"

//#include <vector>
using namespace std;

enum TrafficType {BIT_COMPLEMENT_ = 0,
                  BIT_REVERSE_ = 1,
                  BIT_ROTATION_ = 2,
                  NEIGHBOR_ = 3,
                  SHUFFLE_ = 4,
                  TORNADO_ = 5,
                  TRANSPOSE_ = 6,
                  UNIFORM_RANDOM_ = 7,
                  NUM_TRAFFIC_PATTERNS_};

class Packet;
class GarnetSyntheticTraffic : public MemObject
{
  public:
    int cal_cycles;
    int packets_to_send ;
    int dst_num;
    int finish_flag;
    std::vector<int> send_merge;
    std::vector<int> send_dst_list;
    int send_dst;
    int packets_sent;
    int cpu_status;
    int num_packet_wait;
    int cycles_caled;
    int total_packet_recv_previous;
    int cpu_work_stats;
    std::string current_task_line;
    int current_line_num;
    int get_task(int id,int line_num);
    int tick_pre = 0;
    int communication_tick = 0;

    typedef GarnetSyntheticTrafficParams Params;
    GarnetSyntheticTraffic(const Params *p);

    virtual void init();

    // main simulation loop (one cycle)
    void tick();
    void tick_pre_0();
    void tick_pre_1();
    virtual BaseMasterPort &getMasterPort(const std::string &if_name,
                                          PortID idx = InvalidPortID);

    /**
     * Print state of address in memory system via PrintReq (for
     * debugging).
     */
    void printAddr(Addr a);

  protected:
    EventFunctionWrapper tickEvent;

    class CpuPort : public MasterPort
    {
        GarnetSyntheticTraffic *tester;

      public:

        int num_packet_recv;

        CpuPort(const std::string &_name, GarnetSyntheticTraffic *_tester)
            : MasterPort(_name, _tester), tester(_tester)
        { }

      protected:

        virtual bool recvTimingResp(PacketPtr pkt);

        virtual void recvReqRetry();
    };

    CpuPort cachePort;

    class GarnetSyntheticTrafficSenderState : public Packet::SenderState
    {
      public:
        /** Constructor. */
        GarnetSyntheticTrafficSenderState(uint8_t *_data)
            : data(_data)
        { }

        // Hold onto data pointer
        uint8_t *data;
    };

    PacketPtr retryPkt;
    unsigned size;
    int id;


    std::map<std::string, TrafficType> trafficStringToEnum;

    unsigned blockSizeBits;

    Tick noResponseCycles;

    int numDestinations;
    Tick simCycles;
    int numPacketsMax;
    int numPacketsSent;
    int singleSender;
    int singleDest;

    std::string trafficType; // string
    TrafficType traffic; // enum from string
    double injRate;
    int injVnet;
    int precision;

    const Cycles responseLimit;

    MasterID masterId;

    void completeRequest(PacketPtr pkt);

    void generatePkt(int send_dst);
    void sendPkt(PacketPtr pkt);
    void initTrafficType();

    void doRetry();

    friend class MemCompleteEvent;
};

#endif // __CPU_GARNET_SYNTHETIC_TRAFFIC_HH__
