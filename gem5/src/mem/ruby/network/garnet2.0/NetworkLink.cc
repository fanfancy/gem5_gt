/*
 * Copyright (c) 2008 Princeton University
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
 * Authors: Niket Agarwal
 *          Tushar Krishna
 */
using namespace std;
#include <cmath>
#include <iomanip>
#include <set>
#include <string>
#include <vector>

#include <fstream>
#include <cassert>
#include <sstream> 

#include "mem/ruby/network/garnet2.0/NetworkLink.hh"

#include "mem/ruby/network/garnet2.0/CreditLink.hh"

NetworkLink::NetworkLink(const Params *p)
    : ClockedObject(p), Consumer(this), m_id(p->link_id),
      m_type(NUM_LINK_TYPES_),
      m_latency(p->link_latency),
      linkBuffer(new flitBuffer()), link_consumer(nullptr),
      link_srcQueue(nullptr), m_link_utilized(0),
      m_vc_load(p->vcs_per_vnet * p->virt_nets)
{
}

NetworkLink::~NetworkLink()
{
    delete linkBuffer;
}

void
NetworkLink::setLinkConsumer(Consumer *consumer)
{
    link_consumer = consumer;
}

void
NetworkLink::setSourceQueue(flitBuffer *srcQueue)
{
    link_srcQueue = srcQueue;
}


void outputMessage(std::string line, std::string filename)
{
    std::string file;
    file = "./../output_info/" + filename;
    fstream f;
    f.open(file,ios::out|ios::app);
    f<<line<<std::endl; 
    f.close(); 
}
//wxy add in 4.6
void
NetworkLink::updateStats()
{
    time_cur = curCycle();
    //Cycles time_delta = time_cur - time_pre;
    int link_util_total = 0;
    int link_util_all = 0;

    int activity = getLinkUtilization();
    int activity_delta = activity - activity_pre;

    link_util_all += activity;
    link_util_total += activity_delta;

    //double link_util_average = ((double)activity_delta / time_delta);
    activity_pre = activity;
    time_pre = time_cur;
    std::string message_to_write;
    message_to_write.append("link id : ");
    message_to_write.append(std::to_string(m_id));
    message_to_write.append("  ; curTime : ");
    message_to_write.append(std::to_string(time_cur));
    message_to_write.append("  ; link_util_all : ");
    message_to_write.append(std::to_string(link_util_total));
    message_to_write.append("  ; link_util_all_time : ");
    message_to_write.append(std::to_string(link_util_all));
    outputMessage(message_to_write, "link_utilization.txt");
}

void
NetworkLink::wakeup()
{
    if (link_srcQueue->isReady(curCycle())) {
        flit *t_flit = link_srcQueue->getTopFlit();
        t_flit->set_time(curCycle() + m_latency);
        linkBuffer->insert(t_flit);
        link_consumer->scheduleEventAbsolute(clockEdge(m_latency));
        m_link_utilized++;
        m_vc_load[t_flit->get_vc()]++;
    }
    // wxy change in 4.6
    if(curCycle() % 10000 == 0)
        updateStats();
}

void
NetworkLink::resetStats()
{
    for (int i = 0; i < m_vc_load.size(); i++) {
        m_vc_load[i] = 0;
    }

    m_link_utilized = 0;
}

NetworkLink *
NetworkLinkParams::create()
{
    return new NetworkLink(this);
}

CreditLink *
CreditLinkParams::create()
{
    return new CreditLink(this);
}

uint32_t
NetworkLink::functionalWrite(Packet *pkt)
{
    return linkBuffer->functionalWrite(pkt);
}
