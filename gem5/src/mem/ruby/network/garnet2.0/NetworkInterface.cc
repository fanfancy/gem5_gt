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


#include "mem/ruby/network/garnet2.0/NetworkInterface.hh"

#include <cassert>
#include <cmath>

#include<fstream>
#include<iostream>

#include "base/cast.hh"
#include "base/stl_helpers.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/network/garnet2.0/Credit.hh"
#include "mem/ruby/network/garnet2.0/flitBuffer.hh"
#include "mem/ruby/slicc_interface/Message.hh"

using namespace std;
using m5::stl_helpers::deletePointers;

# define NI_send (std::string("./../Node_NI/ni2node/"))
# define NI_recv (std::string("./../Node_NI/node2ni/"))
# define NI_output_file (std::string("./../output_info/cpu_output_resnet_col/"))
# define node_num       (int(81))

NetworkInterface::NetworkInterface(const Params *p)
    : ClockedObject(p), Consumer(this), m_id(p->id),
      m_virtual_networks(p->virt_nets), m_vc_per_vnet(p->vcs_per_vnet),
      m_num_vcs(m_vc_per_vnet * m_virtual_networks),
      m_deadlock_threshold(p->garnet_deadlock_threshold),
      vc_busy_counter(m_virtual_networks, 0)
{
    num_recv_packet = 0;
    m_router_id = -1;
    m_vc_round_robin = 0;
    m_ni_out_vcs.resize(m_num_vcs);
    m_ni_out_vcs_enqueue_time.resize(m_num_vcs);
    outCreditQueue = new flitBuffer();

    m_data_num = 0;
    // instantiating the NI flit buffers
    for (int i = 0; i < m_num_vcs; i++) {
        m_ni_out_vcs[i] = new flitBuffer();
        m_ni_out_vcs_enqueue_time[i] = Cycles(INFINITE_);
    }

    m_vc_allocator.resize(m_virtual_networks); // 1 allocator per vnet
    for (int i = 0; i < m_virtual_networks; i++) {
        m_vc_allocator[i] = 0;
    }

    m_stall_count.resize(m_virtual_networks);
}

void init_recv_packet_file(int id){
    std::string file1;
    std::string file2;
    file1 = NI_send+std::to_string(id)+"_col_id.txt";
	ofstream OutFile(file1); 
    OutFile.close(); 
    file2 = NI_send+std::to_string(id)+".txt";
	ofstream OutFile2(file2); 
    OutFile2 << std::to_string(0);
    OutFile2.close();
}

void init_recv_flit_info(int id)
{
	std::string file;
    file = "./../output_info/flit_out/"+std::to_string(id)+".txt";
	ofstream OutFile(file); 
    OutFile.close();       
}

void
NetworkInterface::init()
{
    for (int i = 0; i < m_num_vcs; i++) {
        m_out_vc_state.push_back(new OutVcState(i, m_net_ptr));
    }
    //if(m_id > node_num)
        //init_recv_flit_info(m_id-node_num);
    m_data_num = 0;
    data_last = 0;
    if(m_id < node_num){
        init_recv_packet_file(m_id);
    }
}

NetworkInterface::~NetworkInterface()
{
    deletePointers(m_out_vc_state);
    deletePointers(m_ni_out_vcs);
    delete outCreditQueue;
    delete outFlitQueue;
}

void
NetworkInterface::addInPort(NetworkLink *in_link,
                              CreditLink *credit_link)
{
    inNetLink = in_link;
    in_link->setLinkConsumer(this);
    outCreditLink = credit_link;
    credit_link->setSourceQueue(outCreditQueue);
}

void
NetworkInterface::addOutPort(NetworkLink *out_link,
                             CreditLink *credit_link,
                             SwitchID router_id)
{
    inCreditLink = credit_link;
    credit_link->setLinkConsumer(this);

    outNetLink = out_link;
    outFlitQueue = new flitBuffer();
    out_link->setSourceQueue(outFlitQueue);

    m_router_id = router_id;
}

void
NetworkInterface::addNode(vector<MessageBuffer *>& in,
                            vector<MessageBuffer *>& out)
{
    inNode_ptr = in;
    outNode_ptr = out;

    for (auto& it : in) {
        if (it != nullptr) {
            it->setConsumer(this);
        }
    }
}

void
NetworkInterface::dequeueCallback()
{
    // An output MessageBuffer has dequeued something this cycle and there
    // is now space to enqueue a stalled message. However, we cannot wake
    // on the same cycle as the dequeue. Schedule a wake at the soonest
    // possible time (next cycle).
    scheduleEventAbsolute(clockEdge(Cycles(1)));
}

void
NetworkInterface::incrementStats(flit *t_flit)
{
    int vnet = t_flit->get_vnet();

    // Latency
    m_net_ptr->increment_received_flits(vnet);
    Cycles network_delay =
        t_flit->get_dequeue_time() - t_flit->get_enqueue_time() - Cycles(1);
    Cycles src_queueing_delay = t_flit->get_src_delay();
    Cycles dest_queueing_delay = (curCycle() - t_flit->get_dequeue_time());
    Cycles queueing_delay = src_queueing_delay + dest_queueing_delay;

    // Decrement the additional cycle counted at src NIC
    // when flit is enqueued.
    network_delay = network_delay - Cycles(1);

    // Decrement default 2 cycles of queueing delay in every message
    // from protocol
    queueing_delay = queueing_delay - Cycles(2);

    m_net_ptr->increment_flit_network_latency(network_delay, vnet);
    m_net_ptr->increment_flit_queueing_latency(queueing_delay, vnet);

    if (t_flit->get_type() == TAIL_ || t_flit->get_type() == HEAD_TAIL_) {
        m_net_ptr->increment_received_packets(vnet);
        m_net_ptr->increment_packet_network_latency(network_delay, vnet);
        m_net_ptr->increment_packet_queueing_latency(queueing_delay, vnet);
    }

    // Hops
    m_net_ptr->increment_total_hops(t_flit->get_route().hops_traversed);
}

/*
 * The NI wakeup checks whether there are any ready messages in the protocol
 * buffer. If yes, it picks that up, flitisizes it into a number of flits and
 * puts it into an output buffer and schedules the output link. On a wakeup
 * it also checks whether there are flits in the input link. If yes, it picks
 * them up and if the flit is a tail, the NI inserts the corresponding message
 * into the protocol buffer. It also checks for credits being sent by the
 * downstream router.
 */

// 更新接收到的包的数目
void update_recv_packets(int id,int num_recv_packet)
{
	std::string file;
	file = NI_send+std::to_string(id)+".txt";
	ofstream OutFile(file);
	OutFile << std::to_string(num_recv_packet); 
    //std::cout<<"fanxi added, update_recv_packets ing, id= " << id <<" packets="<<num_recv_packet<<std::endl;
	OutFile.close();        
}
//
void update_recv_info(int id, int src, int time, int last_flag){
    std::string file;
    file = NI_output_file + std::to_string(id) + ".txt";
    fstream f;
    std::string message_to_write;

    message_to_write.append("Receive_Packet_form_node ");
    message_to_write.append(std::to_string(src));
    message_to_write.append(" at_time ");
    message_to_write.append(std::to_string(time));
    message_to_write.append(" is_last? ");
    message_to_write.append(std::to_string(last_flag));
    //追加写入,在原来基础上加了ios::app 
    f.open(file,ios::out|ios::app);
    f<<message_to_write<<endl; 
    f.close(); 
    //std::cout<<"fanxi added, tell_mem_send_data ing, id= " << id << std::endl;
}
// 更新接收到的包的列号信息
void update_recv_col_id(int id,int col_id)
{
	std::string file;
	file = NI_send+std::to_string(id)+"_col_id.txt";
	ofstream OutFile(file);
	OutFile << std::to_string(col_id); 
    //std::cout<<"fanxi added, update_recv_packets ing, id= " << id <<" packets="<<num_recv_packet<<std::endl;
	OutFile.close();        
}


vector<string> split_1(const string &str, const string &pattern)
{
    vector<string> res;
    if(str == "")
        return res;
    //在字符串末尾也加入分隔符，方便截取最后一段
    string strs = str + pattern;
    size_t pos = strs.find(pattern);

    while(pos != strs.npos)
    {
        string temp = strs.substr(0, pos);
        res.push_back(temp);
        //去掉已分割的字符串,在剩下的字符串中进行分割
        strs = strs.substr(pos+1, strs.size());
        pos = strs.find(pattern);
    }

    return res;
}


//wxy add in 4.8
// 接受有关于send_last的信息
bool NetworkInterface::send_last_update(int id){
    std::string file;
	file = NI_recv+std::to_string(id)+".txt";
	ifstream infile; 
    infile.open(file.data());  
    assert(infile.is_open());   
    std::string data_line;
    std::vector<string> data_list;
    int read_line_num=0;
    while(getline(infile, data_line))
    {
        if (read_line_num == m_data_num){
            break;
        }
        read_line_num += 1;
    }
    data_list = split_1(data_line," ");
    data_last = atoi(data_list[0].c_str());
    m_data = atoi(data_list[1].c_str());
    //std::cout<<"wxy add in NI id"<<id<<"  send_data "<<data_list[1]<<" "<<m_data<<std::endl;
    infile.close();             //关闭文件输入流 
    if (read_line_num < m_data_num || data_line == ""){ //文件最后会有个空行
        return 0;
    }
    else {
        //std::cout<<"fanxi added, get_task, id= " << id <<" linenum=" <<line_num << " task ="<< current_task_line <<std::endl;
        m_data_num += 1;
        return 1;   
    }
}

//wxy add in 3.31
//总结每个尾部flit的有关latency的信息，输出到flit_out文件夹
void
NetworkInterface:: recv_flit_info(flit *t_flit, int id)
{
	std::string file;
	file = "./../output_info/flit_out/"+std::to_string(id)+".txt";
	ofstream OutFile;
    OutFile.open(file,ios::app);
    Cycles network_delay =
        t_flit->get_dequeue_time() - t_flit->get_enqueue_time() - Cycles(1);
    Cycles src_queueing_delay = t_flit->get_src_delay();
    Cycles dest_queueing_delay = curCycle() - t_flit->get_dequeue_time();
    Cycles queueing_delay = src_queueing_delay + dest_queueing_delay;
    OutFile<<std::endl;
    OutFile<< "CurTick="<<curTick()<<" : [ network_delay="<<network_delay<<" "<<"src_queueing_delay="<<src_queueing_delay<<" dest_queueing_delay="<<dest_queueing_delay<<" queueing_delay="<<queueing_delay<<" ]"<<std::endl;
    t_flit->print(OutFile);
	OutFile.close();        
}

void
NetworkInterface::wakeup()
{
    DPRINTF(RubyNetwork, "Network Interface %d connected to router %d "
            "woke up at time: %lld\n", m_id, m_router_id, curCycle());

    MsgPtr msg_ptr;
    Tick curTime = clockEdge();

    // Checking for messages coming from the protocol
    // can pick up a message/cycle for each virtual net
    for (int vnet = 0; vnet < inNode_ptr.size(); ++vnet) {
        MessageBuffer *b = inNode_ptr[vnet];
        if (b == nullptr) {
            continue;
        }

        if (b->isReady(curTime)) { // Is there a message waiting
            msg_ptr = b->peekMsgPtr();
            if (flitisizeMessage(msg_ptr, vnet)) {
                b->dequeue(curTime);
            }
        }
    }

    scheduleOutputLink();
    checkReschedule();

    // Check if there are flits stalling a virtual channel. Track if a
    // message is enqueued to restrict ejection to one message per cycle.
    bool messageEnqueuedThisCycle = checkStallQueue();

    /*********** Check the incoming flit link **********/
    if (inNetLink->isReady(curCycle())) {
        flit *t_flit = inNetLink->consumeLink();
        int vnet = t_flit->get_vnet();
        t_flit->set_dequeue_time(curCycle());

        // If a tail flit is received, enqueue into the protocol buffers if
        // space is available. Otherwise, exchange non-tail flits for credits.
        if (t_flit->get_type() == TAIL_ || t_flit->get_type() == HEAD_TAIL_) {
            if (!messageEnqueuedThisCycle &&
                outNode_ptr[vnet]->areNSlotsAvailable(1, curTime)) {
                // Space is available. Enqueue to protocol buffer.
                outNode_ptr[vnet]->enqueue(t_flit->get_msg_ptr(), curTime,
                                           cyclesToTicks(Cycles(1)));

                // Simply send a credit back since we are not buffering
                // this flit in the NI
                sendCredit(t_flit, true);

                // Update stats and delete flit pointer
                // std::cout << "fanxi added in NI.cc, now incrementStats(t_flit) ing" << std::endl;
                incrementStats(t_flit);
                //recv_flit_info(t_flit, m_id-node_num);
                //std::cout <<  "NI"  << m_id << "num_recv_packet = " << num_recv_packet << std::endl;
                // # received packets ++ here

                //wxy add in 4.8
                bool last_flag = t_flit->check_last();
                int col_id = t_flit->get_col_id();
                if(last_flag){
                    num_recv_packet ++;
                    update_recv_packets(m_id-node_num, num_recv_packet);
                    update_recv_col_id(m_id-node_num, col_id);
                }
                update_recv_info(m_id-node_num,t_flit->get_src(),curTick(),last_flag);

                delete t_flit;
            } else {
                // No space available- Place tail flit in stall queue and set
                // up a callback for when protocol buffer is dequeued. Stat
                // update and flit pointer deletion will occur upon unstall.
                m_stall_queue.push_back(t_flit);
                m_stall_count[vnet]++;

                auto cb = std::bind(&NetworkInterface::dequeueCallback, this);
                outNode_ptr[vnet]->registerDequeueCallback(cb);
            }
        } else {
            // Non-tail flit. Send back a credit but not VC free signal.
            sendCredit(t_flit, false);

            // Update stats and delete flit pointer.
            incrementStats(t_flit);
            delete t_flit;
        }
    }

    /****************** Check the incoming credit link *******/

    if (inCreditLink->isReady(curCycle())) {
        Credit *t_credit = (Credit*) inCreditLink->consumeLink();
        m_out_vc_state[t_credit->get_vc()]->increment_credit();
        if (t_credit->is_free_signal()) {
            m_out_vc_state[t_credit->get_vc()]->setState(IDLE_, curCycle());
        }
        delete t_credit;
    }


    // It is possible to enqueue multiple outgoing credit flits if a message
    // was unstalled in the same cycle as a new message arrives. In this
    // case, we should schedule another wakeup to ensure the credit is sent
    // back.
    if (outCreditQueue->getSize() > 0) {
        outCreditLink->scheduleEventAbsolute(clockEdge(Cycles(1)));
    }
}

void
NetworkInterface::sendCredit(flit *t_flit, bool is_free)
{
    Credit *credit_flit = new Credit(t_flit->get_vc(), is_free, curCycle());
    outCreditQueue->insert(credit_flit);
}

bool
NetworkInterface::checkStallQueue()
{
    bool messageEnqueuedThisCycle = false;
    Tick curTime = clockEdge();

    if (!m_stall_queue.empty()) {
        for (auto stallIter = m_stall_queue.begin();
             stallIter != m_stall_queue.end(); ) {
            flit *stallFlit = *stallIter;
            int vnet = stallFlit->get_vnet();

            // If we can now eject to the protocol buffer, send back credits
            if (outNode_ptr[vnet]->areNSlotsAvailable(1, curTime)) {
                outNode_ptr[vnet]->enqueue(stallFlit->get_msg_ptr(), curTime,
                                           cyclesToTicks(Cycles(1)));

                // Send back a credit with free signal now that the VC is no
                // longer stalled.
                sendCredit(stallFlit, true);

                // Update Stats
                incrementStats(stallFlit);

                // Flit can now safely be deleted and removed from stall queue
                delete stallFlit;
                m_stall_queue.erase(stallIter);
                m_stall_count[vnet]--;

                // If there are no more stalled messages for this vnet, the
                // callback on it's MessageBuffer is not needed.
                if (m_stall_count[vnet] == 0)
                    outNode_ptr[vnet]->unregisterDequeueCallback();

                messageEnqueuedThisCycle = true;
                break;
            } else {
                ++stallIter;
            }
        }
    }

    return messageEnqueuedThisCycle;
}

//wxy add in 4.1
//输出NI得到的数值进行比较判断传输无误，输出到send_data文件夹
void record_recv_data(int id, int data)
{
	std::string file;
	file = "./../output_info/Node_NI/"+std::to_string(id)+".txt";
	ofstream OutFile;
    OutFile.open(file,ios::app); 
    OutFile<<std::to_string(data)<<"\t"<<curTick()<<::endl;
	OutFile.close();        
}

//wxy add in 4.1
//实现NI与PE间的数据数值的传递
int NetworkInterface::get_send_data(int id)
{
	std::string file;
	file = "./../output_info/send_data/"+std::to_string(id)+".txt";
	ifstream infile; 
    infile.open(file.data());  
    assert(infile.is_open());   
    std::string data_line;
    int read_line_num=0;
    while(getline(infile, data_line))
    {
        if (read_line_num == m_data_num){
            break;
        }
        read_line_num += 1;
    }
    m_data = atoi(data_line.c_str());
    infile.close();             //关闭文件输入流 
    if (read_line_num < m_data_num || data_line == ""){ //文件最后会有个空行
        return 0;
    }
    else {
        //std::cout<<"fanxi added, get_task, id= " << id <<" linenum=" <<line_num << " task ="<< current_task_line <<std::endl;
        m_data_num += 1;
        return 1;   
    }
    
}

// Embed the protocol message into flits
bool
NetworkInterface::flitisizeMessage(MsgPtr msg_ptr, int vnet)
{
    Message *net_msg_ptr = msg_ptr.get();
    NetDest net_msg_dest = net_msg_ptr->getDestination();

    // gets all the destinations associated with this message.
    vector<NodeID> dest_nodes = net_msg_dest.getAllDest();

    // Number of flits is dependent on the link bandwidth available.
    // This is expressed in terms of bytes/cycle or the flit size
    int num_flits = (int) ceil((double) m_net_ptr->MessageSizeType_to_int(
        net_msg_ptr->getMessageSize())/m_net_ptr->getNiFlitSize());
    
    //std::cout<<"wxy add in flitisizeMessage ： getMessageSize = "<<(double) m_net_ptr->MessageSizeType_to_int(
        //net_msg_ptr->getMessageSize())<<"   ;NiFlitSize = "<<m_net_ptr->getNiFlitSize()<<"  ;flit per massage = "<<std::to_string(num_flits)<<std::endl;

    // loop to convert all multicast messages into unicast messages
    for (int ctr = 0; ctr < dest_nodes.size(); ctr++) {
        //std::cout<<"wxy add in flitisizeMessage ： dest_node_size" << dest_nodes.size()<<std::endl;
        // this will return a free output virtual channel
        int vc = calculateVC(vnet);

        if (vc == -1) {
            return false ;
        }

        MsgPtr new_msg_ptr = msg_ptr->clone();
        NodeID destID = dest_nodes[ctr];

        Message *new_net_msg_ptr = new_msg_ptr.get();
        if (dest_nodes.size() > 1) {
            NetDest personal_dest;
            for (int m = 0; m < (int) MachineType_NUM; m++) {
                if ((destID >= MachineType_base_number((MachineType) m)) &&
                    destID < MachineType_base_number((MachineType) (m+1))) {
                    // calculating the NetDest associated with this destID
                    personal_dest.clear();
                    personal_dest.add((MachineID) {(MachineType) m, (destID -
                        MachineType_base_number((MachineType) m))});
                    new_net_msg_ptr->getDestination() = personal_dest;
                    break;
                }
            }
            net_msg_dest.removeNetDest(personal_dest);
            // removing the destination from the original message to reflect
            // that a message with this particular destination has been
            // flitisized and an output vc is acquired
            net_msg_ptr->getDestination().removeNetDest(personal_dest);
        }

        // Embed Route into the flits
        // NetDest format is used by the routing table
        // Custom routing algorithms just need destID
        //if(get_send_data(m_id) == 0)
            //std::cout<<"wxy add in NI.cc data transfer fail"<<std::endl;
        //record_recv_data(m_id, m_data);

        //wxy add in 4.8
        send_last_update(m_id);
        record_recv_data(m_id,m_data);

        RouteInfo route;
        route.vnet = vnet;
        route.net_dest = new_net_msg_ptr->getDestination();
        route.src_ni = m_id;
        route.src_router = m_router_id;
        route.dest_ni = destID;
        route.dest_router = m_net_ptr->get_router_id(destID);

        // initialize hops_traversed to -1
        // so that the first router increments it to 0
        route.hops_traversed = -1;

        m_net_ptr->increment_injected_packets(vnet);
        for (int i = 0; i < num_flits; i++) {
            m_net_ptr->increment_injected_flits(vnet);
            flit *fl = new flit(i, vc, vnet, route, num_flits, new_msg_ptr,
                curCycle(),m_data,data_last);

            fl->set_src_delay(curCycle() - ticksToCycles(msg_ptr->getTime()));
            m_ni_out_vcs[vc]->insert(fl);
        }

        m_ni_out_vcs_enqueue_time[vc] = curCycle();
        m_out_vc_state[vc]->setState(ACTIVE_, curCycle());
    }
    return true ;
}

//wxy add in 3.20
//输出VC分配的相关输出，目前已关闭
void calVC_output(int id, int vnet, int m_vc_per_vnet, int delta, int flag)
{
	ofstream OutFile;
    std::string file;
	file = "./../output_info/VCallocator/"+std::to_string(id)+".txt";
    std::string line;
    if(flag == 1){
        line = "VC Allocate Success , allocated in vnet:vc_id = " + std::to_string(vnet) + ":" + std::to_string(delta) ;
    }
    else{
        line = "VC allocate Fail , vnet:vc_busy_counter = " + std::to_string(vnet) + ":" + std::to_string(delta);
    }
    OutFile.open(file, ios::app);
	OutFile <<line<<std::endl; 
    //std::cout<<"wxy added, calVC_output ing, id= " << id <<" m_vc_per_vnet="<<m_vc_per_vnet<<std::endl;
    //std::cout<<"wxy added, calVC_output ing    "<<line<<std::endl;
	OutFile.close();        
}

// Looking for a free output vc
int
NetworkInterface::calculateVC(int vnet)
{
    for (int i = 0; i < m_vc_per_vnet; i++) {
        int delta = m_vc_allocator[vnet];
        m_vc_allocator[vnet]++;
        if (m_vc_allocator[vnet] == m_vc_per_vnet)
            m_vc_allocator[vnet] = 0;

        if (m_out_vc_state[(vnet*m_vc_per_vnet) + delta]->isInState(
                    IDLE_, curCycle())) {
            vc_busy_counter[vnet] = 0;
            //calVC_output(m_id, vnet, m_vc_per_vnet, delta, 1);
            return ((vnet*m_vc_per_vnet) + delta);
        }
    }

    vc_busy_counter[vnet] += 1;
    //calVC_output(m_id, vnet, m_vc_per_vnet, vc_busy_counter[vnet], 0);
    panic_if(vc_busy_counter[vnet] > 500000,
        "%s: Possible network deadlock in vnet: %d at time: %llu \n",
        name(), vnet, curTick());
    //panic_if(vc_busy_counter[vnet] > m_deadlock_threshold,
    //    "%s: Possible network deadlock in vnet: %d at time: %llu \n",
    //    name(), vnet, curTick());

    return -1;
}


/** This function looks at the NI buffers
 *  if some buffer has flits which are ready to traverse the link in the next
 *  cycle, and the downstream output vc associated with this flit has buffers
 *  left, the link is scheduled for the next cycle
 */

void
NetworkInterface::scheduleOutputLink()
{
    int vc = m_vc_round_robin;
    m_vc_round_robin++;
    if (m_vc_round_robin == m_num_vcs)
        m_vc_round_robin = 0;

    for (int i = 0; i < m_num_vcs; i++) {
        vc++;
        if (vc == m_num_vcs)
            vc = 0;

        // model buffer backpressure
        if (m_ni_out_vcs[vc]->isReady(curCycle()) &&
            m_out_vc_state[vc]->has_credit()) {

            bool is_candidate_vc = true;
            int t_vnet = get_vnet(vc);
            int vc_base = t_vnet * m_vc_per_vnet;

            if (m_net_ptr->isVNetOrdered(t_vnet)) {
                for (int vc_offset = 0; vc_offset < m_vc_per_vnet;
                     vc_offset++) {
                    int t_vc = vc_base + vc_offset;
                    if (m_ni_out_vcs[t_vc]->isReady(curCycle())) {
                        if (m_ni_out_vcs_enqueue_time[t_vc] <
                            m_ni_out_vcs_enqueue_time[vc]) {
                            is_candidate_vc = false;
                            break;
                        }
                    }
                }
            }
            if (!is_candidate_vc)
                continue;

            m_out_vc_state[vc]->decrement_credit();
            // Just removing the flit
            flit *t_flit = m_ni_out_vcs[vc]->getTopFlit();
            t_flit->set_time(curCycle() + Cycles(1));
            outFlitQueue->insert(t_flit);
            // schedule the out link
            outNetLink->scheduleEventAbsolute(clockEdge(Cycles(1)));

            if (t_flit->get_type() == TAIL_ ||
               t_flit->get_type() == HEAD_TAIL_) {
                m_ni_out_vcs_enqueue_time[vc] = Cycles(INFINITE_);
            }
            return;
        }
    }
}

int
NetworkInterface::get_vnet(int vc)
{
    for (int i = 0; i < m_virtual_networks; i++) {
        if (vc >= (i*m_vc_per_vnet) && vc < ((i+1)*m_vc_per_vnet)) {
            return i;
        }
    }
    fatal("Could not determine vc");
}


// Wakeup the NI in the next cycle if there are waiting
// messages in the protocol buffer, or waiting flits in the
// output VC buffer
void
NetworkInterface::checkReschedule()
{
    for (const auto& it : inNode_ptr) {
        if (it == nullptr) {
            continue;
        }

        while (it->isReady(clockEdge())) { // Is there a message waiting
            scheduleEvent(Cycles(1));
            return;
        }
    }

    for (int vc = 0; vc < m_num_vcs; vc++) {
        if (m_ni_out_vcs[vc]->isReady(curCycle() + Cycles(1))) {
            scheduleEvent(Cycles(1));
            return;
        }
    }
}

void
NetworkInterface::print(std::ostream& out) const
{
    out << "[Network Interface]";
}

uint32_t
NetworkInterface::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;
    for (unsigned int i  = 0; i < m_num_vcs; ++i) {
        num_functional_writes += m_ni_out_vcs[i]->functionalWrite(pkt);
    }

    num_functional_writes += outFlitQueue->functionalWrite(pkt);
    return num_functional_writes;
}

NetworkInterface *
GarnetNetworkInterfaceParams::create()
{
    return new NetworkInterface(this);
}
