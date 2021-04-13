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

#include "cpu/testers/garnet_synthetic_traffic/GarnetSyntheticTraffic.hh"

#include <cmath>
#include <iomanip>
#include <set>
#include <string>
#include <vector>

#include <iostream>
#include <fstream>
#include <cassert>
#include <sstream>  


#include "base/logging.hh"
#include "base/random.hh"
#include "base/statistics.hh"
#include "debug/GarnetSyntheticTraffic.hh"
#include "mem/mem_object.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "sim/sim_events.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

// CPU satus
# define IDLE               (int(0))
# define WORKIING           (int(1)) 
# define FINISH             (int(2)) 
// CPU working status
# define WORK_WAIT          (int(3))
# define WORK_CAL           (int(4))
# define WORK_SEND          (int(5))
# define WORK_IDLE          (int(6))

// data num per packet
# define DATA_PER_PAC       (int(4))

# define Traffic_output_file    (std::string("./../output_info/cpu_output_resnet_col/"))
# define Task_folder        (std::string("./../task/task_resnet_col/"))
# define Traffic_node_num   (int(81))
# define FINISH_PIC         (int(20))
# define Node_recv (std::string("./../Node_NI/ni2node/"))
# define Node_send (std::string("./../Node_NI/node2ni/"))
# define setup_wait_num (int(1000))

using namespace std;

int TESTER_NETWORK=0;
//vector<int> w(100,1);
//vector<vector<int>> send_dst_list(100,w);
// DATA_PER_PAC send commands merge as one
//vector<int> a(100,1);
//vector<vector<int>> send_merge(100,a);

bool
GarnetSyntheticTraffic::CpuPort::recvTimingResp(PacketPtr pkt)
{
    tester->completeRequest(pkt);  // packet is injected fanxi
    return true;
}

void
GarnetSyntheticTraffic::CpuPort::recvReqRetry()
{
    tester->doRetry();
}

void
GarnetSyntheticTraffic::sendPkt(PacketPtr pkt)
{
    if (!cachePort.sendTimingReq(pkt)) {
        retryPkt = pkt; // RubyPort will retry sending
    }
    numPacketsSent++;
}

GarnetSyntheticTraffic::GarnetSyntheticTraffic(const Params *p)
    : MemObject(p),
      tickEvent([this]{ tick(); }, "GarnetSyntheticTraffic tick",
                false, Event::CPU_Tick_Pri),
      cachePort("GarnetSyntheticTraffic", this),
      retryPkt(NULL),
      size(p->memory_size),
      blockSizeBits(p->block_offset),
      numDestinations(p->num_dest),
      simCycles(p->sim_cycles),
      numPacketsMax(p->num_packets_max),
      numPacketsSent(0),
      singleSender(p->single_sender),
      singleDest(p->single_dest),
      trafficType(p->traffic_type),
      injRate(p->inj_rate),
      injVnet(p->inj_vnet),
      precision(p->precision),
      responseLimit(p->response_limit),
      masterId(p->system->getMasterId(name()))
{
    // set up counters
    noResponseCycles = 0;
    schedule(tickEvent, 0);

    initTrafficType();
    if (trafficStringToEnum.count(trafficType) == 0) {
        fatal("Unknown Traffic Type: %s!\n", traffic);
    }
    traffic = trafficStringToEnum[trafficType];

    id = TESTER_NETWORK++;
    DPRINTF(GarnetSyntheticTraffic,"Config Created: Name = %s , and id = %d\n",
            name(), id);

    // std::cout << "fanxi added when new GarnetSyntheticTraffic, name()= "<< name() <<" id = " << id << std::endl;
    // fanxi added when new GarnetSyntheticTraffic, name()= system.cpu06 id = 6
}

BaseMasterPort &
GarnetSyntheticTraffic::getMasterPort(const std::string &if_name, PortID idx)
{
    // called by 16 times if_name="test"
    //std::cout << "fanxi added when getMasterPort, if_name= " << if_name <<" idx= " <<idx <<std::endl;
    if (if_name == "test")
        return cachePort;
    else
        return MemObject::getMasterPort(if_name, idx);
}

vector<string> split(const string &str, const string &pattern)
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

void init_Node_NI_files(int id){
    std::string file;
    file = Node_send+std::to_string(id)+".txt";
	ofstream OutFile(file);
    OutFile.close();    
}

void init_cpu_output_file(int id){
    std::string file;
    std::string file1;
    file = Traffic_output_file+std::to_string(id)+".txt";
	ofstream OutFile(file);
    OutFile.close(); 
}
//wxy add in 4.6
// 设置当前节点维护的列的所需的列序列与输出的节点信息
void GarnetSyntheticTraffic::init_hold_col(){
    std::string file;
	file = Task_folder+std::to_string(id)+"_hold_col.txt";
	ifstream infile; 
    infile.open(file.data());  
    assert(infile.is_open());   

    int col_num = 0;
    std::string hold_col_line;
    int need_to_send = 0;
    while(getline(infile, hold_col_line))
    {
        int id;
        std::vector <int> i_col_id;
        std::vector <int> o_node_id;
        if(hold_col_line != ""){
            int flag = 0;
            col_num += 1;
            vector<string> hold_col_split;
            hold_col_split = split(hold_col_line, " "); // hold_col_id : i_col_id ; o_node
            id = atoi(hold_col_split[0].c_str());
            for(int i = 0 ; i < hold_col_split.size()-2 ; i++){
                if(hold_col_split[i+2] == ";")
                    flag = 1;
                else{
                    if(!flag)
                        i_col_id.push_back(atoi(hold_col_split[i+2].c_str()));
                    else{
                        o_node_id.push_back(atoi(hold_col_split[i+2].c_str()));
                    }
                }
            }
            if (o_node_id.size() > 0)
                need_to_send = 1;
            Col col_cur = {id, i_col_id, o_node_id, 0};
            hold_col_list.push_back(col_cur);
        }
    }
    send_flag = need_to_send;
    infile.close();             //关闭文件输入流 
}

// 设置每列的计算时间与发送包的数目
void GarnetSyntheticTraffic::init_col(){
    std::string file;
	file = Task_folder+std::to_string(id)+"_task.txt";
	ifstream infile; 
    infile.open(file.data());  
    assert(infile.is_open()); 

    std::string task_line;
    int cal_flag = 0;
    int send_flag = 0;
    col_compute_cycle = 0;
    col_packet_num = 0;
    while(getline(infile, task_line))
    {
        vector<string> task_line_split;
        task_line_split = split(task_line, " ");
        if(task_line_split[0] == "cal" && cal_flag == 0){
            col_compute_cycle = atoi(task_line_split[2].c_str());
            cal_flag = 1;
        }
        else if(task_line_split[0] == "send" && send_flag == 0){
            col_packet_num = atoi(task_line_split[2].c_str());
            send_flag = 1;
        }
        if(cal_flag && send_flag)
            break;
    }
}

void GarnetSyntheticTraffic::init_wait_packet(){
    std::string file;
    std::string line;
	file = Task_folder+std::to_string(id)+"_task.txt";
	ifstream infile; 
    infile.open(file.data());  
    assert(infile.is_open());   

    
    getline(infile,line);

    if (line == ""){ //文件最后会有个空行
        cpu_status = FINISH;
        cpu_work_stats = WORK_IDLE;
        std::cout<<"wxy add empty in id"<<id<<std::endl;
    }
    else {
        std::vector<std::string> line_list = split(line, " ");
        if(line_list[0] == "wait"){
            wait_num = line_list.size() - 1;
        }
        else{
            wait_num = 0;
        }
        cpu_status = IDLE;
        cpu_work_stats = WORK_IDLE;
        std::cout<<"wxy add not empty in id"<<id<<std::endl;
    }

    infile.close();
}

//wxy add in 4.6
void
GarnetSyntheticTraffic::init_computation(){
    recv_col_id.clear();
    init_hold_col();
    init_col();
    init_wait_packet();
}

void GarnetSyntheticTraffic::display(){
    std::string file;
    file = "./../display/"+std::to_string(id)+".txt";
    fstream f;
    f.open(file,ios::out|ios::app);
    std::string message_to_write;
    
    int col_num = hold_col_list.size();
    int col_id;
    std::vector <int> i_col_id;
    std::vector <int> o_node_id;
    for(int i = 0 ; i < col_num ; i++){
        i_col_id.clear();
        o_node_id.clear();
        col_id = hold_col_list[i].id;
        i_col_id = hold_col_list[i].i_col;
        o_node_id = hold_col_list[i].send_node;
        message_to_write.append(std::to_string(col_id));
        message_to_write.append(" len=");
        message_to_write.append(std::to_string(i_col_id.size()));
        message_to_write.append(" len=");
        message_to_write.append(std::to_string(o_node_id.size()));
        message_to_write.append(" : i_col_list :");
        for(int j = 0; j < i_col_id.size(); j++){
            message_to_write.append(" ");
            message_to_write.append(std::to_string(i_col_id[j]));
        }
        message_to_write.append("; o_node_id :");
        for(int j = 0; j < o_node_id.size(); j++){
            message_to_write.append(" ");
            message_to_write.append(std::to_string(o_node_id[j]));
        }
        //追加写入,在原来基础上加了ios::app 
        f<<message_to_write<<std::endl; 
        message_to_write = "";
    }
    f.close(); 
}

void
GarnetSyntheticTraffic::init()
{   
    // std::cout << "fanxi added when GarnetSyntheticTraffic::init()"  <<std::endl;
    // called by 16 times
    cpu_status = IDLE;
    numPacketsSent = 0;
    current_line_num = 0;
    pic_num = 0;
    //init_send_command_output(id);
    //init_send_data(id);
    send_dst_list.resize(100,0);
    init_Node_NI_files(id);
    init_cpu_output_file(id);
    init_computation();
    display();
    //wxy add in 4.7
    compute_complete = 0;
    recv_col_complete = 0;
    total_col_recv_previous = 0;
    cycles_caled = 0;
    cur_col_id = 0;
    packets_sent = 0;
}

// 查找元素是否在vector中
bool is_element_in_vector(vector<int> v,int element){
	vector<int>::iterator it;
	it=find(v.begin(),v.end(),element);
	if (it!=v.end()){
		return true;
	}
	else{
		return false;
	}
}

void
GarnetSyntheticTraffic::completeRequest(PacketPtr pkt)
{
    //std::cout << "fanxi added when GarnetSyntheticTraffic::completeRequest()"  <<std::endl;

    DPRINTF(GarnetSyntheticTraffic,
            "Completed injection of %s packet for address %x\n",
            pkt->isWrite() ? "write" : "read\n",
            pkt->req->getPaddr());

    assert(pkt->isResponse());
    noResponseCycles = 0;
    delete pkt;
}

int recv_packets(int id)
{
	std::string file;
    file = Node_recv+std::to_string(id)+".txt";
	ifstream infile; 
    infile.open(file.data());  
    assert(infile.is_open());   

    string s;
    while(getline(infile,s))
    {
        //std::cout<<"fanxi added, recv_packets ing, id= " << id <<" packets="<<s<<std::endl;
    }
    infile.close();             //关闭文件输入流 
    return atoi(s.c_str());
}

// 1代表读到task 0 代表没有读到，文件结束
// 未用
int GarnetSyntheticTraffic::get_task(int id,int line_num)
{
	std::string file;
	file = "./../task/task_resnet_col/"+std::to_string(id)+".txt";
	ifstream infile; 
    infile.open(file.data());  
    assert(infile.is_open());   

    int read_line_num=0;
    while(getline(infile,current_task_line))
    {
        if (read_line_num == line_num){
            break;
        }
        read_line_num += 1;
    }
    infile.close();             //关闭文件输入流 
    if (read_line_num < line_num || current_task_line == ""){ //文件最后会有个空行
        return 0;
    }
    else {
        //std::cout<<"fanxi added, get_task, id= " << id <<" linenum=" <<line_num << " task ="<< current_task_line <<std::endl;
        return 1;   
    }
    
}

// 未用
void tell_mem_send_data(std::string src_mem_index,std::string num_wait_packets,int id) 
{
	std::string file;
    file = "./../cpu_task/"+src_mem_index+".txt";
    fstream f;

    std::string message_to_write = "send ";
    message_to_write.append(std::to_string(id));
    message_to_write.append(" ");
    message_to_write.append(num_wait_packets);
    //追加写入,在原来基础上加了ios::app 
	f.open(file,ios::out|ios::app);
    f<<message_to_write<<endl; 
    f.close(); 
    //std::cout<<"fanxi added, tell_mem_send_data ing, id= " << id << std::endl;
	 
}

//flag == 1 : record communicaion time ; info = communication time;
//== 0 : finish  ; info = current_line_num; 
// 未用
void output_data(int id, int info, bool flag)
{
    std::string file;
    file = "./../cpu_output/"+std::to_string(id)+".txt";
    fstream f;
    std::string message_to_write;
    if ( flag )
    {
        message_to_write.append("communication time = ");
        message_to_write.append(std::to_string(info));
        message_to_write.append(" ;  curTick = ");
        message_to_write.append(std::to_string(curTick()));
    }
    else
    {
        message_to_write.append("finish : curTick = ");
        message_to_write.append(std::to_string(curTick()));
        message_to_write.append("  ;  curTaskLineNum = ");
        message_to_write.append(std::to_string(info));
    }
    //追加写入,在原来基础上加了ios::app 
	f.open(file,ios::out|ios::app);
    f<<message_to_write<<endl; 
    f.close(); 
    //std::cout<<"fanxi added, tell_mem_send_data ing, id= " << id << std::endl;
}

//flag == 1 : record task read time ; info = task_line_num;
//== 0 : record send/finish successfully ; info = dst_node; 
//now use
void output_data_1(int id, int info, std::string type, bool flag, int pic_num)
{
    std::string file;
    //file = Traffic_output_file+std::to_string(id)+"_"+std::to_string(pic_num)+".txt";
    file = Traffic_output_file+std::to_string(id)+".txt";
    fstream f;
    std::string message_to_write;

    if(flag == 1){
        message_to_write.append(type);
        message_to_write.append("_at_time ");
        message_to_write.append(std::to_string(curTick()));
        message_to_write.append(" ;now_col_id ");
        message_to_write.append(std::to_string(info));
    }
    else{
        if(type == "send_last"){
            message_to_write.append("Send_last_successfully_to_node ");
            message_to_write.append(std::to_string(info));
        }
        else if(type == "send_notlast"){
            message_to_write.append("Send_not_last_successfully_to_node ");
            message_to_write.append(std::to_string(info));
        }
        else if(type == "finish"){
            message_to_write.append("Finish_successfully_at_picture ");
            message_to_write.append(std::to_string(pic_num));
        }
        message_to_write.append(" at_time ");
        message_to_write.append(std::to_string(curTick()));
    }
    //追加写入,在原来基础上加了ios::app 
    f.open(file,ios::out|ios::app);
    f<<message_to_write<<endl; 
    f.close(); 
    //std::cout<<"fanxi added, tell_mem_send_data ing, id= " << id << std::endl;
}

void
GarnetSyntheticTraffic::tick_pre_0()
{   
    std::cout << "cpu id"<<id<<" status:" << cpu_status <<" current_line_num:"<< current_line_num << std::endl;
    std::cout << "cpu id"<<id<<" cpu_work_stats:" << cpu_work_stats <<std::endl;
    
    int if_get_task;
    bool sendAllowedThisCycle = false;
    // idle status : read task file
    if (cpu_status == IDLE){
        if_get_task = get_task(id, current_line_num);
        
        
        if (if_get_task == 0){
            cpu_status = IDLE;
        }
        
        else{// 解析task_line
            current_line_num += 1;
            vector<string>  current_task;
            current_task  = split(current_task_line," ");

            if (current_task[0] == "wait"){
                std::cout << "== wait" << std::endl;
                std::cout << "== ID" << id <<"  wait curTick : " << curTick() <<std::endl;
                tick_pre = curTick();

                cpu_status = WORKIING;
                cpu_work_stats = WORK_WAIT;
                num_packet_wait = atoi(current_task[1].c_str());
                std::string str_num_wait_packets = current_task[1];
                std::string str_src_mem_index = current_task[2];
                tell_mem_send_data(str_src_mem_index,  str_num_wait_packets,  id);
            }
            else if (current_task[0] == "cal"){
                std::cout << "== cal" << std::endl;
                std::cout << "== cal curTick : " << curTick() <<std::endl;

                communication_tick = curTick() - tick_pre;
                std::cout << "== ID"<<id << "  comm_tick : "<< communication_tick << std::endl;
                
                output_data(id, communication_tick , 1);

                cpu_status = WORKIING;
                cpu_work_stats = WORK_CAL;

                stringstream stream;            //声明一个stringstream变量
                stream << current_task[1];      //向stream中插入字符串"1234"
                stream >> cal_cycles;           // 初始化cal_cycles 
                cycles_caled = 0;
            }
            else if (current_task[0] == "send"){
                std::cout << "== send" << std::endl;
                cpu_status = WORKIING;
                cpu_work_stats = WORK_SEND; 
                packets_to_send = atoi(current_task[2].c_str());
                send_dst = atoi(current_task[1].c_str());
                packets_sent = 0;
            }
            else if (strstr(current_task[0].c_str(), "finish") != NULL ) { // 最后一行current_task[0]会多一个终结符
                cpu_status = FINISH;
                cpu_work_stats = WORK_IDLE;

                output_data(id, current_line_num , 0);
            }
        }
    }

    // working status
    else if (cpu_status == WORKIING){
        if (cpu_work_stats == WORK_WAIT){
            int packet_recv = recv_packets(id) - total_packet_recv_previous;
            if (packet_recv == num_packet_wait){
                cpu_work_stats = WORK_IDLE;
                cpu_status = IDLE;
                total_packet_recv_previous += packet_recv;
            }
            // 否则维持wait状态
        }
        else if (cpu_work_stats == WORK_SEND){
            if (packets_sent == packets_to_send){  //TODO ++ packets_sent
                cpu_work_stats = WORK_IDLE;
                cpu_status = IDLE;
            }
            else {
                sendAllowedThisCycle = true;
            }
        }

        else if (cpu_work_stats == WORK_CAL){
            if (cycles_caled == cal_cycles){
                cpu_work_stats = WORK_IDLE;
                cpu_status = IDLE;
            }
            else {
                cycles_caled += 1;
            }
        }
    }
    

    
	
	// std::cout<<" fanxi added GarnetSyntheticTraffic::tick(), id= "<< id << std::endl;
    if (++noResponseCycles >= responseLimit) {
        fatal("%s deadlocked at cycle %d\n", name(), curTick());
    }

    // make new request based on injection rate
    // (injection rate's range depends on precision)
    // - generate a random number between 0 and 10^precision
    // - send pkt if this number is < injRate*(10^precision)
    
    // double injRange = pow((double) 10, (double) precision);
    // unsigned trySending = random_mt.random<unsigned>(0, (int) injRange);
    // if (trySending < injRate*injRange)
    //     sendAllowedThisCycle = true;
    // else
    //     sendAllowedThisCycle = false;

    // always generatePkt unless fixedPkts or singleSender is enabled
    if (sendAllowedThisCycle) {
		// std::cout<<" fanxi added GarnetSyntheticTraffic: sendAllowedThisCycle id = "<< id << std::endl; 
        bool senderEnable = true;

        if (numPacketsMax >= 0 && numPacketsSent >= numPacketsMax)
            senderEnable = false;

        if (singleSender >= 0 && id != singleSender)
            senderEnable = false;

        if (senderEnable){
            generatePkt(send_dst);
            packets_sent += 1;
        }
           
    }

    // Schedule wakeup
    if (curTick() >= simCycles)
    {
        if(cpu_status != FINISH) output_data(id, current_line_num, 0);
        exitSimLoop("Network Tester completed simCycles");
    }
    else {
        if (!tickEvent.scheduled())
            schedule(tickEvent, clockEdge(Cycles(1)));
    }
}

//wxy add in 3.30
//输出有关于合并send指令后的send指令执行情况，输出到send_command_info文件夹
// 未用
void send_command_output(int id, std::string line, int pic_num){
    ofstream OutFile;
    std::string file;
	file = "./../output_info/send_command_info/"+std::to_string(id)+"_"+std::to_string(pic_num)+".txt";
    OutFile.open(file, ios::app);
	OutFile <<line <<std::endl; 
	OutFile.close(); 
}

//wxy add in 4.1
//修改finish，使得他从头开始读任务
// not new ,older than tick()
void
GarnetSyntheticTraffic::tick_new()
{   
    int if_get_task;
    bool sendAllowedThisCycle = false;
    int flag = 0;
    std::string send_output_line;
    // idle status : read task file
    while(!flag){
        flag = 1;
        if (cpu_status == IDLE){
            if_get_task = get_task(id, current_line_num);
            
            
            if (if_get_task == 0){
                cpu_status = IDLE;
            }
            
            else{// 解析task_line
                current_line_num += 1;
                vector<string>  current_task;
                current_task  = split(current_task_line," ");
                output_data_1(id, current_line_num-1, current_task[0], 1, pic_num);

                if (current_task[0] == "wait"){
                    //std::cout << "== wait" << std::endl;
                    //std::cout << "== ID" << id <<"  wait curTick : " << curTick() <<std::endl;
                    tick_pre = curTick();

                    cpu_status = WORKIING;
                    cpu_work_stats = WORK_WAIT;
                    num_packet_wait = atoi(current_task[1].c_str());
                    std::string str_num_wait_packets = current_task[1];
                    //std::string str_src_mem_index = current_task[2];
                    //tell_mem_send_data(str_src_mem_index,  str_num_wait_packets,  id);
                }
                else if (current_task[0] == "cal"){
                    //std::cout << "== cal" << std::endl;
                    //std::cout << "== cal curTick : " << curTick() <<std::endl;

                    communication_tick = curTick() - tick_pre;
                    //std::cout << "== ID"<<id << "  comm_tick : "<< communication_tick << std::endl;
                    
                    output_data(id, communication_tick , 1);

                    cpu_status = WORKIING;
                    cpu_work_stats = WORK_CAL;

                    stringstream stream;            //声明一个stringstream变量
                    stream << current_task[1];      //向stream中插入字符串"1234"
                    stream >> cal_cycles;           // 初始化cal_cycles 
                    cycles_caled = 0;
                }
                else if (current_task[0] == "send"){
                    //std::cout << "== send" << std::endl;
                    cpu_status = WORKIING;
                    cpu_work_stats = WORK_SEND; 
                    //packets_to_send = atoi(current_task[2].c_str());
                    //packets_to_send = current_task.size() - 1;
                    dst_num = current_task.size() - 1;
                    int dst_node = 0;
                    for( int i = 0 ; i < dst_num ; i++){
                        dst_node = atoi(current_task[i+1].c_str());
                        send_dst_list[packets_to_send] = dst_node;
                        packets_to_send += 1;
                        //send_output_line = "dst_node " + std::to_string(dst_node) + "  ;  num = " + std::to_string(send_merge[dst_node]) + "     ; curTick = " + std::to_string(curTick());
                        //send_command_output(id, send_output_line, pic_num);
                    }
                    
                    //send_output_line = "current packets_to_send = "+ std::to_string(packets_to_send) + "     ; curTick = " + std::to_string(curTick());
                    //send_command_output(id, send_output_line);
                    packets_sent = 0;
                }
                else if (strstr(current_task[0].c_str(), "finish") != NULL ) { // 最后一行current_task[0]会多一个终结符
                    cpu_status = IDLE;
                    cpu_work_stats = WORK_IDLE;
                    current_line_num = 0;
                    numPacketsSent = 0;
                    output_data_1(id, pic_num, "finish", 0, pic_num);
                    pic_num += 1;
                    if(pic_num > 1000){
                        cpu_status = FINISH;
                        cpu_work_stats = WORK_IDLE;
                    }
                }
            }
        }
    }
    // working status
    if (cpu_status == WORKIING){
        if (cpu_work_stats == WORK_WAIT){
            int packet_recv = recv_packets(id) - total_packet_recv_previous;
            if (packet_recv == 1){
                output_data_1(id, current_line_num-1, "recv first packet", 1, pic_num);
            }
            if (packet_recv == num_packet_wait){
                cpu_work_stats = WORK_IDLE;
                cpu_status = IDLE;
                total_packet_recv_previous += packet_recv;
            }
            // 否则维持wait状态
        }
        else if (cpu_work_stats == WORK_SEND){
            /*
            if (packets_sent == packets_to_send){  //TODO ++ packets_sent
                cpu_work_stats = WORK_IDLE;
                cpu_status = IDLE;
            }
            else {
                send_dst = send_dst_list[packets_sent];
                sendAllowedThisCycle = true;
            }
            */
            send_dst = send_dst_list[packets_sent];
            //send_output_line = "!!! dst_node " + std::to_string(send_dst) + "  ;  num = " + std::to_string(send_merge[send_dst]) + "     ; curTick = " + std::to_string(curTick());
            //send_command_output(id, send_output_line, pic_num);
            sendAllowedThisCycle = true;
            output_data_1(id, send_dst, "send", 0, pic_num);
            if (packets_sent == packets_to_send - 1){  //TODO ++ packets_sent
                cpu_work_stats = WORK_IDLE;
                cpu_status = IDLE;
                packets_to_send = 0;
            }
        }

        else if (cpu_work_stats == WORK_CAL){
            /*
            if (cycles_caled == cal_cycles){
                cpu_work_stats = WORK_IDLE;
                cpu_status = IDLE;
            }
            else {
                cycles_caled += 1;
            }
            */
            cycles_caled += 1;
            if (cycles_caled == cal_cycles){
                cpu_work_stats = WORK_IDLE;
                cpu_status = IDLE;
            }
        }
    }

    std::cout << "cpu id"<<id<<" status:" << cpu_status <<" current_line_num:"<< current_line_num << std::endl;
    std::cout << "cpu id"<<id<<" cpu_work_stats:" << cpu_work_stats <<" cur_tick:"<<curTick()<<std::endl;
	
	// std::cout<<" fanxi added GarnetSyntheticTraffic::tick(), id= "<< id << std::endl;
    if (++noResponseCycles >= responseLimit) {
        fatal("%s deadlocked at cycle %d\n", name(), curTick());
    }

    // make new request based on injection rate
    // (injection rate's range depends on precision)
    // - generate a random number between 0 and 10^precision
    // - send pkt if this number is < injRate*(10^precision)
    
    // double injRange = pow((double) 10, (double) precision);
    // unsigned trySending = random_mt.random<unsigned>(0, (int) injRange);
    // if (trySending < injRate*injRange)
    //     sendAllowedThisCycle = true;
    // else
    //     sendAllowedThisCycle = false;

    // always generatePkt unless fixedPkts or singleSender is enabled
    if (sendAllowedThisCycle) {
		// std::cout<<" fanxi added GarnetSyntheticTraffic: sendAllowedThisCycle id = "<< id << std::endl; 
        bool senderEnable = true;

        if (numPacketsMax >= 0 && numPacketsSent >= numPacketsMax)
            senderEnable = false;

        if (singleSender >= 0 && id != singleSender)
            senderEnable = false;

        if (senderEnable){
            generatePkt(send_dst);
            packets_sent += 1;
        }
           
    }

    // Schedule wakeup
    if (curTick() >= simCycles)
    {
        if(cpu_status != FINISH) output_data(id, current_line_num, 0);
        exitSimLoop("Network Tester completed simCycles");
    }
    else {
        if (!tickEvent.scheduled())
            schedule(tickEvent, clockEdge(Cycles(1)));
    }
}

//wxy add in 4.7
// 检测是否有可计算的列，若有则设置对应的cur_col_id，并判断当前是否结束计算。
bool
GarnetSyntheticTraffic::compute_check()
{
    int col_num = hold_col_list.size();
    std::vector <int> i_col_list;
    int flag = 0;
    int complete = 1;
    if(wait_num == 0){
        cur_col_id += 1;
        if(cur_col_id == col_num){
            complete = 1;
            compute_complete = complete;
            return 0;
        }
        else{
            complete = 0;
            compute_complete = complete;
            return 1;
        }
    }
    else
    {
        for (int i = 0; i < col_num ; i++){
            i_col_list = hold_col_list[i].i_col;
            if(!hold_col_list[i].flag){
                complete = 0;
                for (vector<int>::const_iterator iter = i_col_list.cbegin();iter != i_col_list.cend(); iter++) {
                    flag = is_element_in_vector(recv_col_id,(*iter));
                    if(!flag)
                        break;
                }
                if(flag){
                    compute_complete = complete;
                    cur_col_id = i;
                    return 1;
                }
            }
        }
    }
    
    compute_complete = complete;
    return 0;
}

// 输出当前接收到的列好
int recv_col(int id)
{
	std::string file;
    file = Node_recv + std::to_string(id)+"_col_id.txt";
	ifstream infile; 
    infile.open(file.data());  
    assert(infile.is_open());   

    string s;
    while(getline(infile,s))
    {
        //std::cout<<"fanxi added, recv_packets ing, id= " << id <<" packets="<<s<<std::endl;
    }
    infile.close();             //关闭文件输入流 
    return atoi(s.c_str());
}

// 检测是否接收到新的列
bool
GarnetSyntheticTraffic::recv_col_check()
{
    int recv_new = recv_packets(id) - total_col_recv_previous;
    if(recv_new == 0){
        return 0;
    }
    else{
        int col_id = recv_col(id);
        recv_col_id.push_back(col_id);
        total_col_recv_previous += 1;
        return 1;
    }
}
//wxy add in 4.7
//支持细粒度，修改中

//wxy add in 4.8
// 告知NI这个包是否是列的最后一个包， 1：是；0：不是。
void send_last(int id, int flag, int col_id){
    std::string file;
	file = Node_send+std::to_string(id)+".txt";
	ofstream OutFile;
    OutFile.open(file,ios::app); 
    OutFile<<std::to_string(flag)<<" "<<std::to_string(col_id)<<std::endl;
	OutFile.close();
}

void
GarnetSyntheticTraffic::tick()
{   
    //bool recv_flag;
    bool sendAllowedThisCycle = false;
    std::string send_output_line;
    // idle status : read task file

    // 每个周期都会检测是否有包收到
    if(!recv_col_complete){
        recv_col_check();
    }

    if (cpu_status == IDLE){
        if(wait_num == 0){
            //tick_pre = curTick();
            cpu_status = WORKIING;
            cpu_work_stats = WORK_WAIT;
            recv_col_complete = 1;
            // wxy add in 4.7 parameter 第一层的节点初始等待时间
            num_cycle_wait = setup_wait_num;
        }
        else{
            if(compute_check()){
                cpu_status = WORKIING;
                cpu_work_stats = WORK_CAL;
            }
            else if (compute_complete){
                output_data_1(id,0,"finish",0,pic_num);
                cpu_status = FINISH;
                cpu_work_stats = WORK_IDLE;
            }
        }
    }
    // working status
    if (cpu_status == WORKIING){
        if (cpu_work_stats == WORK_WAIT){
            if (wait_num == 0){
                if (num_cycle_wait == 0){
                    cpu_status = WORKIING;
                    cpu_work_stats = WORK_CAL;
                }
                num_cycle_wait -= 1;
            }
            // 否则维持wait状态
        }
        else if (cpu_work_stats == WORK_SEND){
            /*
            if (packets_sent == packets_to_send){  //TODO ++ packets_sent
                cpu_work_stats = WORK_IDLE;
                cpu_status = IDLE;
            }
            else {
                send_dst = send_dst_list[packets_sent];
                sendAllowedThisCycle = true;
            }
            */
            int node_id = floor(packets_sent / col_packet_num);
            send_dst = cur_col.send_node[node_id];
            //std::cout<<"wxy add send in node_id"<<node_id<<"  send_node_id"<< send_dst<<" packet_num "<<col_packet_num<<std::endl;
            //send_output_line = "!!! dst_node " + std::to_string(send_dst) + "  ;  num = " + std::to_string(send_merge[send_dst]) + "     ; curTick = " + std::to_string(curTick());
            //send_command_output(id, send_output_line, pic_num);
            sendAllowedThisCycle = true;
            output_data_1(id, cur_col.id, "send", 1, pic_num);
            if (packets_sent == packets_to_send - 1){  //TODO ++ packets_sent
                if(recv_col_complete){
                    cpu_work_stats = WORK_CAL;
                    cpu_status = WORKIING;
                    if(!compute_check()){
                        if(compute_complete){
                            output_data_1(id,0,"finish",0,pic_num);
                            cpu_work_stats = WORK_IDLE;
                            cpu_status = FINISH;
                        }
                        else{
                            std::cout<<"wxy add in GST.cc error"<<std::endl;
                        }
                    }
                }
                else{
                    cpu_work_stats = WORK_IDLE;
                    cpu_status = IDLE;
                }
            }
        }

        else if (cpu_work_stats == WORK_CAL){
            /*
            if (cycles_caled == cal_cycles){
                cpu_work_stats = WORK_IDLE;
                cpu_status = IDLE;
            }
            else {
                cycles_caled += 1;
            }
            */
            //std::cout<<"wxy add compute 1 in id"<<id<<std::endl;
            if (cycles_caled == 0){
                cur_col = hold_col_list[cur_col_id];
                packets_to_send = cur_col.send_node.size() * col_packet_num;
                output_data_1(id,cur_col.id,"cal",1,pic_num);
            }
            cycles_caled += 1;
            if (cycles_caled == col_compute_cycle){
                cycles_caled = 0;
                packets_sent = 0;
                hold_col_list[cur_col_id].flag = 1;
                if(send_flag){
                    cpu_work_stats = WORK_SEND;
                    cpu_status = WORKIING;
                }
                else{
                    cpu_work_stats = WORK_IDLE;
                    cpu_status = IDLE;
                }
            }
        }
    }

    std::cout << "cpu id"<<id<<" status:" << cpu_status <<" current_line_num:"<< current_line_num << std::endl;
    std::cout << "cpu id"<<id<<" cpu_work_stats:" << cpu_work_stats <<" cur_tick:"<<curTick()<<std::endl;
	
	// std::cout<<" fanxi added GarnetSyntheticTraffic::tick(), id= "<< id << std::endl;
    if (++noResponseCycles >= responseLimit) {
        fatal("%s deadlocked at cycle %d\n", name(), curTick());
    }

    // make new request based on injection rate
    // (injection rate's range depends on precision)
    // - generate a random number between 0 and 10^precision
    // - send pkt if this number is < injRate*(10^precision)
    
    // double injRange = pow((double) 10, (double) precision);
    // unsigned trySending = random_mt.random<unsigned>(0, (int) injRange);
    // if (trySending < injRate*injRange)
    //     sendAllowedThisCycle = true;
    // else
    //     sendAllowedThisCycle = false;

    // always generatePkt unless fixedPkts or singleSender is enabled
    if (sendAllowedThisCycle) {
		// std::cout<<" fanxi added GarnetSyntheticTraffic: sendAllowedThisCycle id = "<< id << std::endl; 
        bool senderEnable = true;

        if (numPacketsMax >= 0 && numPacketsSent >= numPacketsMax)
            senderEnable = false;

        if (singleSender >= 0 && id != singleSender)
            senderEnable = false;

        if (senderEnable){
            if ((packets_sent+1) % col_packet_num == 0){
                send_last(id, 1, cur_col.id);
                output_data_1(id, send_dst, "send_last", 0, pic_num);
                //std::cout<<"wxy add send is last yes"<<"   col_id "<<cur_col.id<<std::endl;
            }
            else{
                send_last(id, 0, cur_col.id);
                output_data_1(id, send_dst, "send_notlast", 0, pic_num);
                //std::cout<<"wxy add send is last no"<<"   col_id "<<cur_col.id<<std::endl;
            }
            generatePkt(send_dst);
            packets_sent += 1;
        }
           
    }

    // Schedule wakeup
    if (curTick() >= simCycles)
    {
        if(cpu_status != FINISH) output_data(id, current_line_num, 0);
        exitSimLoop("Network Tester completed simCycles");
    }
    else {
        if (!tickEvent.scheduled())
            schedule(tickEvent, clockEdge(Cycles(1)));
    }
}


//wxy add in 4.1
//实现与NI的数据交互，输入此刻传输的信息，输出文件夹send_data
void send_packet_data(int data, int src)
{
	std::string file;
    file = "./../output_info/send_data/"+std::to_string(src)+".txt";
	ofstream OutFile;
    OutFile.open(file,ios::app);
    OutFile<< std::to_string(data)<<std::endl;;  
    OutFile.close(); 
}

void
GarnetSyntheticTraffic::generatePkt(int send_dst)
{
	//std::cout<<" fanxi added GarnetSyntheticTraffic: generatePkt(), id: " <<id << std::endl; 
    int num_destinations = numDestinations;
    int radix = (int) sqrt(num_destinations);
    unsigned destination = id;
    int dest_x = -1;
    int dest_y = -1;
    int source = id;
    int src_x = id%radix;
    int src_y = id/radix;

    if (singleDest >= 0)
    {
        destination = singleDest;
    } else if (traffic == UNIFORM_RANDOM_) {
        destination = random_mt.random<unsigned>(0, num_destinations - 1);
    } else if (traffic == BIT_COMPLEMENT_) {
        dest_x = radix - src_x - 1;
        dest_y = radix - src_y - 1;
        destination = dest_y*radix + dest_x;
    } else if (traffic == BIT_REVERSE_) {
        unsigned int straight = source;
        unsigned int reverse = source & 1; // LSB

        int num_bits = (int) log2(num_destinations);

        for (int i = 1; i < num_bits; i++)
        {
            reverse <<= 1;
            straight >>= 1;
            reverse |= (straight & 1); // LSB
        }
        destination = reverse;
    } else if (traffic == BIT_ROTATION_) {
        if (source%2 == 0)
            destination = source/2;
        else // (source%2 == 1)
            destination = ((source/2) + (num_destinations/2));
    } else if (traffic == NEIGHBOR_) {
            dest_x = (src_x + 1) % radix;
            dest_y = src_y;
            destination = dest_y*radix + dest_x;
    } else if (traffic == SHUFFLE_) {
        if (source < num_destinations/2)
            destination = source*2;
        else
            destination = (source*2 - num_destinations + 1);
    } else if (traffic == TRANSPOSE_) {
            dest_x = src_y;
            dest_y = src_x;
            destination = dest_y*radix + dest_x;
    } else if (traffic == TORNADO_) {
        dest_x = (src_x + (int) ceil(radix/2)) % radix;
        dest_y = src_y;
        destination = dest_y*radix + dest_x;
    }
    else {
        fatal("Unknown Traffic Type: %s!\n", traffic);
    }

    // The source of the packets is a cache.
    // The destination of the packets is a directory.
    // The destination bits are embedded in the address after byte-offset.
    Addr paddr =  send_dst;  // fanxi modified
    paddr <<= blockSizeBits;
    unsigned access_size = 1; // Does not affect Ruby simulation

    // Modeling different coherence msg types over different msg classes.
    //
    // GarnetSyntheticTraffic assumes the Garnet_standalone coherence protocol
    // which models three message classes/virtual networks.
    // These are: request, forward, response.
    // requests and forwards are "control" packets (typically 8 bytes),
    // while responses are "data" packets (typically 72 bytes).
    //
    // Life of a packet from the tester into the network:
    // (1) This function generatePkt() generates packets of one of the
    //     following 3 types (randomly) : ReadReq, INST_FETCH, WriteReq
    // (2) mem/ruby/system/RubyPort.cc converts these to RubyRequestType_LD,
    //     RubyRequestType_IFETCH, RubyRequestType_ST respectively
    // (3) mem/ruby/system/Sequencer.cc sends these to the cache controllers
    //     in the coherence protocol.
    // (4) Network_test-cache.sm tags RubyRequestType:LD,
    //     RubyRequestType:IFETCH and RubyRequestType:ST as
    //     Request, Forward, and Response events respectively;
    //     and injects them into virtual networks 0, 1 and 2 respectively.
    //     It immediately calls back the sequencer.
    // (5) The packet traverses the network (simple/garnet) and reaches its
    //     destination (Directory), and network stats are updated.
    // (6) Network_test-dir.sm simply drops the packet.
    //
    MemCmd::Command requestType;

    RequestPtr req = nullptr;
    Request::Flags flags;

    // Inject in specific Vnet
    // Vnet 0 and 1 are for control packets (1-flit)
    // Vnet 2 is for data packets (5-flit)
    int injReqType = injVnet;
    //std::cout<<" fanx added the generated packet: destination= "<< send_dst << "; injVnet=" <<injVnet << std::endl;

    if (injReqType < 0 || injReqType > 2)
    {
        // randomly inject in any vnet
        injReqType = random_mt.random(0, 2);
    }

    if (injReqType == 0) {
        // generate packet for virtual network 0
        requestType = MemCmd::ReadReq;
        req = new Request(paddr, access_size, flags, masterId);
    } else if (injReqType == 1) {
        // generate packet for virtual network 1
        requestType = MemCmd::ReadReq;
        flags.set(Request::INST_FETCH);
        req = new Request(
            0, 0x0, access_size, flags, masterId, 0x0, 0);
        req->setPaddr(paddr);
    } else {  // if (injReqType == 2)
        // generate packet for virtual network 2
        requestType = MemCmd::WriteReq;
        req = new Request(paddr, access_size, flags, masterId);
    }
    //std::cout<<"wxy added the generated packet: masterId =  "<< masterId << std::endl;
    req->setContext(id);

    //No need to do functional simulation
    //We just do timing simulation of the network

    DPRINTF(GarnetSyntheticTraffic,
            "Generated packet with destination %d, embedded in address %x\n",
            destination, req->getPaddr());

    PacketPtr pkt = new Packet(req, requestType);
    pkt->dataDynamic(new uint8_t[req->getSize()]);
    pkt->senderState = NULL;

    //send_packet_data(curTick(), id);
    sendPkt(pkt);
}

void
GarnetSyntheticTraffic::initTrafficType()
{
    trafficStringToEnum["bit_complement"] = BIT_COMPLEMENT_;
    trafficStringToEnum["bit_reverse"] = BIT_REVERSE_;
    trafficStringToEnum["bit_rotation"] = BIT_ROTATION_;
    trafficStringToEnum["neighbor"] = NEIGHBOR_;
    trafficStringToEnum["shuffle"] = SHUFFLE_;
    trafficStringToEnum["tornado"] = TORNADO_;
    trafficStringToEnum["transpose"] = TRANSPOSE_;
    trafficStringToEnum["uniform_random"] = UNIFORM_RANDOM_;
}

void
GarnetSyntheticTraffic::doRetry()
{
    if (cachePort.sendTimingReq(retryPkt)) {
        retryPkt = NULL;
    }
}

void
GarnetSyntheticTraffic::printAddr(Addr a)
{
    cachePort.printAddr(a);
}

GarnetSyntheticTraffic *
GarnetSyntheticTrafficParams::create()
{   
    //std::cout<<" fanxi added GarnetSyntheticTrafficParams::create() " << std::endl;  
    // called by 16 times at the begining to new 16 cpus
    return new GarnetSyntheticTraffic(this);
}

