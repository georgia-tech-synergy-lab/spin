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


#ifndef __MEM_RUBY_NETWORK_GARNET_FLIT_HH__
#define __MEM_RUBY_NETWORK_GARNET_FLIT_HH__

#include <cassert>
#include <iostream>
#include <queue>

#include "base/types.hh"
#include "mem/ruby/network/garnet2.0/CommonTypes.hh"
#include "mem/ruby/slicc_interface/Message.hh"
#include "mem/ruby/network/garnet2.0/GarnetNetwork.hh"

class flit
{
  public:
  flit() 
  {
    must_send = false;
    part_of_move = false;
    //std::cout<<"hello from flit constructor 1\n"<<std::flush;
  }
    flit(int id, int vc, int vnet, RouteInfo route, int size,
         MsgPtr msg_ptr, Cycles curTime);

  flit(int source_id, int source_inp_port, int source_vc, int source_out_port, int vnet, flit_type type, 
       Cycles curTime, const std::queue<int> &path, GarnetNetwork *net_ptr);

  flit(int source_id, const std::queue<int> &path, Cycles curTime, int inport);
  flit(flit *t_flit, Cycles curTime, int outport);

    int get_outport() {return m_outport; }
    int get_size() { return m_size; }
    Cycles get_enqueue_time() { return m_enqueue_time; }
    int get_id() { return m_id; }
    Cycles get_time() { return m_time; }
    int get_vnet() { return m_vnet; }
    int get_vc() { return m_vc; }
  RouteInfo get_route() { return m_route; }
  RouteInfo* get_route_ptr() { return (&m_route); }
    MsgPtr& get_msg_ptr() { return m_msg_ptr; }
    flit_type get_type() { return m_type; }
    std::pair<flit_stage, Cycles> get_stage() { return m_stage; }
    Cycles get_src_delay() { return src_delay; }

    void set_outport(int port) { m_outport = port; }
    void set_time(Cycles time) { m_time = time; }
    void set_vc(int vc) { m_vc = vc; }
    void set_route(RouteInfo route) { m_route = route; }
    void set_src_delay(Cycles delay) { src_delay = delay; }

    void increment_hops() { m_route.hops_traversed++; }
    void print(std::ostream& out) const;

    bool
    is_stage(flit_stage stage, Cycles time)
    {
        return (stage == m_stage.first &&
                time >= m_stage.second);
    }

    void
    advance_stage(flit_stage t_stage, Cycles newTime)
    {
        m_stage.first = t_stage;
        m_stage.second = newTime;
    }

  static bool greater(flit* n1, flit* n2);

    bool functionalWrite(Packet *pkt);

  //spin scheme:
  void add_delay(Cycles c) { delay += c;}
  void sub_delay(Cycles c) 
  {
    // std::cout<<"flit type is "<<m_type<<std::endl<<std::flush;

    assert(c <= delay); 
    delay = delay - c;
  }

  Cycles get_delay() { return delay; }
  int get_source_id() {return source_id; }
  void set_inport(int port) { inport = port; }
  int get_inport() { return inport; }
  int get_source_inp_port() { return source_inp_port; }
  int get_source_vc() { return source_vc; }
  std::queue<int> get_path() { return path; }
  int get_path_top();
  int peek_path_top() { return path.front(); }
  void set_must_send() { must_send = true; }
  void reset_must_send() { must_send = false; }
  bool get_must_send() { return must_send; }
  void set_part_of_move() { part_of_move = true; }
  void reset_part_of_move() { part_of_move = false; }
  bool is_part_of_move() { return part_of_move; }
  unsigned get_num_turns() { return path.size(); }
  //void print_path();

  //dragon-fly deadlock avoidance
  int get_hops() { return m_route.hops_traversed; }

  GarnetNetwork* get_net_ptr() { return m_net_ptr; }

  protected:
    int m_id;
    int m_vnet;
    int m_vc;
    RouteInfo m_route;
    int m_size;
    Cycles m_enqueue_time, m_time;
    flit_type m_type;
    MsgPtr m_msg_ptr;
    int m_outport;
    Cycles src_delay;
    std::pair<flit_stage, Cycles> m_stage;

  static GarnetNetwork *m_net_ptr;

  //spin scheme
  int source_id;
  int source_inp_port;
  int source_vc;
  std::queue<int> path;  //contains output ports of routers
  Cycles delay;
  int inport;
  bool must_send = false;
  bool part_of_move = false;

  static int probe_id;
  static int move_id;
  static int kill_move_id;
  static int check_probe_id;
};

inline std::ostream&
operator<<(std::ostream& out, const flit& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_NETWORK_GARNET_FLIT_HH__
