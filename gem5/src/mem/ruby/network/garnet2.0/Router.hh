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


#ifndef __MEM_RUBY_NETWORK_GARNET_ROUTER_HH__
#define __MEM_RUBY_NETWORK_GARNET_ROUTER_HH__

#include <iostream>
#include <vector>
#include <queue>

#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/BasicRouter.hh"
#include "mem/ruby/network/garnet2.0/CommonTypes.hh"
#include "mem/ruby/network/garnet2.0/GarnetNetwork.hh"
#include "mem/ruby/network/garnet2.0/flit.hh"
#include "params/GarnetRouter.hh"

class NetworkLink;
class CreditLink;
class InputUnit;
class OutputUnit;
class RoutingUnit;
class SwitchAllocator;
class CrossbarSwitch;
class FaultModel;
class flitBuffer;

class Router : public BasicRouter, public Consumer
{
  public:
    typedef GarnetRouterParams Params;
    Router(const Params *p);

    ~Router();

    void wakeup();
    void print(std::ostream& out) const {};

    void init();
    void addInPort(PortDirection inport_dirn, NetworkLink *link,
                   CreditLink *credit_link);
    void addOutPort(PortDirection outport_dirn, NetworkLink *link,
                    const NetDest& routing_table_entry,
                    int link_weight, CreditLink *credit_link);

    Cycles get_pipe_stages(){ return m_latency; }
    int get_num_vcs()       { return m_num_vcs; }
    int get_num_vnets()     { return m_virtual_networks; }
    int get_vc_per_vnet()   { return m_vc_per_vnet; }
    int get_num_inports()   { return m_input_unit.size(); }
    int get_num_outports()  { return m_output_unit.size(); }
    int get_id()            { return m_id; }
    bool has_free_vc(int outport, int vnet);

    void init_net_ptr(GarnetNetwork* net_ptr)
    {
        m_network_ptr = net_ptr;
    }

    GarnetNetwork* get_net_ptr()                    { return m_network_ptr; }
    std::vector<InputUnit *>& get_inputUnit_ref()   { return m_input_unit; }
    std::vector<OutputUnit *>& get_outputUnit_ref() { return m_output_unit; }
    PortDirection getOutportDirection(int outport);
    PortDirection getInportDirection(int inport);

  int route_compute(RouteInfo *route, int inport, PortDirection direction, int invc);
    void grant_switch(int inport, flit *t_flit);
    void schedule_wakeup(Cycles time);

    std::string getPortDirectionName(PortDirection direction);
    void printFaultVector(std::ostream& out);
    void printAggregateFaultProbability(std::ostream& out);

    void regStats();
    void collateStats();
    void resetStats();
  
  //spin scheme
  /*counter* create_counter();
  Router::path_buffer* create_path_buffer();
  Router::source_id_buffer* create_source_id_buffer();*/

  void init_spin_scheme_ptr();
  
  
  void set_counter(unsigned input_port, unsigned vc, Counter_state state, unsigned thresh);
  Counter_state get_counter_state() { return m_counter->state; }
  void increment_counter_ptr();
  bool check_counter_ptr(unsigned inport, unsigned invc);
  inline Cycles get_loop_delay() { return loop_delay; }
  void set_loop_delay (Cycles c) { loop_delay = c; }
  void check_counter_timeout();
  int get_counter_inport() { return m_counter->cptr->input_port; }
  int get_counter_vc() { return m_counter->cptr->vc; }
  bool spin_scheme_enabled() { return m_network_ptr->isSpinSchemeEnabled(); }
  unsigned get_max_turn_capacity() { return m_network_ptr->get_max_turn_capacity(); }
  
  void latch_path(flit *probe) 
  {
    m_path_buffer->path = probe->get_path();
    m_path_buffer->valid = true;
  }

  int peek_path_top()
  {
    assert(m_path_buffer->valid);
    return m_path_buffer->path.front();
  }

  std::queue<int> get_path_buffer_path() { return m_path_buffer->path; }
  int get_path_buffer_length();

  void invalidate_path_buffer();

  void latch_source_id_buffer(int source_id, int move_id);
  void invalidate_source_id_buffer();
  bool check_source_id_buffer(int source_id, int move_id);
  bool partial_check_source_id_buffer(int source_id);

  void set_move_bit() { m_move = true; }
  void reset_move_bit() { m_move = false; }
  bool get_move_bit() { return m_move; }

  inline void set_kill_move_processed_this_cycle() { kill_move_processed_this_cycle = true; }
  inline void reset_kill_move_processed_this_cycle() { kill_move_processed_this_cycle = false; }
  inline bool get_kill_move_processed_this_cycle() { return kill_move_processed_this_cycle; }
  

  flitBuffer* getprobeQueue_ptr() { return probeQueue; }
  flitBuffer* get_moveQueue_ptr() { return moveQueue; }
  flitBuffer* get_kill_moveQueue_ptr() {return kill_moveQueue; }
  flitBuffer* get_check_probeQueue_ptr() {return check_probeQueue; }

  void set_start_move() { start_move = true; }
  void reset_start_move() { start_move = false; }
  bool get_start_move() { return start_move; }



  /*void set_move_vc_at_downstream_router(int vc);
  int get_move_vc_at_downstream_router() { return move_vc_at_downstream_router; }
  void invalidate_move_vc_at_downstream_router() { move_vc_at_downstream_router = -1; }*/

  /*void set_move_outport(int outport) { move_outport = outport; }
  int get_move_outport() { return move_outport; }
  void invalidate_move_outport() { move_outport=-1; }*/

  void reset_spin_stats();
  unsigned get_num_probes_sent() { return num_probes_sent; }
  unsigned get_num_move_sent() { return num_move_sent; }
  unsigned get_num_kill_move_sent() { return num_kill_move_sent; }
  unsigned get_num_probes_dropped();
  unsigned get_num_move_dropped();
  unsigned get_num_kill_move_dropped();
  unsigned get_num_spins() { return num_spins; }
  unsigned get_num_spin_cycles();
  unsigned get_max_spin_cycles();
  unsigned get_max_deadlock_path_length() { return max_deadlock_path_length; }
  unsigned get_deadlock_path_length_sum() { return deadlock_path_length_sum; }
  unsigned get_num_check_probe_dropped();
  unsigned get_num_check_probe_sent() { return num_check_probe_sent; }

  int send_move_msg(int inport, int vc);
  void fork_probes(flit *t_flit, const std::vector<bool> &fork_vector);
  void send_kill_move(int inport);
  void forward_kill_move(flit *kill_move);
  void move_complete();
  //bool is_vc_frozen(int inport, int vc);
  void forward_move(flit *move);
  void forward_check_probe(flit *check_probe);
  void send_probe();
  void create_move_info_entry(int inport, int vc, int outport);
  void clear_move_registry();
  const std::vector<move_info *>& get_move_registry() { return move_registry; }
  void update_move_vc_at_downstream_router(int vc, int outport);
  int get_num_move_registry_entries() { return move_registry.size(); }
  void invalidate_move_registry_entry(int inport, int outport);
  void update_move_info_entry(int inport, int vc, int outport);
  void invalidate_move_vcs();
  void send_check_probe(int inport, int vc);
  bool check_outport_entry_in_move_registry(int outport);

  bool is_sb_node() { return m_is_sb_node; }
  bool sb_placement_enabled() { return m_network_ptr->is_sb_placement_enabled(); }
  unsigned get_my_dd_thresh();

  //UGAL routing
  int get_num_active_vcs(int outport, int vnet);
  int get_num_hops(int vnet, NetDest net_dest);
  

    // For Fault Model:
    bool get_fault_vector(int temperature, float fault_vector[]) {
        return m_network_ptr->fault_model->fault_vector(m_id, temperature,
                                                        fault_vector);
    }
    bool get_aggregate_fault_probability(int temperature,
                                         float *aggregate_fault_prob) {
        return m_network_ptr->fault_model->fault_prob(m_id, temperature,
                                                      aggregate_fault_prob);
    }

    uint32_t functionalWrite(Packet *);

  //spin scheme

  struct pointer
  {
    unsigned input_port;
    unsigned vc;
    unsigned vnet;
  };
  
  struct counter
  {
    pointer *cptr;
    unsigned count;
    Cycles thresh;
    Counter_state state;

    ~counter()
    {
      delete cptr;
    }
  };

  struct path_buffer
  {
    std::queue<int> path;
    bool valid;
  };

  struct source_id_buffer
  {
    int source_id;
    int move_id;
    bool valid;
  };


  //1-vc-per-vnet routing
  Cycles get_vc_active_time(int outport, int vc);


  private:
    Cycles m_latency;
    int m_virtual_networks, m_num_vcs, m_vc_per_vnet;
    GarnetNetwork *m_network_ptr;

    std::vector<InputUnit *> m_input_unit;
    std::vector<OutputUnit *> m_output_unit;
    RoutingUnit *m_routing_unit;
    SwitchAllocator *m_sw_alloc;
    CrossbarSwitch *m_switch;

  //spin scheme

  counter *m_counter;
  path_buffer *m_path_buffer;
  bool m_move;
  source_id_buffer *m_source_id_buffer;
  Cycles loop_delay;
  flitBuffer *probeQueue;
  flitBuffer *moveQueue;
  flitBuffer *kill_moveQueue;
  flitBuffer *check_probeQueue;
  bool kill_move_processed_this_cycle;
  bool start_move;
  //int move_vc_at_downstream_router;
  //int move_outport;
  std::vector<move_info *> move_registry;

  //spin scheme : sb_placement
  bool m_is_sb_node;

  //spin scheme: tdm
  bool counter_expired_in_dd;

  //spin scheme stat variables
  unsigned num_spins;
  unsigned num_probes_sent;
  unsigned num_move_sent;
  unsigned num_check_probe_sent;
  unsigned num_kill_move_sent;
  unsigned deadlock_path_length_sum;
  unsigned max_deadlock_path_length;

    // Statistical variables required for power computations
    Stats::Scalar m_buffer_reads;
    Stats::Scalar m_buffer_writes;

    Stats::Scalar m_sw_input_arbiter_activity;
    Stats::Scalar m_sw_output_arbiter_activity;

    Stats::Scalar m_crossbar_activity;


  //spin scheme : stat variables

  Stats::Scalar m_num_probes_sent;
  Stats::Scalar m_num_move_sent;
  Stats::Scalar m_num_kill_move_sent;
  Stats::Scalar m_num_check_probe_sent;
  
  Stats::Scalar m_num_probes_dropped;
  Stats::Scalar m_num_move_dropped;
  Stats::Scalar m_num_kill_move_dropped;
  Stats::Scalar m_num_check_probe_dropped;
  
  Stats::Scalar m_num_spins;
  Stats::Scalar m_num_total_spin_cycles;
  Stats::Scalar m_max_spin_cycles;

  Stats::Scalar m_max_deadlock_path_length;
  Stats::Scalar m_deadlock_path_length_sum;

};

#endif // __MEM_RUBY_NETWORK_GARNET_ROUTER_HH__
