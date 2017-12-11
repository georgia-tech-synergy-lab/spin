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


#include "mem/ruby/network/garnet2.0/GarnetNetwork.hh"

#include <cassert>

#include "base/cast.hh"
#include "base/stl_helpers.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/network/garnet2.0/CommonTypes.hh"
#include "mem/ruby/network/garnet2.0/CreditLink.hh"
#include "mem/ruby/network/garnet2.0/GarnetLink.hh"
#include "mem/ruby/network/garnet2.0/NetworkInterface.hh"
#include "mem/ruby/network/garnet2.0/NetworkLink.hh"
#include "mem/ruby/network/garnet2.0/Router.hh"
#include "mem/ruby/system/RubySystem.hh"

using namespace std;
using m5::stl_helpers::deletePointers;


/*
 * GarnetNetwork sets up the routers and links and collects stats.
 * Default parameters (GarnetNetwork.py) can be overwritten from command line
 * (see configs/network/Network.py)
 */

GarnetNetwork::GarnetNetwork(const Params *p)
    : Network(p)
{
    m_num_rows = p->num_rows;
    m_ni_flit_size = p->ni_flit_size;
    m_vcs_per_vnet = p->vcs_per_vnet;
    m_buffers_per_data_vc = p->buffers_per_data_vc;
    m_buffers_per_ctrl_vc = p->buffers_per_ctrl_vc;
    m_routing_algorithm = p->routing_algorithm;

    //spin scheme
    spin_scheme_enabled = p->enable_spin_scheme;
    dd_thresh = p->dd_thresh;
    max_turn_capacity = p->max_turn_capacity;
    sb_placement_enabled = p->enable_sb_placement;
    variable_dd_enabled = p->enable_variable_dd;
    enable_rotating_priority = p->enable_rotating_priority;

    //minimal adaptive for 1-vc
    staleness_thresh = p->staleness_thresh;
    topology = p->topology;
    dfly_group_size = p->dfly_group_size;

    //dragon-fly deadlock avoidance
    enable_dfly_dlock_avoidance = p->enable_dfly_dlock_avoidance;

    //3-d fbfly deadlock avoidance
    enable_fbfly_vc_partition = p->enable_fbfly_vc_partition;

    //escape-vc
    enable_escape_vc = p->enable_escape_vc;

    m_enable_fault_model = p->enable_fault_model;
    if (m_enable_fault_model)
        fault_model = p->fault_model;

    m_vnet_type.resize(m_virtual_networks);

    for (int i = 0 ; i < m_virtual_networks ; i++) {
        if (m_vnet_type_names[i] == "response")
            m_vnet_type[i] = DATA_VNET_; // carries data (and ctrl) packets
        else
            m_vnet_type[i] = CTRL_VNET_; // carries only ctrl packets
    }

    // record the routers
    for (vector<BasicRouter*>::const_iterator i =  p->routers.begin();
         i != p->routers.end(); ++i) {
        Router* router = safe_cast<Router*>(*i);
        m_routers.push_back(router);

        // initialize the router's network pointers
        router->init_net_ptr(this);
    }

    // record the network interfaces
    for (vector<ClockedObject*>::const_iterator i = p->netifs.begin();
         i != p->netifs.end(); ++i) {
        NetworkInterface *ni = safe_cast<NetworkInterface *>(*i);
        m_nis.push_back(ni);
        ni->init_net_ptr(this);
    }
}

void
GarnetNetwork::init()
{
    Network::init();

    for (int i=0; i < m_nodes; i++) {
        m_nis[i]->addNode(m_toNetQueues[i], m_fromNetQueues[i]);
    }

    // The topology pointer should have already been initialized in the
    // parent network constructor
    assert(m_topology_ptr != NULL);
    m_topology_ptr->createLinks(this);

    // Initialize topology specific parameters
    if (getNumRows() > 0) {
        // Only for Mesh topology
        // m_num_rows and m_num_cols are only used for
        // implementing XY or custom routing in RoutingUnit.cc
        m_num_rows = getNumRows();
        m_num_cols = m_routers.size() / m_num_rows;
        assert(m_num_rows * m_num_cols == m_routers.size());
    } else {
        m_num_rows = -1;
        m_num_cols = -1;
    }

    // FaultModel: declare each router to the fault model
    if (isFaultModelEnabled()) {
        for (vector<Router*>::const_iterator i= m_routers.begin();
             i != m_routers.end(); ++i) {
            Router* router = safe_cast<Router*>(*i);
            int router_id M5_VAR_USED =
                fault_model->declare_router(router->get_num_inports(),
                                            router->get_num_outports(),
                                            router->get_vc_per_vnet(),
                                            getBuffersPerDataVC(),
                                            getBuffersPerCtrlVC());
            assert(router_id == router->get_id());
            router->printAggregateFaultProbability(cout);
            router->printFaultVector(cout);
        }
    }
}

GarnetNetwork::~GarnetNetwork()
{
    deletePointers(m_routers);
    deletePointers(m_nis);
    deletePointers(m_networklinks);
    deletePointers(m_creditlinks);
}

/*
 * This function creates a link from the Network Interface (NI)
 * into the Network.
 * It creates a Network Link from the NI to a Router and a Credit Link from
 * the Router to the NI
*/

void
GarnetNetwork::makeExtInLink(NodeID src, SwitchID dest, BasicLink* link,
                            const NetDest& routing_table_entry)
{
    assert(src < m_nodes);

    GarnetExtLink* garnet_link = safe_cast<GarnetExtLink*>(link);

    // GarnetExtLink is bi-directional
    NetworkLink* net_link = garnet_link->m_network_links[LinkDirection_In];
    net_link->setType(EXT_IN_);
    CreditLink* credit_link = garnet_link->m_credit_links[LinkDirection_In];

    m_networklinks.push_back(net_link);
    m_creditlinks.push_back(credit_link);

    PortDirection dst_inport_dirn = "Local";
    m_routers[dest]->addInPort(dst_inport_dirn, net_link, credit_link);
    m_nis[src]->addOutPort(net_link, credit_link, dest);
}

/*
 * This function creates a link from the Network to a NI.
 * It creates a Network Link from a Router to the NI and
 * a Credit Link from NI to the Router
*/

void
GarnetNetwork::makeExtOutLink(SwitchID src, NodeID dest, BasicLink* link,
                             const NetDest& routing_table_entry)
{
    assert(dest < m_nodes);
    assert(src < m_routers.size());
    assert(m_routers[src] != NULL);

    GarnetExtLink* garnet_link = safe_cast<GarnetExtLink*>(link);

    // GarnetExtLink is bi-directional
    NetworkLink* net_link = garnet_link->m_network_links[LinkDirection_Out];
    net_link->setType(EXT_OUT_);
    CreditLink* credit_link = garnet_link->m_credit_links[LinkDirection_Out];

    m_networklinks.push_back(net_link);
    m_creditlinks.push_back(credit_link);

    PortDirection src_outport_dirn = "Local";
    m_routers[src]->addOutPort(src_outport_dirn, net_link,
                               routing_table_entry,
                               link->m_weight, credit_link);
    m_nis[dest]->addInPort(net_link, credit_link);
}

/*
 * This function creates an internal network link between two routers.
 * It adds both the network link and an opposite credit link.
*/

void
GarnetNetwork::makeInternalLink(SwitchID src, SwitchID dest, BasicLink* link,
                                const NetDest& routing_table_entry,
                                PortDirection src_outport_dirn,
                                PortDirection dst_inport_dirn)
{
    GarnetIntLink* garnet_link = safe_cast<GarnetIntLink*>(link);

    // GarnetIntLink is unidirectional
    NetworkLink* net_link = garnet_link->m_network_link;
    net_link->setType(INT_);
    CreditLink* credit_link = garnet_link->m_credit_link;

    m_networklinks.push_back(net_link);
    m_creditlinks.push_back(credit_link);

    m_routers[dest]->addInPort(dst_inport_dirn, net_link, credit_link);
    m_routers[src]->addOutPort(src_outport_dirn, net_link,
                               routing_table_entry,
                               link->m_weight, credit_link);
}

// Total routers in the network
int
GarnetNetwork::getNumRouters()
{
    return m_routers.size();
}

// Get ID of router connected to a NI.
int
GarnetNetwork::get_router_id(int ni)
{
    return m_nis[ni]->get_router_id();
}

void
GarnetNetwork::regStats()
{
    Network::regStats();

    // Packets
    m_packets_received
        .init(m_virtual_networks)
        .name(name() + ".packets_received")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_packets_injected
        .init(m_virtual_networks)
        .name(name() + ".packets_injected")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_packet_network_latency
        .init(m_virtual_networks)
        .name(name() + ".packet_network_latency")
        .flags(Stats::oneline)
        ;

    m_packet_queueing_latency
        .init(m_virtual_networks)
        .name(name() + ".packet_queueing_latency")
        .flags(Stats::oneline)
        ;

    for (int i = 0; i < m_virtual_networks; i++) {
        m_packets_received.subname(i, csprintf("vnet-%i", i));
        m_packets_injected.subname(i, csprintf("vnet-%i", i));
        m_packet_network_latency.subname(i, csprintf("vnet-%i", i));
        m_packet_queueing_latency.subname(i, csprintf("vnet-%i", i));
    }

    m_avg_packet_vnet_latency
        .name(name() + ".average_packet_vnet_latency")
        .flags(Stats::oneline);
    m_avg_packet_vnet_latency =
        m_packet_network_latency / m_packets_received;

    m_avg_packet_vqueue_latency
        .name(name() + ".average_packet_vqueue_latency")
        .flags(Stats::oneline);
    m_avg_packet_vqueue_latency =
        m_packet_queueing_latency / m_packets_received;

    m_avg_packet_network_latency
        .name(name() + ".average_packet_network_latency");
    m_avg_packet_network_latency =
        sum(m_packet_network_latency) / sum(m_packets_received);

    m_avg_packet_queueing_latency
        .name(name() + ".average_packet_queueing_latency");
    m_avg_packet_queueing_latency
        = sum(m_packet_queueing_latency) / sum(m_packets_received);

    m_avg_packet_latency
        .name(name() + ".average_packet_latency");
    m_avg_packet_latency
        = m_avg_packet_network_latency + m_avg_packet_queueing_latency;

    // Flits
    m_flits_received
        .init(m_virtual_networks)
        .name(name() + ".flits_received")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_flits_injected
        .init(m_virtual_networks)
        .name(name() + ".flits_injected")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_flit_network_latency
        .init(m_virtual_networks)
        .name(name() + ".flit_network_latency")
        .flags(Stats::oneline)
        ;

    m_flit_queueing_latency
        .init(m_virtual_networks)
        .name(name() + ".flit_queueing_latency")
        .flags(Stats::oneline)
        ;

    for (int i = 0; i < m_virtual_networks; i++) {
        m_flits_received.subname(i, csprintf("vnet-%i", i));
        m_flits_injected.subname(i, csprintf("vnet-%i", i));
        m_flit_network_latency.subname(i, csprintf("vnet-%i", i));
        m_flit_queueing_latency.subname(i, csprintf("vnet-%i", i));
    }

    m_avg_flit_vnet_latency
        .name(name() + ".average_flit_vnet_latency")
        .flags(Stats::oneline);
    m_avg_flit_vnet_latency = m_flit_network_latency / m_flits_received;

    m_avg_flit_vqueue_latency
        .name(name() + ".average_flit_vqueue_latency")
        .flags(Stats::oneline);
    m_avg_flit_vqueue_latency =
        m_flit_queueing_latency / m_flits_received;

    m_avg_flit_network_latency
        .name(name() + ".average_flit_network_latency");
    m_avg_flit_network_latency =
        sum(m_flit_network_latency) / sum(m_flits_received);

    m_avg_flit_queueing_latency
        .name(name() + ".average_flit_queueing_latency");
    m_avg_flit_queueing_latency =
        sum(m_flit_queueing_latency) / sum(m_flits_received);

    m_avg_flit_latency
        .name(name() + ".average_flit_latency");
    m_avg_flit_latency =
        m_avg_flit_network_latency + m_avg_flit_queueing_latency;


    // Hops
    m_avg_hops.name(name() + ".average_hops");
    m_avg_hops = m_total_hops / sum(m_flits_received);

    // Links
    m_total_ext_in_link_utilization
        .name(name() + ".ext_in_link_utilization");
    m_total_ext_out_link_utilization
        .name(name() + ".ext_out_link_utilization");
    m_total_int_link_utilization
        .name(name() + ".int_link_utilization");
    m_average_link_utilization
        .name(name() + ".avg_link_utilization");

    m_average_vc_load
        .init(m_virtual_networks * m_vcs_per_vnet)
        .name(name() + ".avg_vc_load")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;



    //spin scheme
    num_total_probes_sent
      .name(name() + ".total_probes_sent")
      ;

    num_total_move_sent
      .name(name() + ".total_move_sent")
      ;

    num_total_kill_move_sent
      .name(name() + ".total_kill_move_sent")
      ;

    num_total_check_probe_sent
      .name(name() + ".total_check_probe_sent")
      ;

    num_total_probes_dropped
      .name(name() + ".total_probes_dropped")
      ;

    num_total_move_dropped
      .name(name() + ".total_move_dropped")
      ;

    num_total_kill_move_dropped
      .name(name() + ".total_kill_move_dropped")
      ;

    num_total_check_probe_dropped
      .name(name() + ".total_check_probe_dropped")
      ;

    num_total_spins
      .name(name() + ".total_spins")
      ;

    num_total_spin_cycles
      .name(name() + ".total_spin_cycles")
      ;

    network_max_spin_cycles
      .name(name() + ".network_max_spin_cycles")
      ;

    network_deadlock_path_length_sum
      .name(name() + ".network_deadlock_path_length_sum")
      ;

    network_max_deadlock_path_length
      .name(name() + ".network_max_deadlock_path_length")
      ;

    network_probe_link_utilisation
      .name(name() + ".probe_link_utilisation")
      ;

    network_move_link_utilisation
      .name(name() + ".move_link_utilisation")
      ;
    
    network_kill_move_link_utilisation
      .name(name() + ".kill_move_link_utilisation")
      ;
    
    network_flit_link_utilisation
      .name(name() + ".flit_link_utilisation")
      ;

    network_check_probe_link_utilisation
      .name(name() + ".check_probe_link_utilisation")
      ;
}

void
GarnetNetwork::collateStats()
{
    RubySystem *rs = params()->ruby_system;
    double time_delta = double(curCycle() - rs->getStartCycle());

    for (int i = 0; i < m_networklinks.size(); i++) {
        link_type type = m_networklinks[i]->getType();
        int activity = m_networklinks[i]->getLinkUtilization();

        if (type == EXT_IN_)
            m_total_ext_in_link_utilization += activity;
        else if (type == EXT_OUT_)
            m_total_ext_out_link_utilization += activity;
        else if (type == INT_)
            m_total_int_link_utilization += activity;

        m_average_link_utilization +=
            (double(activity) / time_delta);

        vector<unsigned int> vc_load = m_networklinks[i]->getVcLoad();
        for (int j = 0; j < vc_load.size(); j++) {
            m_average_vc_load[j] += ((double)vc_load[j] / time_delta);
        }
	

	//spin scheme
	network_probe_link_utilisation += m_networklinks[i]->getLinkUtilization_probe();
	network_move_link_utilisation += m_networklinks[i]->getLinkUtilization_move();
	network_kill_move_link_utilisation += m_networklinks[i]->getLinkUtilization_kill_move();
	network_flit_link_utilisation += m_networklinks[i]->getLinkUtilization_flits();
	network_check_probe_link_utilisation += m_networklinks[i]->getLinkUtilization_check_probe();
    }

    unsigned int max_deadlock_path_length = 0;
    unsigned int max_spin_cycles = 0;

    // Ask the routers to collate their statistics
    for (int i = 0; i < m_routers.size(); i++) {
        m_routers[i]->collateStats();

	//spin scheme
	num_total_probes_sent += m_routers[i]->get_num_probes_sent();
	num_total_move_sent += m_routers[i]->get_num_move_sent();
	num_total_kill_move_sent += m_routers[i]->get_num_kill_move_sent();
	num_total_check_probe_sent += m_routers[i]->get_num_check_probe_sent();

	num_total_probes_dropped += m_routers[i]->get_num_probes_dropped();
	num_total_move_dropped += m_routers[i]->get_num_move_dropped();
	num_total_kill_move_dropped += m_routers[i]->get_num_kill_move_dropped();
	num_total_check_probe_dropped += m_routers[i]->get_num_check_probe_dropped();

	num_total_spins += m_routers[i]->get_num_spins(); //only source router tracks the spin and cycles
	num_total_spin_cycles += m_routers[i]->get_num_spin_cycles();

	if(m_routers[i]->get_max_spin_cycles() > max_spin_cycles)
	  max_spin_cycles = m_routers[i]->get_max_spin_cycles();


	if(m_routers[i]->get_max_deadlock_path_length() > max_deadlock_path_length)
	  {
	    max_deadlock_path_length = m_routers[i]->get_max_deadlock_path_length();
	  }
	
	network_deadlock_path_length_sum += m_routers[i]->get_deadlock_path_length_sum();
    }

    network_max_deadlock_path_length = max_deadlock_path_length;
    network_max_spin_cycles = max_spin_cycles;    
}

void
GarnetNetwork::print(ostream& out) const
{
    out << "[GarnetNetwork]";
}

GarnetNetwork *
GarnetNetworkParams::create()
{
    return new GarnetNetwork(this);
}

uint32_t
GarnetNetwork::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;

    for (unsigned int i = 0; i < m_routers.size(); i++) {
        num_functional_writes += m_routers[i]->functionalWrite(pkt);
    }

    for (unsigned int i = 0; i < m_nis.size(); ++i) {
        num_functional_writes += m_nis[i]->functionalWrite(pkt);
    }

    for (unsigned int i = 0; i < m_networklinks.size(); ++i) {
        num_functional_writes += m_networklinks[i]->functionalWrite(pkt);
    }

    return num_functional_writes;
}

//UGAL
int GarnetNetwork::get_num_hops(int source_router, int vnet, NetDest net_dest)
{
  return m_routers[source_router]->get_num_hops(vnet, net_dest);
}

//spin scheme : rotating priority

bool GarnetNetwork::is_my_priority_greater(int my_router_id, int source_router_id)
{
  //the number of routers should be a power of 2
  int num_routers = getNumRouters();

  //std::cout<<"my router id is "<<my_router_id<<", source router-id is "<<source_router_id
  //	   <<std::endl<<std::flush;

  assert( (my_router_id < num_routers) && (source_router_id < num_routers) );

  assert( ((num_routers) & (num_routers-1)) == 0);

  unsigned thresh = dd_thresh;

  //deadlock detection threshold should be a power of 2
  assert( ((thresh) & (thresh-1)) == 0);

  int num_bits = log2(4*thresh);
  
  long long unsigned cur_cycle = m_routers[0]->curCycle();

  //std::cout<<"cur cycle is "<<cur_cycle<<std::endl<<std::flush;

  cur_cycle = cur_cycle >> num_bits;

  //std::cout<<"cur cycle after right shift is "<<cur_cycle<<std::endl<<std::flush;

  num_bits = log2(num_routers);

  int temp = 1;
  temp = temp << num_bits;
  temp--;

  //std::cout<<"temp is "<<temp<<std::endl<<std::flush;

  cur_cycle = cur_cycle & temp;

  //std::cout<<"cur cycle after bit-wise AND is "<<cur_cycle<<std::endl<<std::flush;
 
  int cur_top_priority_router = cur_cycle;

  int my_router_priority = get_router_priority(cur_top_priority_router, my_router_id); 
  int source_router_priority = get_router_priority(cur_top_priority_router, source_router_id);

  return (my_router_priority > source_router_priority);
}

int GarnetNetwork::get_router_priority(int cur_top_priority_router, int my_router_id)
{
  int my_router_priority, num_routers = getNumRouters();

  if(my_router_id == cur_top_priority_router)
    {
      my_router_priority = num_routers - 1;
    }
  else if(my_router_id > cur_top_priority_router)
    {
      my_router_priority = my_router_id - cur_top_priority_router - 1;
    }
  else
    {
      my_router_priority = num_routers - (cur_top_priority_router - my_router_id) -1;
    }

  //std::cout<<"router priority is "<<my_router_priority<<", cur top priority router "
  //	   <<cur_top_priority_router<<", my router-id is "<<my_router_id<<std::endl<<std::flush;

  assert( (my_router_priority >= 0) && (my_router_priority < num_routers) );

  return my_router_priority;
}

