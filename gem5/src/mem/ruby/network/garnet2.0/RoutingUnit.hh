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


#ifndef __MEM_RUBY_NETWORK_GARNET_ROUTING_UNIT_HH__
#define __MEM_RUBY_NETWORK_GARNET_ROUTING_UNIT_HH__

#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/garnet2.0/CommonTypes.hh"
#include "mem/ruby/network/garnet2.0/GarnetNetwork.hh"
#include "mem/ruby/network/garnet2.0/flit.hh"

class InputUnit;
class Router;

class RoutingUnit
{
  public:
    RoutingUnit(Router *router);
    int outportCompute(RouteInfo *route,
                      int inport,
		       PortDirection inport_dirn, int invc);

    // Topology-agnostic Routing Table based routing (default)
    void addRoute(const NetDest& routing_table_entry);
    void addWeight(int link_weight);

    // get output port from routing table
    int  lookupRoutingTable(int vnet, NetDest net_dest);

    // Topology-specific direction based routing
    void addInDirection(PortDirection inport_dirn, int inport);
    void addOutDirection(PortDirection outport_dirn, int outport);

    // Routing for Mesh
    int outportComputeXY(RouteInfo route,
                         int inport,
                         PortDirection inport_dirn);

    int outportComputeWestFirst(RouteInfo *route,
                         int inport,
				PortDirection inport_dirn, int invc);

    int outportComputeRandom(RouteInfo route,
                         int inport,
                         PortDirection inport_dirn);

  // UGAL
  int outportComputeUgal(RouteInfo *route,
                             int inport,
                             PortDirection inport_dirn);
  int get_num_hops(int vnet, NetDest net_dest);
  int dragon_fly_multi_vc_ugal(RouteInfo *route, int inport, PortDirection inport_dirn);
  void change_route_non_minimal_dragonfly(RouteInfo *route, int intermediate_group, 
					  int intermediate_node, NetDest tdest);
  void change_route_minimal_dragonfly(RouteInfo *route);
  int dragon_fly_one_vc_ugal(RouteInfo *route, int inport, PortDirection inport_dirn);
  int Mesh_one_vc_adaptive(RouteInfo *route, int inport, PortDirection inport_dirn);
  int ugal_general_multi_vc(RouteInfo *route, int inport, PortDirection inport_dirn);
  int ugal_general_one_vc(RouteInfo *route, int inport, PortDirection inport_dirn);

  
  int route_minimal_adaptive(int vnet, NetDest msg_destination);
  int get_min_active_vc_outport(output_links *valiant_links, int vnet);
  output_links* get_link_candidates(int vnet, NetDest msg_destination);

  //one-vc scheme
  int outportCompute_one_vc_Adaptive(RouteInfo *route, int inport, PortDirection inport_dirn);
  int get_min_active_outport(output_links *minimal_links, int vnet);
  std::vector<int> get_free_vc_outports(output_links *links, int vnet);

  //escape-vc
  int WestFirstRouting(RouteInfo *route, int inport, PortDirection inport_dirn);

  private:
    Router *m_router;

    // Routing Table
    std::vector<NetDest> m_routing_table;
    std::vector<int> m_weight_table;

    // Inport and Outport direction to idx maps
    std::map<PortDirection, int> m_inports_dirn2idx;
    std::map<int, PortDirection> m_inports_idx2dirn;
    std::map<int, PortDirection> m_outports_idx2dirn;
    std::map<PortDirection, int> m_outports_dirn2idx;
};

#endif // __MEM_RUBY_NETWORK_GARNET_ROUTING_UNIT_HH__
