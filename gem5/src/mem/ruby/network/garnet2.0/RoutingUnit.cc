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

#include <iostream>

#include "mem/ruby/network/garnet2.0/RoutingUnit.hh"

#include "base/cast.hh"
#include "mem/ruby/network/garnet2.0/InputUnit.hh"
#include "mem/ruby/network/garnet2.0/Router.hh"
#include "mem/ruby/slicc_interface/Message.hh"
#include "mem/ruby/network/garnet2.0/GarnetNetwork.hh"

RoutingUnit::RoutingUnit(Router *router)
{
    m_router = router;
    m_routing_table.clear();
    m_weight_table.clear();
}

void
RoutingUnit::addRoute(const NetDest& routing_table_entry)
{
    m_routing_table.push_back(routing_table_entry);
}

void
RoutingUnit::addWeight(int link_weight)
{
    m_weight_table.push_back(link_weight);
}

/*
 * This is the default routing algorithm in garnet.
 * The routing table is populated during topology creation.
 * Routes can be biased via weight assignments in the topology file.
 * Correct weight assignments are critical to provide deadlock avoidance.
 */

int
RoutingUnit::lookupRoutingTable(int vnet, NetDest msg_destination)
{
    // First find all possible output link candidates
    // For ordered vnet, just choose the first
    // (to make sure different packets don't choose different routes)
    // For unordered vnet, randomly choose any of the links
    // To have a strict ordering between links, they should be given
    // different weights in the topology file

    int output_link = -1;
    int min_weight = INFINITE_;
    std::vector<int> output_link_candidates;
    int num_candidates = 0;

    // Identify the minimum weight among the candidate output links
    for (int link = 0; link < m_routing_table.size(); link++) {
        if (msg_destination.intersectionIsNotEmpty(m_routing_table[link])) {

        if (m_weight_table[link] <= min_weight)
            min_weight = m_weight_table[link];
        }
    }

    // Collect all candidate output links with this minimum weight
    for (int link = 0; link < m_routing_table.size(); link++) {
        if (msg_destination.intersectionIsNotEmpty(m_routing_table[link])) {

            if (m_weight_table[link] == min_weight) {

                num_candidates++;
                output_link_candidates.push_back(link);
            }
        }
    }

    if (output_link_candidates.size() == 0) {
        fatal("Fatal Error:: No Route exists from this Router.");
        exit(0);
    }

    // Randomly select any candidate output link
    int candidate = 0;
    if (!(m_router->get_net_ptr())->isVNetOrdered(vnet))
        candidate = rand() % num_candidates;

    output_link = output_link_candidates.at(candidate);
    return output_link;
}


void
RoutingUnit::addInDirection(PortDirection inport_dirn, int inport_idx)
{
    m_inports_dirn2idx[inport_dirn] = inport_idx;
    m_inports_idx2dirn[inport_idx]  = inport_dirn;
}

void
RoutingUnit::addOutDirection(PortDirection outport_dirn, int outport_idx)
{
    m_outports_dirn2idx[outport_dirn] = outport_idx;
    m_outports_idx2dirn[outport_idx]  = outport_dirn;
}

// outportCompute() is called by the InputUnit
// It calls the routing table by default.
// A template for adaptive topology-specific routing algorithm
// implementations using port directions rather than a static routing
// table is provided here.

int
RoutingUnit::outportCompute(RouteInfo *route, int inport,
                            PortDirection inport_dirn, int invc)
{
    int outport = -1;

    // Routing Algorithm set in GarnetNetwork.py
    // Can be over-ridden from command line using --routing-algorithm = 1
    RoutingAlgorithm routing_algorithm =
      (RoutingAlgorithm) m_router->get_net_ptr()->getRoutingAlgorithm();


    if(routing_algorithm != UGAL_)
      {
	if (route->dest_router == m_router->get_id()) 
	  {
	    // Multiple NIs may be connected to this router,
	    // all with output port direction = "Local"
	    // Get exact outport id from table

	    //std::cout<<" new tdest is "<<route.net_dest<<std::endl<<std::flush;
	    outport = lookupRoutingTable(route->vnet, route->net_dest);
	    return outport;
	  }
      }


    //std::cout<<"routing algo is "<<routing_algorithm<<std::endl<<std::flush;

    switch (routing_algorithm) 
      {
      case TABLE_:  outport =
	  lookupRoutingTable(route->vnet, route->net_dest);
	break;
      case XY_:     outport =
	  outportComputeXY(*route, inport, inport_dirn); break;
      case WEST_FIRST_: outport =
	  outportComputeWestFirst(route, inport, inport_dirn, invc); break;
      case RANDOM_: outport =
	  outportComputeRandom(*route, inport, inport_dirn); break;
        // any custom algorithm
      case UGAL_: outport =
	  outportComputeUgal(route, inport, inport_dirn); break;
      default: outport =
	  lookupRoutingTable(route->vnet, route->net_dest); break;
      }

    assert(outport != -1);
    return outport;
}

// XY routing implemented using port directions
// Only for reference purpose in a Mesh
// By default Garnet uses the routing table
int
RoutingUnit::outportComputeXY(RouteInfo route,
                              int inport,
                              PortDirection inport_dirn)
{
    PortDirection outport_dirn = "Unknown";

    int M5_VAR_USED num_rows = m_router->get_net_ptr()->getNumRows();
    int num_cols = m_router->get_net_ptr()->getNumCols();
    assert(num_rows > 0 && num_cols > 0);

    int my_id = m_router->get_id();
    int my_x = my_id % num_cols;
    int my_y = my_id / num_cols;

    int dest_id = route.dest_router;
    int dest_x = dest_id % num_cols;
    int dest_y = dest_id / num_cols;

    int x_hops = abs(dest_x - my_x);
    int y_hops = abs(dest_y - my_y);

    bool x_dirn = (dest_x >= my_x);
    bool y_dirn = (dest_y >= my_y);

    // already checked that in outportCompute() function
    assert(!(x_hops == 0 && y_hops == 0));

    if (x_hops > 0) {
        if (x_dirn) {
            assert(inport_dirn == "Local" || inport_dirn == "West");
            outport_dirn = "East";
        } else {
            assert(inport_dirn == "Local" || inport_dirn == "East");
            outport_dirn = "West";
        }
    } else if (y_hops > 0) {
        if (y_dirn) {
            // "Local" or "South" or "West" or "East"
            assert(inport_dirn != "North");
            outport_dirn = "North";
        } else {
            // "Local" or "North" or "West" or "East"
            assert(inport_dirn != "South");
            outport_dirn = "South";
        }
    } else {
        // x_hops == 0 and y_hops == 0
        // this is not possible
        // already checked that in outportCompute() function
        assert(0);
    }

    return m_outports_dirn2idx[outport_dirn];
}

int RoutingUnit::outportComputeWestFirst(RouteInfo *route, int inport, PortDirection inport_dirn, 
					 int invc)
{
  int vnet = route->vnet;
  int escape_vc = ( (vnet+1) * m_router->get_vc_per_vnet() ) -1;

  // if escape-vc scheme is enabled and packet is in escape-vc
  // it should stay in the escape-vc. Else route minimal adaptively
  // in the remaining vcs.
    
  if(m_router->get_vc_per_vnet() == 1)
    return WestFirstRouting(route, inport, inport_dirn);

  if(m_router->get_net_ptr()->is_escape_vc_enabled())
    {
      if(invc == escape_vc)
	return WestFirstRouting(route, inport, inport_dirn);
      else
	return route_minimal_adaptive(vnet, route->net_dest);
    }
  else
    {
      // escape-vc is not enabled
      // do west first routing in all vcs
      
      return WestFirstRouting(route, inport, inport_dirn);
    }
}

int RoutingUnit::WestFirstRouting(RouteInfo *route, int inport, PortDirection inport_dirn)
{
  int num_rows = m_router->get_net_ptr()->getNumRows();
  int num_cols = m_router->get_net_ptr()->getNumCols();
  assert(num_rows > 0 && num_cols > 0);

  // std::cout<<"num_rows "<<num_rows<<" num_cols "<<num_cols<<std::flush;

  int my_id = m_router->get_id();
  int my_x = my_id % num_cols;
  int my_y = my_id / num_cols;

  int dest_id = route->dest_router;
  int dest_x = dest_id % num_cols;
  int dest_y = dest_id / num_cols;

  int x_hops = abs(dest_x - my_x);
  int y_hops = abs(dest_y - my_y);

  // already checked that in outportCompute() function
  assert(!(x_hops == 0 && y_hops == 0));

  PortDirection west_port_dirn = "West";
  int west_outport = m_outports_dirn2idx[west_port_dirn];

  //need to route West first
  if(dest_x < my_x)
    {
      //assert(inport_dirn == "East" || inport_dirn == "Local");
      return west_outport;
    }

  if(m_router->get_vc_per_vnet() == 1)
    {
      return Mesh_one_vc_adaptive(route, inport, inport_dirn);
    }
  else
    {
      int outport = route_minimal_adaptive(route->vnet, route->net_dest);

      if(dest_x == my_x)
	{
	  if(dest_y > my_y)
	    {
	      assert(m_outports_idx2dirn[outport] == "North");
	    }
	  else
	    {
	      assert(m_outports_idx2dirn[outport] == "South");
	    }
	}
      return outport;
    }
}

int
RoutingUnit::outportComputeRandom(RouteInfo route,
                              int inport,
                              PortDirection inport_dirn)
{
    PortDirection outport_dirn = "Unknown";

    int M5_VAR_USED num_rows = m_router->get_net_ptr()->getNumRows();
    int num_cols = m_router->get_net_ptr()->getNumCols();
    assert(num_rows > 0 && num_cols > 0);

    int my_id = m_router->get_id();
    int my_x = my_id % num_cols;
    int my_y = my_id / num_cols;

    int dest_id = route.dest_router;
    int dest_x = dest_id % num_cols;
    int dest_y = dest_id / num_cols;

    int x_hops = abs(dest_x - my_x);
    int y_hops = abs(dest_y - my_y);

    bool x_dirn = (dest_x >= my_x);
    bool y_dirn = (dest_y >= my_y);

    // already checked that in outportCompute() function
    assert(!(x_hops == 0 && y_hops == 0));

    if (x_hops == 0)
    {
        if (y_dirn > 0)
            outport_dirn = "North";
        else
            outport_dirn = "South";
    }
    else if (y_hops == 0)
    {
        if (x_dirn > 0)
            outport_dirn = "East";
        else
            outport_dirn = "West";
    } else {
        int rand = random() % 2;

        if (x_dirn && y_dirn) // Quadrant I
            outport_dirn = rand ? "East" : "North";
        else if (!x_dirn && y_dirn) // Quadrant II
            outport_dirn = rand ? "West" : "North";
        else if (!x_dirn && !y_dirn) // Quadrant III
            outport_dirn = rand ? "West" : "South";
        else // Quadrant IV
            outport_dirn = rand ? "East" : "South";
    }

    return m_outports_dirn2idx[outport_dirn];
}

// implementation of UGAL adaptive algorithm

int RoutingUnit::outportComputeUgal(RouteInfo *route, int inport, PortDirection inport_dirn)
{
  if(m_router->get_net_ptr()->get_topology() == "dragonfly")
    {
      if(m_router->get_vc_per_vnet() == 1)
	{
	  return dragon_fly_one_vc_ugal(route, inport, inport_dirn);
	}
      else
	{
	  return dragon_fly_multi_vc_ugal(route, inport, inport_dirn);
	}
    }
  else if(m_router->get_net_ptr()->get_topology() == "Mesh")
    {
      if(m_router->get_vc_per_vnet() == 1)
	{
	  return Mesh_one_vc_adaptive(route, inport, inport_dirn);
	}
      else
	{
	  return route_minimal_adaptive(route->vnet, route->net_dest);
	}
    }
  else
    {
      if(m_router->get_vc_per_vnet() == 1)
	{
	  return ugal_general_one_vc(route, inport, inport_dirn);
	}
      else
	{
	  return ugal_general_multi_vc(route, inport, inport_dirn);
	}
    }  
}


output_links* RoutingUnit::get_link_candidates(int vnet, NetDest msg_destination)
{
  output_links *outlinks = new output_links;
  outlinks->hop_count = INFINITE_;
  outlinks->output_link_candidates.clear();

  
  // Identify the minimum weight among the candidate output links
  for (int link = 0; link < m_routing_table.size(); link++) 
    {
      if (msg_destination.intersectionIsNotEmpty(m_routing_table[link])) 
	{
	  if (m_weight_table[link] <= outlinks->hop_count)
            outlinks->hop_count = m_weight_table[link];
        }
    }

  // Collect all candidate output links with this minimum weight
  for (int link = 0; link < m_routing_table.size(); link++) 
    {
      if (msg_destination.intersectionIsNotEmpty(m_routing_table[link])) 
	{
	  if (m_weight_table[link] == outlinks->hop_count) 
	    {
	      outlinks->output_link_candidates.push_back(link);
            }
        }
    }

  if( outlinks->output_link_candidates.size() == 0) 
    {
      fatal("Fatal Error:: No Route exists from this Router.");
      exit(0);
    }

  return outlinks;
}

int RoutingUnit::get_min_active_vc_outport(output_links *valiant_links, int vnet)
{
  //loop through all outport candidates to find the one 
  //with minimum vc_occupied
  int min_active_vcs = INFINITE_;
  int winner_outport = -1;
  
  for(int i=0; i<valiant_links->output_link_candidates.size(); i++)
    {
      int outport = valiant_links->output_link_candidates[i];

      //      assert(m_outports_idx2dirn[outport] != "West");

      int vcs_active = m_router->get_num_active_vcs(outport, vnet);
      
      if(vcs_active < min_active_vcs)
	{
	  min_active_vcs = vcs_active;
	  winner_outport = outport;
	}
    }

  assert(winner_outport != -1);

  return winner_outport;
}

int RoutingUnit::route_minimal_adaptive(int vnet, NetDest msg_destination)
{
  //get minimal output link candidates
  output_links *minimal_links = get_link_candidates(vnet, msg_destination);
  int minimal_outport = get_min_active_vc_outport(minimal_links, vnet);

  if(m_router->get_net_ptr()->get_topology() == "dragonfly")
    assert(minimal_links->output_link_candidates.size() == 1);

  delete minimal_links;
  
  return minimal_outport;
}


int RoutingUnit::get_num_hops(int vnet, NetDest msg_destination)
{
  //return the minimum no of hops from this source to a dest 

  int min_weight = INFINITE_;

  // Identify the minimum weight among the candidate output links
  for (int link = 0; link < m_routing_table.size(); link++) 
    {
      if (msg_destination.intersectionIsNotEmpty(m_routing_table[link])) 
	{
	  if (m_weight_table[link] <= min_weight)
            min_weight = m_weight_table[link];
        }
    }

  return min_weight;
}

int RoutingUnit::Mesh_one_vc_adaptive(RouteInfo *route, int inport, PortDirection inport_dirn)
{
  //get minimal output link candidates
  output_links *minimal_links = get_link_candidates(route->vnet, route->net_dest);
  
  //find all outports that have a free outvc
  std::vector<int> free_vc_minimal_outports = get_free_vc_outports(minimal_links, route->vnet);
  
  if(free_vc_minimal_outports.size() == 0)
    {
      //none of the minimal outports are free
      //find the minimal outport that has been ACTIVE for the least number of cycles
      //and return this outport
      
      int outport = get_min_active_outport(minimal_links, route->vnet);
      delete minimal_links;
      return outport;
    }
  else
    {
      //randomly choose any
      delete minimal_links;
      int index = rand() % free_vc_minimal_outports.size();
      return (free_vc_minimal_outports[index]);
    }
}

std::vector<int> RoutingUnit::get_free_vc_outports(output_links *links, int vnet)
{
  std::vector<int> outports;

  for(int i=0; i<links->output_link_candidates.size(); i++)
    {
      if(m_router->has_free_vc(links->output_link_candidates[i], vnet))
	outports.push_back(links->output_link_candidates[i]);
    }

  return outports;
}

int RoutingUnit::get_min_active_outport(output_links *minimal_links, int vnet)
{
  //returns the ouput port whose outvc has been active for the least
  //number of cycles. Works for only 1-vc-per-vnet

  //only the first vc of every vnet is checked. make sure that there is only
  //1-vc-per-vnet
  assert(m_router->get_vc_per_vnet() == 1);

  Cycles min_active_time = m_router->curCycle();
  int outport = -1;

  for(int i=0; i<minimal_links->output_link_candidates.size(); i++)
    {
      Cycles active_time = m_router->
	get_vc_active_time(minimal_links->output_link_candidates[i], vnet);

      if(active_time < min_active_time)
	{
	  min_active_time = active_time;
	  outport = minimal_links->output_link_candidates[i];
	}
    }

  assert(outport != -1);
  return outport;
}

int RoutingUnit::dragon_fly_multi_vc_ugal(RouteInfo *route, int inport, PortDirection inport_dirn)
{
  int cur_group = m_router->get_id() / m_router->get_net_ptr()->get_dfly_group_size();

  if(inport_dirn == "Local")
    {
      //packt is at the source node
      //decide whether to route to an intermediate node
      //or straight to destination

      int source_group = m_router->get_id() / m_router->get_net_ptr()->get_dfly_group_size();
      int dest_group = route->dest_router / m_router->get_net_ptr()->get_dfly_group_size();
      
      //generate a random intermediate node
      int intermediate_node = rand() % (m_router->get_net_ptr()->getNumRouters());
      int intermediate_group = intermediate_node / m_router->get_net_ptr()->get_dfly_group_size();

      //choice between valiant and minimal only makes sense if intermediate group
      //is different than source and destination groups

      if( (source_group != intermediate_group) && (dest_group != intermediate_group) )
	{
	  //create a net dest for the intermediate node
	  NetDest tdest;
	  tdest.clear();
	  NodeID destID = intermediate_node;
	  int m = 0;
	  tdest.add((MachineID) {(MachineType) m, (destID - MachineType_base_number((MachineType) m))});

	  //get valiant output link candidates
	  output_links *valiant_links = get_link_candidates(route->vnet, tdest);
	  
	  //there can be only 1 minimal path in dragon-fly
	  assert(valiant_links->output_link_candidates.size() == 1);

	  int valiant_outport = valiant_links->output_link_candidates[0];
	  int phase1_hop_count = valiant_links->hop_count;
	  int phase2_hop_count = 
	    m_router->get_net_ptr()->get_num_hops(intermediate_node, route->vnet, route->net_dest);
	  int valiant_hop_count = phase1_hop_count + phase2_hop_count;

	  //get minimal output link candidates
	  output_links *minimal_links = get_link_candidates(route->vnet, route->net_dest);

	  //there can be only 1 minimal path in dragon-fly
	  assert(minimal_links->output_link_candidates.size() == 1);

	  int minimal_outport = minimal_links->output_link_candidates[0];
	  

	  // choose minimal if 
	  // (hop_count_minimal * minimal_vcs_active) < (hop_count_valiant * valiant_vcs_active)

	  int minimal_product = m_router->get_num_active_vcs(minimal_outport, route->vnet) * 
	    minimal_links->hop_count;
	  int valiant_product = m_router->get_num_active_vcs(valiant_outport, route->vnet) * 
	    valiant_hop_count;


	  delete minimal_links;
	  delete valiant_links;

	  if(minimal_product <= valiant_product)
	    {
	      //choose minimal routing
	      change_route_minimal_dragonfly(route);
	      return minimal_outport;
	    }
	  else
	    {
	      //choose valiant
	      change_route_non_minimal_dragonfly(route, intermediate_group, intermediate_node, tdest);
	      return valiant_outport;
	    }
	}
      else
	{
	  //route minimally adaptive  
	  change_route_minimal_dragonfly(route);
	  return route_minimal_adaptive(route->vnet, route->net_dest);
	}
    }
  else if(route->intermediate_group_id == cur_group)
    {
      //reached intermediate group
      //route to the correct dest
      
      route->net_dest = route->final_net_dest;
      route->intermediate_group_id = -1;
      //route minimally adaptive
      return route_minimal_adaptive(route->vnet, route->net_dest);
    }
  else
    {
      //route minimally adaptive
      return route_minimal_adaptive(route->vnet, route->net_dest);
    }

}


int RoutingUnit::ugal_general_multi_vc(RouteInfo *route, int inport, PortDirection inport_dirn)
{
  if(inport_dirn == "Local")
    {
      //packt is at the source node
      //decide whether to route to an intermediate node
      //or straight to destination

      //generate a random intermediate node
      int intermediate_node = rand() % (m_router->get_net_ptr()->getNumRouters());

      //choice between valiant and minimal only makes sense if intermediate node
      //is different than source and destination node

      if((intermediate_node != m_router->get_id()) && (intermediate_node != route->dest_router))
	{
	  //create a net dest for the intermediate node
	  NetDest tdest;
	  tdest.clear();
	  NodeID destID = intermediate_node;
	  int m = 0;
	  tdest.add((MachineID) {(MachineType) m, (destID - MachineType_base_number((MachineType) m))});

	  //get valiant output link candidates
	  output_links *valiant_links = get_link_candidates(route->vnet, tdest);
	  int valiant_outport = get_min_active_vc_outport(valiant_links, route->vnet);
	  int phase1_hop_count = valiant_links->hop_count;
	  int phase2_hop_count = 
	    m_router->get_net_ptr()->get_num_hops(intermediate_node, route->vnet, route->net_dest);
	  int valiant_hop_count = phase1_hop_count + phase2_hop_count;

	  //get minimal output link candidates
	  output_links *minimal_links = get_link_candidates(route->vnet, route->net_dest);
	  int minimal_outport = get_min_active_vc_outport(minimal_links, route->vnet);
	  

	  // choose minimal if 
	  // (hop_count_minimal * minimal_vcs_active) < (hop_count_valiant * valiant_vcs_active)

	  int minimal_product = m_router->get_num_active_vcs(minimal_outport, route->vnet) * 
	    minimal_links->hop_count;
	  int valiant_product = m_router->get_num_active_vcs(valiant_outport, route->vnet) * 
	    valiant_hop_count;


	  delete minimal_links;
	  delete valiant_links;

	  if(minimal_product <= valiant_product)
	    {
	      //choose minimal routing
	      route->intermediate_node = m_router->get_id();
	      route->final_net_dest = route->net_dest;
	      return minimal_outport;
	    }
	  else
	    {
	      //choose valiant
	      route->intermediate_node = intermediate_node;
	      route->final_net_dest = route->net_dest;
	      route->net_dest = tdest;
	      return valiant_outport;
	    }
	}
      else
	{
	  //route minimally adaptive  
	  route->intermediate_node = m_router->get_id();
	  route->final_net_dest = route->net_dest;
	  return route_minimal_adaptive(route->vnet, route->net_dest);
	}
    }
  else if(route->intermediate_node == m_router->get_id())
    {
      //reached intermediate node
      //route to the correct dest
      
      route->net_dest = route->final_net_dest;
      
      //route minimally adaptive
      return route_minimal_adaptive(route->vnet, route->net_dest);
    }
  else
    {
      //route minimally adaptive
      return route_minimal_adaptive(route->vnet, route->net_dest);
    }
}

int RoutingUnit::ugal_general_one_vc(RouteInfo *route, int inport, PortDirection inport_dirn)
{
  if(inport_dirn == "Local")
    {
      //packet at source router

      //get minimal links
      output_links *minimal_links = get_link_candidates(route->vnet, route->net_dest);

      //find all outports that have a free outvc
      std::vector<int> free_vc_minimal_outports = get_free_vc_outports(minimal_links, route->vnet);

      //if there are more than one free minimal outports, 
      //randomly choose any.

      if(free_vc_minimal_outports.size() > 0)
	{
	  //randomly choose any
	  delete minimal_links;
	  int index = rand() % free_vc_minimal_outports.size();
	  return (free_vc_minimal_outports[index]);
	}
      else
	{
	  /* No minimal outports are free. Generate a random intermediate 
	   * node to route to. Choice between Minimal and non-minimal is 
	   * made using 
	   * Min(H_minimal + t_contention_minimal, H_non-minimal + t_contention_non-minimal)
	   * t_contention is relaxed by the buffer turnaround time.
	   *
	   * For 3-d flattened butterfly topology, the buffer turn-around time is taken
	   * as 1-cycle.
	   */

	  int intermediate_node = rand() % (m_router->get_net_ptr()->getNumRouters());
	  
	  int minimal_outport = get_min_active_outport(minimal_links, route->vnet);

	  if( (intermediate_node != route->dest_router) && (intermediate_node != m_router->get_id()) )
	    {
	      NetDest tdest;
	      tdest.clear();
	      NodeID destID = intermediate_node;
	      int m = 0;
	      tdest.add((MachineID) {(MachineType) m, 
		    (destID - MachineType_base_number((MachineType) m))});
	      
	      output_links *valiant_links = get_link_candidates(route->vnet, tdest);

	      /* Among the valiant/minimal outports, choose the one that has been
	       * active for the least number of cycles.
	       */

	      int valiant_outport = get_min_active_outport(valiant_links, route->vnet);
	      

	      int minimal_hops = minimal_links->hop_count;

	      int phase1_hops = valiant_links->hop_count;
	      int phase2_hops = m_router->get_net_ptr()->
		get_num_hops(intermediate_node, route->vnet, route->net_dest);

	      int valiant_hops = phase1_hops + phase2_hops;

	      int minimal_sum = minimal_hops + m_router->
		get_vc_active_time(minimal_outport, route->vnet);

	      int valiant_sum = valiant_hops + m_router->
		get_vc_active_time(valiant_outport, route->vnet);

	      delete valiant_links;
	      delete minimal_links;

	      if(minimal_sum <= valiant_sum)
		{
		  //route minimally
		  
		  route->intermediate_node = m_router->get_id();
		  route->final_net_dest = route->net_dest;

		  return minimal_outport;
		}
	      else
		{
		  //route valiant

		  route->intermediate_node = m_router->get_id();
		  route->final_net_dest = route->net_dest;
		  route->net_dest = tdest;

		  return valiant_outport;
		}
	    }
	  else
	    {
	      //route minimally
	      
	      route->intermediate_node = m_router->get_id();
	      route->final_net_dest = route->net_dest;

	      delete minimal_links;

	      return minimal_outport;
	    }
	}
    }
  else if(route->intermediate_node == m_router->get_id())
    {
      route->intermediate_node = -1;
      route->net_dest = route->final_net_dest;
    }

  return Mesh_one_vc_adaptive(route, inport, inport_dirn);
}

int RoutingUnit::dragon_fly_one_vc_ugal(RouteInfo *route, int inport, PortDirection inport_dirn)
{
  int cur_group = m_router->get_id() / m_router->get_net_ptr()->get_dfly_group_size();
  
  if(inport_dirn == "Local")
    {
      //packet at source router.

      //prefer minimal routing : If vc on minimal route is empty,
      //use it.
      output_links *minimal_links = get_link_candidates(route->vnet, route->net_dest);

      //there can be only one minimal path between a (source, destination) pair
      //in dragon-fly.
      assert(minimal_links->output_link_candidates.size() == 1);
      
      if(m_router->has_free_vc(minimal_links->output_link_candidates[0], route->vnet))
	{
	  //route minimal
	  int outport = minimal_links->output_link_candidates[0];
	  change_route_minimal_dragonfly(route);
	  
	  delete minimal_links;
	  return outport;
	}

      //minimal path has no free-vc
	  
      int source_group = m_router->get_id() / m_router->get_net_ptr()->get_dfly_group_size();
      int dest_group = route->dest_router / m_router->get_net_ptr()->get_dfly_group_size();

      if(source_group == dest_group)
	{
	  /* if routing within the group, choose the intermediate node randomly as some node
	   * within the group. choice between minimal and 
	   * non-minimal is made using 
	   * Min(H_minimal + t_contention_minimal, H_non-minimal + t_contention_non-minimal)
	   * t_contention is relaxed by the buffer turnaround time.
	   */

	  assert(minimal_links->hop_count == 1);

	  /* Minimal path has no free-vc
	   * Get a radom node within the group to route to 
	   */

	  int offset = rand() % m_router->get_net_ptr()->get_dfly_group_size();
	  int intermediate_node = 
	    (source_group * m_router->get_net_ptr()->get_dfly_group_size()) + offset;
	  
	  if( (intermediate_node == route->dest_router) || (intermediate_node == m_router->get_id()))
	    {
	      //route minimally
	      int outport = minimal_links->output_link_candidates[0];
	      change_route_minimal_dragonfly(route);
	      
	      delete minimal_links;
	      return outport;
	    }
	  else
	    {
	      /* choice between minimal and non-minimal is based on
	       * Min(H_minimal + t_contention_minimal, H_non-minimal + t_contention_non-minimal)
	       * H_minimal =1 and H_non-minimal = 2
	       * t_contention_non-minimal =0 if outvc is free
	       * subtract buffer turnaround time from t_contention_minimal
	       */
	      
	      //create a net dest for the intermediate node
	      NetDest tdest;
	      tdest.clear();
	      NodeID destID = intermediate_node;
	      int m = 0;
	      tdest.add((MachineID) {(MachineType) m, 
		    (destID - MachineType_base_number((MachineType) m))});
	      
	      output_links *non_minimal_links = get_link_candidates(route->vnet, tdest);
	      
	      assert(non_minimal_links->output_link_candidates.size() == 1);
	      assert(non_minimal_links->hop_count == 1);
	      
	      int t_contention_non_minimal = 0;
	      
	      if(!(m_router->has_free_vc(non_minimal_links->output_link_candidates[0], 
					 route->vnet)))
		{
		  Cycles act_time = m_router->
		    get_vc_active_time(non_minimal_links->output_link_candidates[0], route->vnet);
		  
		  t_contention_non_minimal = static_cast<int>(act_time) - BUFFER_TURNAROUND_INTRA;
		      
		  if(t_contention_non_minimal < 0)
		    t_contention_non_minimal = 0;
		}

	      Cycles act_time = m_router->
		get_vc_active_time(minimal_links->output_link_candidates[0], route->vnet);
		      
	      int t_contention_minimal = static_cast<int>(act_time) - BUFFER_TURNAROUND_INTRA;
	      
	      if(t_contention_minimal < 0)
		t_contention_minimal = 0;
	      
	      int minimal_sum = 1 + t_contention_minimal;
	      int non_minimal_sum = 2 + t_contention_non_minimal;
	      
	      int outport;

	      if(minimal_sum <= non_minimal_sum)
		{
		  //use minimal
		  outport = minimal_links->output_link_candidates[0];
		  change_route_minimal_dragonfly(route);
		}
	      else
		{
		  //use non-minimal
		  outport = non_minimal_links->output_link_candidates[0];
		  change_route_non_minimal_dragonfly(route, -1, intermediate_node, tdest);
		}
	      
	      delete minimal_links;
	      delete non_minimal_links;
	      return outport;
	    }
	}
      else
	{
	  //route to a random group and then to destination
	  //choice between minimal and non-minimal is made in the same way


	  //generate a random intermediate node
	  int intermediate_node = rand() % (m_router->get_net_ptr()->getNumRouters());
	  int intermediate_group = intermediate_node / m_router->get_net_ptr()->get_dfly_group_size();

	  //choice between valiant and minimal only makes sense if intermediate group
	  //is different than source and destination groups

	  if( (source_group != intermediate_group) && (dest_group != intermediate_group) )
	    {
	      //create a net dest for the intermediate node
	      NetDest tdest;
	      tdest.clear();
	      NodeID destID = intermediate_node;
	      int m = 0;
	      tdest.add((MachineID) {(MachineType) m, 
		    (destID - MachineType_base_number((MachineType) m))});

	      //get valiant output link candidates
	      output_links *valiant_links = get_link_candidates(route->vnet, tdest);
	  
	      //there can be only 1 minimal path in dragon-fly
	      assert(valiant_links->output_link_candidates.size() == 1);

	      int valiant_outport = valiant_links->output_link_candidates[0];
	      int phase1_hop_count = valiant_links->hop_count;
	      int phase2_hop_count = 
		m_router->get_net_ptr()->get_num_hops(intermediate_node, route->vnet, route->net_dest);
	      int valiant_hop_count = phase1_hop_count + phase2_hop_count;

	      int minimal_outport = minimal_links->output_link_candidates[0];
	      int minimal_hop_count = minimal_links->hop_count;

	      int t_contention_valiant = 0;
	      int t_contention_minimal = 0;

	      if(!m_router->has_free_vc(valiant_outport, route->vnet))
		{
		  Cycles act_time = m_router->
		    get_vc_active_time(valiant_outport, route->vnet);

		  if(m_router->getOutportDirection(valiant_outport) == "Inter")
		    t_contention_valiant = static_cast<int>(act_time) - BUFFER_TURNAROUND_INTER;
		  else if(m_router->getOutportDirection(valiant_outport) == "Intra")
		    t_contention_valiant = static_cast<int>(act_time) - BUFFER_TURNAROUND_INTRA;

		  if(t_contention_valiant < 0)
		    t_contention_valiant = 0;
		}

	      Cycles act_time = m_router->
		get_vc_active_time(minimal_outport, route->vnet);

	      if(m_router->getOutportDirection(minimal_outport) == "Inter")
		t_contention_minimal = static_cast<int>(act_time) - BUFFER_TURNAROUND_INTER;
	      else if(m_router->getOutportDirection(minimal_outport) == "Intra")
		t_contention_minimal = static_cast<int>(act_time) - BUFFER_TURNAROUND_INTRA;

	      if(t_contention_minimal < 0)
		t_contention_minimal = 0;

	      int valiant_sum = t_contention_valiant + valiant_hop_count;
	      int minimal_sum = t_contention_minimal + minimal_hop_count;

	      int outport;

	      if(minimal_sum <= valiant_sum)
		{
		  //route minimally
		  outport = minimal_outport;
		  change_route_minimal_dragonfly(route);
		}
	      else
		{
		  //route non-minimally
		  outport = valiant_outport;
		  change_route_non_minimal_dragonfly(route, intermediate_group, -1, tdest);
		}

	      delete minimal_links;
	      delete valiant_links;
	      return outport;
	    }
	  else
	    {
	      //route minimally
	      int t_outport = minimal_links->output_link_candidates[0];
	      change_route_minimal_dragonfly(route);
	      delete minimal_links;
	      return t_outport;
	    }
	}
    }
  else if(route->intermediate_node == m_router->get_id())
    {
      //reached intermediate node. This will be when the source n destination r in the same 
      //group. route to the correct destination now.

      route->net_dest = route->final_net_dest;
      route->intermediate_group_id = -1;
      //route minimally adaptive
      return route_minimal_adaptive(route->vnet, route->net_dest);
    }
  else if(route->intermediate_group_id == cur_group)
    {
      //reached intermediate group. This will happen when source and destination are not
      //in the same group.

      route->net_dest = route->final_net_dest;
      route->intermediate_group_id = -1;
      //route minimally adaptive
      return route_minimal_adaptive(route->vnet, route->net_dest);
    }
  else
    {
      return route_minimal_adaptive(route->vnet, route->net_dest);
    }
}

void RoutingUnit::change_route_minimal_dragonfly(RouteInfo *route)
{
  route->intermediate_node = m_router->get_id();
  route->final_net_dest = route->net_dest;
  route->intermediate_group_id = -1;
}

void RoutingUnit::change_route_non_minimal_dragonfly(RouteInfo *route, int intermediate_group, 
						     int intermediate_node, NetDest tdest)
{
  route->intermediate_node = intermediate_node;
  route->final_net_dest = route->net_dest;
  route->net_dest = tdest;
  route->intermediate_group_id = intermediate_group;
}
