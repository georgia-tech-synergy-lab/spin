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


#include "mem/ruby/network/garnet2.0/OutputUnit.hh"

#include "base/stl_helpers.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/garnet2.0/Credit.hh"
#include "mem/ruby/network/garnet2.0/Router.hh"

using namespace std;
using m5::stl_helpers::deletePointers;

OutputUnit::OutputUnit(int id, PortDirection direction, Router *router)
    : Consumer(router)
{
    m_id = id;
    m_direction = direction;
    m_router = router;
    m_num_vcs = m_router->get_num_vcs();
    m_vc_per_vnet = m_router->get_vc_per_vnet();
    m_out_buffer = new flitBuffer();

    for (int i = 0; i < m_num_vcs; i++) {
        m_outvc_state.push_back(new OutVcState(i, m_router->get_net_ptr()));
    }
}

OutputUnit::~OutputUnit()
{
    delete m_out_buffer;
    deletePointers(m_outvc_state);
}

void
OutputUnit::decrement_credit(int out_vc)
{
    DPRINTF(RubyNetwork, "Router %d OutputUnit %d decrementing credit for "
            "outvc %d at time: %lld\n",
            m_router->get_id(), m_id, out_vc, m_router->curCycle());

    m_outvc_state[out_vc]->decrement_credit();
}

void
OutputUnit::increment_credit(int out_vc)
{
    DPRINTF(RubyNetwork, "Router %d OutputUnit %d incrementing credit for "
            "outvc %d at time: %lld\n",
            m_router->get_id(), m_id, out_vc, m_router->curCycle());

    m_outvc_state[out_vc]->increment_credit();
}

// Check if the output VC (i.e., input VC at next router)
// has free credits (i..e, buffer slots).
// This is tracked by OutVcState
bool
OutputUnit::has_credit(int out_vc)
{
  assert(m_outvc_state[out_vc]->isInState(ACTIVE_, m_router->curCycle()));
  return m_outvc_state[out_vc]->has_credit();
}

bool
OutputUnit::has_free_vc(int vnet)
{
    int vc_base = vnet*m_vc_per_vnet;
    for (int vc = vc_base; vc < vc_base + m_vc_per_vnet; vc++) {
        if (is_vc_idle(vc, m_router->curCycle()))
            return true;
    }

    return false;
}

// Check if the output port (i.e., input port at next router) has free VCs.
bool
OutputUnit::has_free_vc(int vnet, int invc,
         PortDirection inport_dirn, PortDirection outport_dirn, RouteInfo route)
{

  int vc_base = vnet*m_vc_per_vnet;
  int vc_limit = (vnet+1)*m_vc_per_vnet;


  if(m_router->get_net_ptr()->is_dfly_dlock_avoidance_enabled())
    {

      //dragon-fly deadlock avoidance scheme: change vc
      //every time you use a global channel.
      //If using local channel, vc should remain the same
      //VC number should be determined using hop-count
      //If outport is "Local", any VC is allowed
      //output ports: Inter, Intra, Local

      assert(m_router->get_net_ptr()->get_topology() == "dragonfly");
      assert( (m_vc_per_vnet == 3) || (m_vc_per_vnet == 2) );

      int vc;

      if(outport_dirn == "Inter")
	{
	  //using global link

	  if(route.hops_traversed == 0)
	    {
	      vc = vc_base + 1;
	    }
	  else
	    {
	      vc = invc + 1;

	      if(!(vc<vc_limit))
		std::cout<<"assertion failure at router "<<m_router->get_id()<<" for outvc "<<vc
			 <<" vnet "<<vnet<<" outport dirn: "<<outport_dirn<<" inport_dirn: "<<inport_dirn
			 <<" source-router:"<<route.src_router<<" dest-router:"<<route.dest_router
			 <<" in cycle "<<m_router->curCycle()<<std::endl<<std::flush;

	      assert(vc<vc_limit);
	    }
	  
	  return is_vc_idle(vc, m_router->curCycle());
	}
      else if(outport_dirn == "Intra")
	{
	  //using intra-group link
	  if(route.hops_traversed == 0)
	    {
	      vc = vc_base;
	    }
	  else
	    {
	      vc = invc;
	    }
	  
	  return is_vc_idle(vc, m_router->curCycle());
	}
    }
  else if(m_router->get_net_ptr()->is_fbfly_dlock_avoidance_enabled())
    {
      /* 3-d fbfly deadlock avoidance scheme : Change VC at every
       * hop. At Local inport (connected to the NIC) or Local
       * outport, can use any vc
       */

      assert(m_router->get_net_ptr()->get_topology() == "fbfly3d");
      assert(m_vc_per_vnet == 6);

      if(outport_dirn != "Local")
	{
	  int vc = vc_base + route.hops_traversed;
	  assert(vc<vc_limit);
	  
	  return is_vc_idle(vc, m_router->curCycle());
	}
    }
  else if(m_router->get_net_ptr()->is_escape_vc_enabled())
    {
      // escape-vc: if u have west-hops remaining, u cannot
      // enter the escape-vc

      assert(m_router->get_net_ptr()->get_topology() == "Mesh");

      int num_rows = m_router->get_net_ptr()->getNumRows();
      int num_cols = m_router->get_net_ptr()->getNumCols();
      assert(num_rows > 0 && num_cols > 0);

      int my_id = m_router->get_id();
      int my_x = my_id % num_cols;
      
      int dest_id = route.dest_router;
      int dest_x = dest_id % num_cols;
      
      int ii = 0;

      if(dest_x < my_x)
	ii = 1;  //have to go west
      
      for (int vc = vc_base; vc < vc_base + m_vc_per_vnet - ii; vc++) 
	{
	  if (is_vc_idle(vc, m_router->curCycle()))
            return true;
	}
      
      return false;
    }

    
    for (int vc = vc_base; vc < vc_base + m_vc_per_vnet; vc++) {
        if (is_vc_idle(vc, m_router->curCycle()))
            return true;
    }

    return false;
}

// Assign a free output VC to the winner of Switch Allocation
int
OutputUnit::select_free_vc(int vnet, int invc,
         PortDirection inport_dirn, PortDirection outport_dirn, RouteInfo route)
{
  //dragon-fly deadlock avoidance scheme: change vc
  //every time you use a global channel.
  //If using local channel, vc should remain the same
  //VC number should be determined using hop-count
  //If outport is "Local", any VC is allowed
  //output ports: Inter, Intra, Local

    int vc_base = vnet*m_vc_per_vnet;
    int vc_limit = (vnet+1)*m_vc_per_vnet;

    if(m_router->get_net_ptr()->is_dfly_dlock_avoidance_enabled())
      {
	assert(m_router->get_net_ptr()->get_topology() == "dragonfly");
	assert( (m_vc_per_vnet == 3) || (m_vc_per_vnet == 2) );
	
	int vc;
	
	if(outport_dirn == "Inter")
	  {
	    //using global link

	    if(route.hops_traversed == 0)
	      {
		vc = vc_base + 1;
	      }
	    else
	      {
		vc = invc + 1;
		assert(vc < vc_limit);
	      }

	    assert(is_vc_idle(vc, m_router->curCycle()));
	    m_outvc_state[vc]->setState(ACTIVE_, m_router->curCycle());	      
	    return vc;
	  }
	else if(outport_dirn == "Intra")
	  {
	    //using intra-group link
	    if(route.hops_traversed == 0)
	      {
		vc = vc_base;
	      }
	    else
	      {
		vc = invc;
	      }
	    
	    assert(is_vc_idle(vc, m_router->curCycle()));
	    m_outvc_state[vc]->setState(ACTIVE_, m_router->curCycle());
	    return vc;
	  }
      }
    else if(m_router->get_net_ptr()->is_fbfly_dlock_avoidance_enabled())
      {
	/* 3-d fbfly deadlock avoidance scheme : Change VC at every
	 * hop. At Local inport (connected to the NIC) or Local
	 * outport, can use any vc
	 */
	
	assert(m_router->get_net_ptr()->get_topology() == "fbfly3d");
	assert(m_vc_per_vnet == 6);
	
	if(outport_dirn != "Local")
	  {
	    int vc = vc_base + route.hops_traversed;
	    assert(vc<vc_limit);
	    
	    assert(is_vc_idle(vc, m_router->curCycle()));
	    m_outvc_state[vc]->setState(ACTIVE_, m_router->curCycle());
	    return vc;
	  }
      }
    else if(m_router->get_net_ptr()->is_escape_vc_enabled())
      {
	// escape-vc: if u want to go west, u cannot enter the
	// escape-vc.
	
	assert(m_router->get_net_ptr()->get_topology() == "Mesh");
	
	int num_rows = m_router->get_net_ptr()->getNumRows();
	int num_cols = m_router->get_net_ptr()->getNumCols();
	assert(num_rows > 0 && num_cols > 0);
	
	int my_id = m_router->get_id();
	int my_x = my_id % num_cols;
	
	int dest_id = route.dest_router;
	int dest_x = dest_id % num_cols;
	
	int ii = 0;
	
	if(dest_x < my_x)
	  ii = 1;  //have to go west
	
	for (int vc = vc_base; vc < vc_base + m_vc_per_vnet - ii; vc++) 
	  {
	    if (is_vc_idle(vc, m_router->curCycle()))
	      {
		m_outvc_state[vc]->setState(ACTIVE_, m_router->curCycle());
		return vc;
	      }
	  }
	
	return -1;
      }
    
    
    

    for (int vc = vc_base; vc < vc_base + m_vc_per_vnet; vc++) {
        if (is_vc_idle(vc, m_router->curCycle())) {
            m_outvc_state[vc]->setState(ACTIVE_, m_router->curCycle());
            return vc;
        }
    }

    return -1;
}

/*
 * The wakeup function of the OutputUnit reads the credit signal from the
 * downstream router for the output VC (i.e., input VC at downstream router).
 * It increments the credit count in the appropriate output VC state.
 * If the credit carries is_free_signal as true,
 * the output VC is marked IDLE.
 */

void
OutputUnit::wakeup()
{
  while (m_credit_link->isReady(m_router->curCycle())) 
    {
      Credit *t_credit = (Credit*) m_credit_link->consumeLink();


      //debug
      /*if( (m_router->get_id() == 42) && (m_direction == "East") )
	std::cout<<"router "<<m_router->get_id()<<" received VC free signal at outport: "<<m_direction
		 <<" in cycle: "<<m_router->curCycle()<<" for vc: "<<t_credit->get_vc()<<" is_move_credit:"
		 <<t_credit->is_move_credit()<<" is_free_signal: "<<t_credit->is_free_signal()<<endl<<std::flush;*/
     
 
      if(t_credit->is_move_credit())
	{
	  //only accept this credit if the source-id buffer at this router
	  // matches the source_id set in this credit_flit
	  //also the counter must be in either s_frozen or s_move

	  if( !((m_router->get_counter_state() == s_move) || (m_router->get_counter_state() == s_frozen) 
		|| (m_router->get_counter_state() == s_check_probe)) )
	    cout<<" ass. fail at router "<<m_router->get_id()<<" for direction: "
		<<m_direction<<" counter state:"<<m_router->get_counter_state()
		<<" in cycle: "<<m_router->curCycle()<<endl<<flush;
	  
	  assert( (m_router->get_counter_state() == s_move) || (m_router->get_counter_state() == s_frozen) || 
		  (m_router->get_counter_state() == s_check_probe) );
	  assert(m_router->partial_check_source_id_buffer(t_credit->get_source_id()));
	  

	  m_router->update_move_vc_at_downstream_router(t_credit->get_vc(), m_id);
	}
      else
	{
	  increment_credit(t_credit->get_vc());
	  
	  if (t_credit->is_free_signal())
	    {
	      set_vc_state(IDLE_, t_credit->get_vc(), m_router->curCycle());
	    }
	}
   
      delete t_credit;
    }
}

flitBuffer*
OutputUnit::getOutQueue()
{
    return m_out_buffer;
}

void
OutputUnit::set_out_link(NetworkLink *link)
{
    m_out_link = link;
}

void
OutputUnit::set_credit_link(CreditLink *credit_link)
{
    m_credit_link = credit_link;
}

uint32_t
OutputUnit::functionalWrite(Packet *pkt)
{
    return m_out_buffer->functionalWrite(pkt);
}

int OutputUnit::get_num_active_vcs(int vnet)
{
  int count = 0;

  int start_vc = vnet * m_router->get_vc_per_vnet();
  int end_vc = (vnet+1) * m_router->get_vc_per_vnet();

  for(int i=start_vc; i<end_vc; i++)
    {
      if(m_outvc_state[i]->isInState(ACTIVE_, m_router->curCycle()))
	count++;
    }
  return count;
}


Cycles OutputUnit::get_vc_active_time(int vc)
{
  assert(m_outvc_state[vc]->isInState(ACTIVE_, m_router->curCycle()));

  return (m_router->curCycle() - m_outvc_state[vc]->get_time());
}
