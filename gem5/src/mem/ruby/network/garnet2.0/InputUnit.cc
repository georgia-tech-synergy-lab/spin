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


#include "mem/ruby/network/garnet2.0/InputUnit.hh"

#include "base/stl_helpers.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/garnet2.0/Credit.hh"
#include "mem/ruby/network/garnet2.0/Router.hh"

using namespace std;
using m5::stl_helpers::deletePointers;

InputUnit::InputUnit(int id, PortDirection direction, Router *router)
            : Consumer(router)
{
    m_id = id;
    m_direction = direction;
    m_router = router;
    m_num_vcs = m_router->get_num_vcs();
    m_vc_per_vnet = m_router->get_vc_per_vnet();

    m_num_buffer_reads.resize(m_num_vcs/m_vc_per_vnet);
    m_num_buffer_writes.resize(m_num_vcs/m_vc_per_vnet);
    for (int i = 0; i < m_num_buffer_reads.size(); i++) {
        m_num_buffer_reads[i] = 0;
        m_num_buffer_writes[i] = 0;
    }

    creditQueue = new flitBuffer();
    // Instantiating the virtual channels
    m_vcs.resize(m_num_vcs);
    for (int i=0; i < m_num_vcs; i++) {
        m_vcs[i] = new VirtualChannel(i);
    }

    count = 0;
}

InputUnit::~InputUnit()
{
    delete creditQueue;
    deletePointers(m_vcs);
}

void InputUnit::init_spin_scheme()
{
  //spin scheme
  for (int i=0; i<m_router->get_num_outports(); i++)
    {
      fork_vector.push_back(false);
    }
  
  reset_spin_stats();
}

/*
 * The InputUnit wakeup function reads the input flit from its input link.
 * Each flit arrives with an input VC.
 * For HEAD/HEAD_TAIL flits, performs route computation,
 * and updates route in the input VC.
 * The flit is buffered for (m_latency - 1) cycles in the input VC
 * and marked as valid for SwitchAllocation starting that cycle.
 *
 */

void
InputUnit::wakeup()
{

  //debug
  bool condition1 = (m_router->get_id() == 59) && (m_direction == "Local");
  
  bool condition2 = (m_router->get_id() == -1) && (m_direction == "West") && (m_router->curCycle() >= 987) && 
    (m_router->curCycle() <= 989);
  
  bool condition3 = (m_router->get_id() == 34) && (m_direction == "South") && (m_router->curCycle() >= 987) && 
    (m_router->curCycle() <= 989);
  
  bool condition4 = (m_router->get_id() == 38) && (m_direction == "South") && (m_router->curCycle() >= 987) && 
    (m_router->curCycle() <= 989);
  
  bool condition5 = (m_router->get_id() == 30) && (m_direction == "North") && (m_router->curCycle() >= 987) && 
    (m_router->curCycle() <= 989);
  
  bool condition6 = (m_router->get_id() == 29) && ( (m_direction == "East") || (m_direction == "North") )
    && (m_router->curCycle() >= 987) && (m_router->curCycle() <= 989);
  
  bool condition7 = (m_router->get_id() == 28) && (m_direction == "East") && (m_router->curCycle() >= 987) && 
    (m_router->curCycle() <= 989);
  
  bool condition8 = (m_router->get_id() == 27) && (m_direction == "East") && (m_router->curCycle() >= 987) && 
    (m_router->curCycle() <= 989);
  
  bool condition9 = (m_router->get_id() == 26) && (m_direction == "East") && (m_router->curCycle() >= 987) && 
    (m_router->curCycle() <= 989);
  
  if(condition1 || condition2 || condition3 || condition4 || condition5 || condition6 || condition7 || condition8 
     || condition9)
    print_vc_outports(0);
  
  //end debug


    flit *t_flit;
    if (m_in_link->isReady(m_router->curCycle())) 
      {
        t_flit = m_in_link->consumeLink();

	//spin scheme
	if(t_flit->get_type() == PROBE_)
	  {
	    // 1. drop the probe if the counter is not in state deadlock_detection or state off
	    //    state off is necessary as this router might not have been chosen to be a static bubble node
	    //    bcoz of some placement algorithm
	    // 2. Drop it if it came from a lower source-id node.
	    // 3. Drop if if it has exhausted the turn carrying capacity

	    bool cond1 = (m_router->get_counter_state() == s_deadlock_detection) || (m_router->get_counter_state() == s_off);

	    bool cond2 = false;

	    //if sb_placement is enabled, then drop only if u are a sb node and this probe came from a sb node with lower id
	    //if the placement is not enabled then every node is a sb node, drop if this probe came from a node with lower-id

	    if(m_router->get_net_ptr()->is_rotating_priority_enabled())
	      {
		//	std::cout<<"function called from input unit\n"<<std::flush;

		cond2 = m_router->get_net_ptr()->
		  is_my_priority_greater(m_router->get_id(), t_flit->get_source_id());
	      }
	    else if(m_router->sb_placement_enabled())
	      {
		cond2 =  (m_router->is_sb_node()) && (m_router->get_id() > t_flit->get_source_id());
	      }
	    else
	      {
		cond2 =  (m_router->get_id() > t_flit->get_source_id());
	      }

	    bool cond3 = (t_flit->get_num_turns() > m_router->get_max_turn_capacity());

	    /*if(t_flit->get_source_id() == 37)
	      std::cout<<" probe "<<t_flit->get_id()<<" from router "<<t_flit->get_source_id()
		       <<" received at router "<<m_router->get_id()<<" in cycle "
		       <<m_router->curCycle()<<endl<<std::flush;*/

	    /*if(t_flit->get_source_id() == 42)
	      {

		if(cond2)
		  {
		    count++;
		    std::cout<<" probe "<<t_flit->get_id()<<" from router 42 ddropped due to id "
			     <<count<<endl<<std::flush;
		  }
		
		if(!cond1)
		  std::cout<<" probe "<<t_flit->get_id()<<" from 42 ddropped due to counter state at router "<<m_router->get_id()
			   <<" in cycle: "<<m_router->curCycle()<<std::flush;
		
		if(cond3)
		  std::cout<<" probe "<<t_flit->get_id()<<" from 42 ddropped due to turn capacity at router "<<m_router->get_id()
			   <<" in cycle: "<<m_router->curCycle()<<std::flush; 
			   }*/

	    if( (!cond1) || cond2 || cond3 )
	      {
		num_probes_dropped++;
		delete t_flit;
		return;
	      }

	    if(t_flit->get_source_id() == m_router->get_id())
	      {
		/*if(m_router->get_id() == 37)
		  std::cout<<" received probe back at router 28 in cycle "<<m_router->curCycle()<<" counter_state:"
		  <<m_router->get_counter_state()<<endl<<std::flush;*/

		/*if(m_router->get_id() == 42)
		  std::cout<<" probe "<<t_flit->get_id()<<" from router 42 received back at router 42 in cycle "
		  <<m_router->curCycle()<<endl<<std::flush;*/

		if(verify_dependence_at_source(t_flit))
		  {
		    //probe came back. send move msg and change counter state, 
		    //set source-id buffer : req. for processing move_credit
		    //create a move_info entry
		    m_router->set_loop_delay(t_flit->get_delay());
		    m_router->latch_path(t_flit);
		   
		    int move_id = m_router->send_move_msg(m_id, t_flit->get_source_vc());
		    m_router->latch_source_id_buffer(m_router->get_id(), move_id);

		    m_router->create_move_info_entry(m_id, t_flit->get_source_vc(), m_router->peek_path_top());
		    m_router->set_counter(t_flit->get_source_inp_port(), t_flit->get_source_vc(), s_move, 0);
		  }
		else
		  {
		    num_probes_dropped++;
		  }
		delete t_flit;
	      }
	    else
	      {
		//update inport of the probe.
		t_flit->set_inport(m_id);

		//check for buffer dependecies at this input port.
		//fork out of all outports for which there is a dependency except the local port.

		if(create_fork_vector(t_flit))
		  {
		    m_router->fork_probes(t_flit, fork_vector);
		  }
		else
		  {
		    if(t_flit->get_source_id() == 37)
		      {
			std::cout<<" probe from "<<t_flit->get_source_id()<<" dropped due to vc idle/ejection at router "
				 <<m_router->get_id()<<" in cycle: "<<m_router->curCycle()<<endl<<std::flush; 
			print_vc_outports(0);
		      }
		  }

		clear_fork_vector();
		delete t_flit;
	      }
	  }
	else if(t_flit->get_type() == MOVE_)
	  {   
	    std::cout<<" move "<<t_flit->get_id()<<" from router "<<t_flit->get_source_id()
		     <<" received at router "<<m_router->get_id()<<" in cycle "
		     <<m_router->curCycle()<<endl<<std::flush;

	    if(t_flit->get_source_id() == m_router->get_id())
	      {
		//it cannot happen that u get ur move back in the same cycle that u processed some kill_move
		assert(!m_router->get_kill_move_processed_this_cycle());
		assert(m_router->get_counter_state() == s_move);

		//move msg. came back
		if(verify_dependence_at_source(t_flit))
		  {
		    //set counter state to forward progress.
		    m_router->set_move_bit();
		    m_router->set_counter(m_id, t_flit->get_source_vc(), s_forward_progress, 0);

		    //send a move credit to the upstream router
		    send_move_credit(t_flit->get_source_vc(), t_flit->get_source_id(), m_router->curCycle());
		  }
		else
		  {
		    //send kill_move, invalidate path buffer, source-id buffer, change counter state to dd
		    m_router->send_kill_move(m_id);
		    m_router->invalidate_path_buffer();
		    m_router->invalidate_source_id_buffer();
		    m_router->increment_counter_ptr();
		    // m_router->invalidate_move_vc_at_downstream_router();
		    // m_router->invalidate_move_outport();
		    m_router->clear_move_registry();
		    num_move_dropped++;
		  }

		delete t_flit;
	      }
	    else
	      {
		//drop the move if counter not in deadlock detection or off state or a kill_move msg was processed this cycle
		//unless the move is received from a router whose move was previously honored : check_probe

		bool cond1 = (m_router->get_counter_state() == s_deadlock_detection) || (m_router->get_counter_state() == s_off) 
		  || (m_router->get_counter_state() == s_frozen);

		if( (!cond1) || (m_router->get_kill_move_processed_this_cycle()))
		  {
		    /*std::cout<<" move "<<t_flit->get_id()<<" from router "<<t_flit->get_source_id()
			     <<" dropped at router "<<m_router->get_id()<<" in cycle "
			     <<m_router->curCycle()<<" counter state: "<<m_router->get_counter_state()
			     <<endl<<std::flush;*/

		    num_move_dropped++;
		    delete t_flit;
		    return;
		  }

		if(m_router->get_counter_state() == s_frozen)
		  {
		    //accept this move only if it has visited before
		    //this is a cross-over router

		    if(!m_router->check_source_id_buffer(t_flit->get_source_id(), t_flit->get_id()))
		      {
			num_move_dropped++;
			delete t_flit;
			return;
		      }

		    //even if this a cross-over router, don't accept if the outport is already reserved
		    //for use by a VC in the "move" cycle
		    if(m_router->check_outport_entry_in_move_registry(t_flit->peek_path_top()))
		      {
			num_move_dropped++;
			delete t_flit;
			return;
		      }
		  }

		//update the inport
		t_flit->set_inport(m_id);

		int vc = find_move_vc(t_flit);
		
		if( vc != -1)
		  {
		    //set counter to frozen state. change counter ptr to point to this vc
		    m_router->set_move_bit();
		    m_router->latch_source_id_buffer(t_flit->get_source_id(), t_flit->get_id());
		    //m_router->set_move_outport(t_flit->peek_path_top(););

		    //freeze the vc and create entry in fifo to be used for credit registration
		    m_router->create_move_info_entry(m_id, vc, t_flit->peek_path_top());

		    m_router->set_counter(m_id, vc, s_frozen, t_flit->get_delay());
		    m_router->forward_move(t_flit);
   
		    //send a move credit to the upstream router
		    send_move_credit(vc, t_flit->get_source_id(), m_router->curCycle());
		  }
		else
		  {
		    num_move_dropped++;
		    delete t_flit;
		  }	
	      }
	  }
	else if(t_flit->get_type() == CHECK_PROBE_)
	  {
	    std::cout<<" router "<<m_router->get_id()<<"received check_probe from router "<<t_flit->get_source_id()
		     <<" in cycle "<<m_router->curCycle()<<" counter state:"<<m_router->get_counter_state()
		     <<endl<<std::flush;

	    if(t_flit->get_source_id() == m_router->get_id())
	      {
		assert(m_router->get_counter_state() == s_check_probe);

		//it cannot happen that u get ur check_probe back in the same cycle that u processed some kill_move
		assert(!m_router->get_kill_move_processed_this_cycle());
		
		//check_probe came back
		if(verify_dependence_at_source(t_flit))
		  {
		    //set counter state to forward progress.
		    m_router->set_move_bit();
		    m_router->set_counter(m_id, t_flit->get_source_vc(), s_forward_progress, 0);

		    //send a move credit to the upstream router
		    send_move_credit(t_flit->get_source_vc(), t_flit->get_source_id(), m_router->curCycle());
		  }
		else
		  {
		    //send kill_move, invalidate path buffer, source-id buffer, change counter state to dd
		    m_router->send_kill_move(m_id);
		    m_router->invalidate_path_buffer();
		    m_router->invalidate_source_id_buffer();
		    m_router->increment_counter_ptr();
		    // m_router->invalidate_move_vc_at_downstream_router();
		    // m_router->invalidate_move_outport();
		    m_router->clear_move_registry();
		    num_check_probe_dropped++;
		  }

		delete t_flit;
	      }
	    else
	      {
		//check probe arrived
		//counter should be in frozen state
		//only the source-id should have sent it
		//find a move_vc, if cannot find drop this msg
		//update the move_registry entry, remember to un-freeze the previous vc

		assert(m_router->get_counter_state() == s_frozen);
		assert(m_router->partial_check_source_id_buffer(t_flit->get_source_id()));

		//update the inport
		t_flit->set_inport(m_id);

		int vc = find_move_vc(t_flit);
		
		if( vc != -1)
		  {
		    //set counter to frozen state. don't touch the source-id buffer
		    m_router->set_move_bit();
		    
		    //freeze the new vc and un-freeze the previous one
		    m_router->update_move_info_entry(m_id, vc, t_flit->peek_path_top());
		    
		    m_router->set_counter(m_id, vc, s_frozen, t_flit->get_delay());
		    m_router->forward_check_probe(t_flit);
		    
		    //send a move credit to the upstream router
		    send_move_credit(vc, t_flit->get_source_id(), m_router->curCycle());
		  }
		else
		  {
		    num_check_probe_dropped++;
		    delete t_flit;
		  }
	      }
	  }
	else if(t_flit->get_type() == KILL_MOVE_)
	  {
	    //if(m_router->get_id() == 42)
	    //{
	    std::cout<<" router "<<m_router->get_id()<<"received kill_move from router "<<t_flit->get_source_id()
		     <<" in cycle "<<m_router->curCycle()<<" counter state:"<<m_router->get_counter_state()
		     <<endl<<std::flush;
		//}

	    if(t_flit->get_source_id() == m_router->get_id())
	      {
		//kill_move came back. simply drop it. change of counter state was already done when it was sent out.
		delete t_flit;
	      }
	    else
	      {
		//update inport
		t_flit->set_inport(m_id);

		if(m_router->partial_check_source_id_buffer(t_flit->get_source_id()))
		  {
		    t_flit->set_must_send();
		    m_router->set_kill_move_processed_this_cycle();

		    //invalidate the entry in move_registry corresponding to this (inport,outport)
		    //change counter state only when there are no entries in the move_registry
		    
		    assert(m_router->get_counter_state() == s_frozen);
		    assert(m_router->get_num_move_registry_entries() >= 1);

		    if(m_router->get_num_move_registry_entries() == 1)
		      {
			m_router->reset_move_bit();
			m_router->increment_counter_ptr();
			m_router->invalidate_source_id_buffer();
			
			m_router->clear_move_registry();
		      }
		    else
		      {
			m_router->invalidate_move_registry_entry(m_id, t_flit->peek_path_top());
		      }
		  }
		else
		  {
		    //reset must_send, just to be on the safe side
		    t_flit->reset_must_send();
		  }

		m_router->forward_kill_move(t_flit);
	      }
	  }
	else
	  {
	    int vc = t_flit->get_vc();
	    t_flit->increment_hops(); // for stats

	    //if(m_direction == "Local")
	    //std::cout<<" hops are: "<<t_flit->get_hops()<<" at router "<<m_router->get_id()
	    //	     <<std::endl<<std::flush;
	    
	    /*std::cout<<"router "<<m_router->get_id()<<" received flit "<<t_flit->get_id()
		     <<" at inport "<<m_direction<<" in cycle "<<m_router->curCycle()
		     <<std::endl<<std::flush;*/
	    
 
	    if ((t_flit->get_type() == HEAD_) || (t_flit->get_type() == HEAD_TAIL_)) 
	      {

		if(t_flit->is_part_of_move())
		  {
		    t_flit->reset_part_of_move();
		    assert(m_vcs[vc]->get_state() == ACTIVE_);
		  }
		else
		  {
		    assert(m_vcs[vc]->get_state() == IDLE_);
		    set_vc_active(vc, m_router->curCycle());
		  }
		
		// Route computation for this vc
		int outport = m_router->route_compute(t_flit->get_route_ptr(), m_id, m_direction, vc);

		// Update output port in VC
		// All flits in this packet will use this output port
		// The output port field in the flit is updated after it wins SA
		grant_outport(vc, outport);

		//debug
		/*if(m_router->get_net_ptr()->get_topology() == "dragonfly")
		  {
		    if(m_router->getOutportDirection(outport) == "Inter")
		      t_flit->get_route_ptr()->inter_link_count++;

		    assert(t_flit->get_route_ptr()->inter_link_count <= 2);
		    }*/
		
		if( m_router->spin_scheme_enabled() && (m_router->get_counter_state() == s_off) && (m_direction != "Local") )
		  {
		    //make counter point to this vc if the outport on which the VC waits is not local
		    if(m_router->getOutportDirection(outport) != "Local")
		      m_router->set_counter(m_id, vc, s_deadlock_detection, 0);
		  }
	      } 
	    else 
	      {
		t_flit->reset_part_of_move();
		
		/*if(m_vcs[vc]->get_state() != ACTIVE_)
		  cout<<" counter state: "<<m_router->get_counter_state()<<" flit_type: "<<t_flit->get_type()
		      <<" at router:"<<m_router->get_id()<<" in cycle:"<<m_router->curCycle()
		      <<" vc state:"<<m_vcs[vc]->get_state()<<" vc:"<<vc<<" for inport direction: "
		      <<m_direction<<endl<<flush;*/

		assert(m_vcs[vc]->get_state() == ACTIVE_);
	      }
	    
	    
	    // Buffer the flit
	    m_vcs[vc]->insertFlit(t_flit);
	    
	    int vnet = vc/m_vc_per_vnet;
	    // number of writes same as reads
	    // any flit that is written will be read only once
	    m_num_buffer_writes[vnet]++;
	    m_num_buffer_reads[vnet]++;
	    
	    Cycles pipe_stages = m_router->get_pipe_stages();
	    if (pipe_stages == 1) 
	      {
		// 1-cycle router
		// Flit goes for SA directly
		t_flit->advance_stage(SA_, m_router->curCycle());
	      } 
	    else 
	      {
		assert(pipe_stages > 1);
		// Router delay is modeled by making flit wait in buffer for
		// (pipe_stages cycles - 1) cycles before going for SA

		Cycles wait_time = pipe_stages - Cycles(1);
		t_flit->advance_stage(SA_, m_router->curCycle() + wait_time);
		
		// Wakeup the router in that cycle to perform SA
		m_router->schedule_wakeup(Cycles(wait_time));
	      }
	  }
    }
}

// Send a credit back to upstream router for this VC.
// Called by SwitchAllocator when the flit in this VC wins the Switch.
void
InputUnit::increment_credit(int in_vc, bool free_signal, Cycles curTime)
{
    Credit *t_credit = new Credit(in_vc, free_signal, curTime);
    creditQueue->insert(t_credit);
    m_credit_link->scheduleEventAbsolute(m_router->clockEdge(Cycles(1)));
}


uint32_t
InputUnit::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;
    for (int i=0; i < m_num_vcs; i++) {
        num_functional_writes += m_vcs[i]->functionalWrite(pkt);
    }

    return num_functional_writes;
}

void
InputUnit::resetStats()
{
    for (int j = 0; j < m_num_buffer_reads.size(); j++) {
        m_num_buffer_reads[j] = 0;
        m_num_buffer_writes[j] = 0;
    }
}


//spin scheme 

bool InputUnit::verify_dependence_at_source(flit *t_flit)
{
  //check if the input port is the same as when the probe was sent out
  if(t_flit->get_source_inp_port() != m_id)
    {
      assert(t_flit->get_type() == PROBE_);

      std::cout<<" PROBE "<<t_flit->get_id()<<" from router "<<t_flit->get_source_id()
	       <<" dropped at router "<<m_router->get_id()<<" bcoz it came from diff. inport in cycle "
	       <<m_router->curCycle()<<" Input port: "<<m_direction<<" source_outport: "
	       <<m_router->getOutportDirection(t_flit->peek_path_top())<<" source_inport: "
	       <<m_router->getInportDirection(t_flit->get_source_inp_port())
	       <<endl<<endl<<std::flush;

      print_vc_outports(t_flit->get_vnet());

      return false;
    }

  //check for any idle and vcs waiting for the local port
  int vnet = t_flit->get_vnet();
  int start_vc = vnet*m_vc_per_vnet;
  int end_vc = (vnet+1)*m_vc_per_vnet;

  for (int i=start_vc; i<end_vc; i++)
    {
      if( (m_vcs[i]->get_state() != ACTIVE_) || (m_router->getOutportDirection(m_vcs[i]->get_outport()) == "Local") )
	return false;
    }

  //check if the source-vc is still waiting for the same output port
  // as when the probe was sent out

  int vc = t_flit->get_source_vc();

  if(t_flit->get_type() == PROBE_)
    {
      if(m_vcs[vc]->get_outport() != t_flit->peek_path_top())
	return false;
    }
  else if( (t_flit->get_type() == MOVE_) || (t_flit->get_type() == CHECK_PROBE_) )
    {
      // assert(m_router->get_counter_state() == s_move);
      // assert(m_router->check_counter_ptr(m_id, t_flit->get_source_vc()));

      if(m_vcs[vc]->get_outport() != m_router->peek_path_top())
	return false;

      //also check if this VC contains both head and tail.
      if(!m_vcs[vc]->contains_head_n_tail())
	return false;
    }
  else
    {
      assert(false);
    }

  return true;
}

bool InputUnit::create_fork_vector(flit *t_flit)
{
  int vnet = t_flit->get_vnet();
  int start_vc = vnet*m_vc_per_vnet;
  int end_vc = (vnet+1)*m_vc_per_vnet;

  for (int i=start_vc; i<end_vc; i++)
    {
      if( (m_vcs[i]->get_state() != ACTIVE_) || (m_router->getOutportDirection(m_vcs[i]->get_outport()) == "Local") )
	return false;

      fork_vector[m_vcs[i]->get_outport()] = true;
    }

  return true;
}

int InputUnit::find_move_vc(flit *t_flit)
{
  int vnet = t_flit->get_vnet();
  int start_vc = vnet*m_vc_per_vnet;
  int end_vc = (vnet+1)*m_vc_per_vnet;

  int outport = t_flit->peek_path_top();

  int vc = -1;

  for(int i=start_vc; i<end_vc; i++)
    {
      if( (m_vcs[i]->get_state() != ACTIVE_) || (m_router->getOutportDirection(m_vcs[i]->get_outport()) == "Local") )
	{
	  std::cout<<" move "<<t_flit->get_id()<<" from router "<<t_flit->get_source_id()
		   <<" dropped at router "<<m_router->get_id()<<" due to vc IDLE or waiting for ejection in cycle "
		   <<m_router->curCycle()<<" counter state: "<<m_router->get_counter_state()
		   <<endl<<std::flush;

	  return -1;
	}

      //check if its a compelete packet: both head and tail present
      //we only want to freeze a complete packet
   
      if( (vc == -1) && (m_vcs[i]->get_outport() == outport) && (m_vcs[i]->contains_head_n_tail()) )
	{
	  vc = i;
	}
      else if((m_vcs[i]->get_outport() == outport) && (!(m_vcs[i]->contains_head_n_tail())) )
	{
	  std::cout<<" move "<<t_flit->get_id()<<" from router "<<t_flit->get_source_id()
		   <<" dropped at router "<<m_router->get_id()<<" due to not having both head n tail, in cycle "
		   <<m_router->curCycle()<<" counter state: "<<m_router->get_counter_state()
		   <<endl<<std::flush;
	}
    }

  return vc;
}

void InputUnit::clear_fork_vector()
{
  for (int i=0; i<m_router->get_num_outports(); i++)
    {
      fork_vector[i] = false;
    }
}

//to send the move credit to the upstream router.
//used by the upstream router to correctly account
//for credits when doing the move.
//packets with different flit sizes may be allowed
//to use the same vnet by the coherence protocol
void InputUnit::send_move_credit(int in_vc, int source_id, Cycles curTime)
{
  Credit *t_credit = new Credit(in_vc, source_id, curTime, true);
  creditQueue->insert(t_credit);
  m_credit_link->scheduleEventAbsolute(m_router->clockEdge(Cycles(1)));
}

void InputUnit::reset_spin_stats()
{
  num_probes_dropped = 0;
  num_move_dropped = 0;
  num_check_probe_dropped = 0;
}

void InputUnit::print_vc_outports(int vnet)
{
  int start_vc = vnet*m_vc_per_vnet;
  int end_vc = (vnet+1)*m_vc_per_vnet;

  for(int i=start_vc; i<end_vc; i++)
    {
      if(get_vc_state(i) == ACTIVE_)
	std::cout<<" VC "<<i<<" at router "<<m_router->get_id()<<" is waiting for outport "
		 <<m_router->getOutportDirection(get_outport(i))<<" in cycle "
		 <<m_router->curCycle()<<endl<<std::flush;
      else
	std::cout<<"VC "<<i<<" at router "<<m_router->get_id()<<" is IDLE in cycle "<<m_router->curCycle()<<endl<<std::flush; 
    }
}
