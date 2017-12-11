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


#include "mem/ruby/network/garnet2.0/SwitchAllocator.hh"

#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/garnet2.0/GarnetNetwork.hh"
#include "mem/ruby/network/garnet2.0/InputUnit.hh"
#include "mem/ruby/network/garnet2.0/OutputUnit.hh"
#include "mem/ruby/network/garnet2.0/Router.hh"

using namespace std;

SwitchAllocator::SwitchAllocator(Router *router)
    : Consumer(router)
{
    m_router = router;
    m_num_vcs = m_router->get_num_vcs();
    m_vc_per_vnet = m_router->get_vc_per_vnet();

    m_input_arbiter_activity = 0;
    m_output_arbiter_activity = 0;
}

void
SwitchAllocator::init()
{
    m_input_unit = m_router->get_inputUnit_ref();
    m_output_unit = m_router->get_outputUnit_ref();

    m_num_inports = m_router->get_num_inports();
    m_num_outports = m_router->get_num_outports();
    m_round_robin_inport.resize(m_num_outports);
    m_round_robin_invc.resize(m_num_inports);
    m_port_requests.resize(m_num_outports);
    m_vc_winners.resize(m_num_outports);

    for (int i = 0; i < m_num_inports; i++) {
        m_round_robin_invc[i] = 0;
    }

    for (int i = 0; i < m_num_outports; i++) {
        m_port_requests[i].resize(m_num_inports);
        m_vc_winners[i].resize(m_num_inports);

        m_round_robin_inport[i] = 0;

        for (int j = 0; j < m_num_inports; j++) {
            m_port_requests[i][j] = false; // [outport][inport]
        }
    }

    //spin scheme
    probeQueue = m_router->getprobeQueue_ptr();
    moveQueue = m_router->get_moveQueue_ptr();
    kill_moveQueue = m_router->get_kill_moveQueue_ptr();
    check_probeQueue = m_router->get_check_probeQueue_ptr();
    cur_move_count = 0;
    //reset_tail_moved();
    reset_stats();

    for (int i = 0; i < m_num_outports; i++)
      {
	outport_reservations.push_back(false);
      }
}

/*
 * The wakeup function of the SwitchAllocator performs a 2-stage
 * seperable switch allocation. At the end of the 2nd stage, a free
 * output VC is assigned to the winning flits of each output port.
 * There is no separate VCAllocator stage like the one in garnet1.0.
 * At the end of this function, the router is rescheduled to wakeup
 * next cycle for peforming SA for any flits ready next cycle.
 */

void
SwitchAllocator::wakeup()
{
  //spin scheme
  if(m_router->spin_scheme_enabled())
    {
      if(m_router->get_start_move())
	effectuate_move();
      
      send_check_probe();
      send_move();
      send_kill_move();
      send_probes();
    }

    arbitrate_inports(); // First stage of allocation
    arbitrate_outports(); // Second stage of allocation

    clear_request_vector();
    check_for_wakeup();

    //spin scheme
    if(m_router->spin_scheme_enabled())
      clear_reservations();
}

/*
 * SA-I (or SA-i) loops through all input VCs at every input port,
 * and selects one in a round robin manner.
 *    - For HEAD/HEAD_TAIL flits only selects an input VC whose output port
 *     has at least one free output VC.
 *    - For BODY/TAIL flits, only selects an input VC that has credits
 *      in its output VC.
 * Places a request for the output port from this input VC.
 */

void
SwitchAllocator::arbitrate_inports()
{
    // Select a VC from each input in a round robin manner
    // Independent arbiter at each input port
    for (int inport = 0; inport < m_num_inports; inport++) {
        int invc = m_round_robin_invc[inport];

        for (int invc_iter = 0; invc_iter < m_num_vcs; invc_iter++) {

            if (m_input_unit[inport]->need_stage(invc, SA_,
                m_router->curCycle())) {

                // This flit is in SA stage

                int  outport = m_input_unit[inport]->get_outport(invc);
                int  outvc   = m_input_unit[inport]->get_outvc(invc);

                // check if the flit in this InputVC is allowed to be sent
                // send_allowed conditions described in that function.
                bool make_request =
                    send_allowed(inport, invc, outport, outvc);

                if (make_request) {
                    m_input_arbiter_activity++;
                    m_port_requests[outport][inport] = true;
                    m_vc_winners[outport][inport]= invc;

                    // Update Round Robin pointer
                    m_round_robin_invc[inport]++;
                    if (m_round_robin_invc[inport] >= m_num_vcs)
                        m_round_robin_invc[inport] = 0;

                    break; // got one vc winner for this port
                }
            }

            invc++;
            if (invc >= m_num_vcs)
                invc = 0;
        }
    }
}

/*
 * SA-II (or SA-o) loops through all output ports,
 * and selects one input VC (that placed a request during SA-I)
 * as the winner for this output port in a round robin manner.
 *      - For HEAD/HEAD_TAIL flits, performs simplified outvc allocation.
 *        (i.e., select a free VC from the output port).
 *      - For BODY/TAIL flits, decrement a credit in the output vc.
 * The winning flit is read out from the input VC and sent to the
 * CrossbarSwitch.
 * An increment_credit signal is sent from the InputUnit
 * to the upstream router. For HEAD_TAIL/TAIL flits, is_free_signal in the
 * credit is set to true.
 */

void
SwitchAllocator::arbitrate_outports()
{
    // Now there are a set of input vc requests for output vcs.
    // Again do round robin arbitration on these requests
    // Independent arbiter at each output port
    for (int outport = 0; outport < m_num_outports; outport++) {

      //spin scheme : don't do arbitration for this port if some spl. msg. would be using it 
      //              in this cycle.
      if( (m_router->spin_scheme_enabled()) && (outport_reservations[outport]) )
	continue;

        int inport = m_round_robin_inport[outport];

        for (int inport_iter = 0; inport_iter < m_num_inports;
                 inport_iter++) {

            // inport has a request this cycle for outport
            if (m_port_requests[outport][inport]) {

                // grant this outport to this inport
                int invc = m_vc_winners[outport][inport];

                int outvc = m_input_unit[inport]->get_outvc(invc);
                if (outvc == -1) {
                    // VC Allocation - select any free VC from outport
                    outvc = vc_allocate(outport, inport, invc);
                }

                // remove flit from Input VC
                flit *t_flit = m_input_unit[inport]->getTopFlit(invc);

                DPRINTF(RubyNetwork, "SwitchAllocator at Router %d "
                                     "granted outvc %d at outport %d "
                                     "to invc %d at inport %d to flit %s at "
                                     "time: %lld\n",
                        m_router->get_id(), outvc,
                        m_router->getPortDirectionName(
                            m_output_unit[outport]->get_direction()),
                        invc,
                        m_router->getPortDirectionName(
                            m_input_unit[inport]->get_direction()),
                            *t_flit,
                        m_router->curCycle());


                // Update outport field in the flit since this is
                // used by CrossbarSwitch code to send it out of
                // correct outport.
                // Note: post route compute in InputUnit,
                // outport is updated in VC, but not in flit
                t_flit->set_outport(outport);

                // set outvc (i.e., invc for next hop) in flit
                // (This was updated in VC by vc_allocate, but not in flit)
                t_flit->set_vc(outvc);

                // decrement credit in outvc
                m_output_unit[outport]->decrement_credit(outvc);

                // flit ready for Switch Traversal
                t_flit->advance_stage(ST_, m_router->curCycle());
                m_router->grant_switch(inport, t_flit);
                m_output_arbiter_activity++;

                if ((t_flit->get_type() == TAIL_) ||
                    t_flit->get_type() == HEAD_TAIL_) {


		  if(m_input_unit[inport]->isReady(invc, m_router->curCycle()) )
		    std::cout<<" vc not empty after sending tail/head_tail at router "<<m_router->get_id()
			     <<" in cycle "<<m_router->curCycle()<<" vc: "<<invc
			     <<" inport: "<<m_router->getInportDirection(inport)<<endl<<std::flush;


                    // This Input VC should now be empty
                    assert(!(m_input_unit[inport]->isReady(invc,
                        m_router->curCycle())));

                    // Free this VC
                    m_input_unit[inport]->set_vc_idle(invc,
                        m_router->curCycle());

                    // Send a credit back
                    // along with the information that this VC is now idle
                    m_input_unit[inport]->increment_credit(invc, true,
                        m_router->curCycle());

		    /*if( (m_router->get_id() == 30) && (m_router->getInportDirection(inport) == "North") )
		      cout<<"router "<<m_router->get_id()<<" sent free signal/credit:"
			  <<" , flit moved in direction: "<<m_router->getOutportDirection(outport)
			  <<" direction in cycle "<<m_router->curCycle()<<" from vc: "<<invc
			  <<" is vc frozen: "<<m_input_unit[inport]->is_vc_frozen(invc)<<endl<<flush;*/

                } else {
                    // Send a credit back
                    // but do not indicate that the VC is idle
                    m_input_unit[inport]->increment_credit(invc, false,
                        m_router->curCycle());
                }

                // remove this request
                m_port_requests[outport][inport] = false;

                // Update Round Robin pointer
                m_round_robin_inport[outport]++;
                if (m_round_robin_inport[outport] >= m_num_inports)
                    m_round_robin_inport[outport] = 0;

		/*if( (m_router->get_id() == 3) && (m_router->getOutportDirection(outport) == "West") )*/
		/*cout<<"router "<<m_router->get_id()<<"sent flit "<<t_flit->get_id()
		    <<" of type:"<<t_flit->get_type()
		    <<" in "<<m_router->getOutportDirection(outport)
		    <<" direction in cycle "<<m_router->curCycle()<<" from input vc "<<invc
		    <<" for outvc "<<outvc<<" for destination "<<t_flit->get_route().dest_router
		    <<" vnet "<<t_flit->get_vnet()<<endl<<flush;*/
		
		/*if( (m_router->get_id() == 38) && (m_router->getOutportDirection(outport) == "South") )
		  cout<<"router "<<m_router->get_id()<<" sent flit of type:"<<t_flit->get_type()
		      <<" in "<<m_router->getOutportDirection(outport)
		      <<" direction in cycle "<<m_router->curCycle()<<" for output vc "<<outvc<<endl<<flush;*/ 

		//spin scheme
		
		//check to see if this VC was pointed to by the counter
		// If so, increment the counter pointer
		if( m_router->spin_scheme_enabled() && m_router->check_counter_ptr(inport, invc) && 
		    (m_router->get_counter_state() == s_deadlock_detection) )
		  m_router->increment_counter_ptr();

                break; // got a input winner for this outport
            }

            inport++;
            if (inport >= m_num_inports)
                inport = 0;
        }
    }
}

/*
 * A flit can be sent only if
 * (1) there is at least one free output VC at the
 *     output port (for HEAD/HEAD_TAIL),
 *  or
 * (2) if there is at least one credit (i.e., buffer slot)
 *     within the VC for BODY/TAIL flits of multi-flit packets.
 * and
 * (3) pt-to-pt ordering is not violated in ordered vnets, i.e.,
 *     there should be no other flit in this input port
 *     within an ordered vnet
 *     that arrived before this flit and is requesting the same output port.
 *
 *
 *
 *  SPIN scheme: If the VC is frozen, don't allow it to send flits.
 *               This will only be allowed in the "move" cycle.
 */

bool
SwitchAllocator::send_allowed(int inport, int invc, int outport, int outvc)
{
  //spin scheme
  if( (m_router->spin_scheme_enabled()) && (m_input_unit[inport]->is_vc_frozen(invc)) )
    {
      assert( (m_router->get_counter_state() == s_frozen) || (m_router->get_counter_state() == s_forward_progress) || 
	      (m_router->get_counter_state() == s_check_probe) || (m_router->get_counter_state() == s_move) );


      /*std::cout<<" vc "<<invc<<" at router "<<m_router->get_id()<<" waiting for outport "
	       <<m_router->getOutportDirection(outport)<<" is frozen in cycle "<<m_router->curCycle()
	       <<endl<<std::flush;*/

      return false;
    }

    PortDirection inport_dirn  = m_input_unit[inport]->get_direction();
    PortDirection outport_dirn = m_output_unit[outport]->get_direction();
    RouteInfo route = m_input_unit[inport]->peekTopFlit(invc)->get_route();

    // Check if outvc needed
    // Check if credit needed (for multi-flit packet)
    // Check if ordering violated (in ordered vnet)

    int vnet = get_vnet(invc);
    bool has_outvc = (outvc != -1);
    bool has_credit = false;

    if (!has_outvc) {

        // needs outvc
        // this is only true for HEAD and HEAD_TAIL flits.

        if (m_output_unit[outport]->has_free_vc(vnet, invc,
                                    inport_dirn, outport_dirn, route)) {

            has_outvc = true;

            // each VC has at least one buffer,
            // so no need for additional credit check
            has_credit = true;
        }
    } else {
        has_credit = m_output_unit[outport]->has_credit(outvc);
    }

    // cannot send if no outvc or no credit.
    if (!has_outvc || !has_credit)
        return false;


    // protocol ordering check
    if ((m_router->get_net_ptr())->isVNetOrdered(vnet)) {

        // enqueue time of this flit
        Cycles t_enqueue_time = m_input_unit[inport]->get_enqueue_time(invc);

        // check if any other flit is ready for SA and for same output port
        // and was enqueued before this flit
        int vc_base = vnet*m_vc_per_vnet;
        for (int vc_offset = 0; vc_offset < m_vc_per_vnet; vc_offset++) {
            int temp_vc = vc_base + vc_offset;
            if (m_input_unit[inport]->need_stage(temp_vc, SA_,
                                                 m_router->curCycle()) &&
               (m_input_unit[inport]->get_outport(temp_vc) == outport) &&
               (m_input_unit[inport]->get_enqueue_time(temp_vc) <
                    t_enqueue_time)) {
                return false;
            }
        }
    }

    return true;
}

// Assign a free VC to the winner of the output port.
int
SwitchAllocator::vc_allocate(int outport, int inport, int invc)
{
    // ICN Lab 3:
    // Hint: invc, route, inport_dirn, outport_dirn are provided
    // to implement escape VC
    PortDirection inport_dirn  = m_input_unit[inport]->get_direction();
    PortDirection outport_dirn = m_output_unit[outport]->get_direction();
    RouteInfo route = m_input_unit[inport]->peekTopFlit(invc)->get_route();

    // Select a free VC from the output port
    int vnet = get_vnet(invc);
    int outvc = m_output_unit[outport]->select_free_vc(vnet, invc,
                                        inport_dirn, outport_dirn, route);

    // has to get a valid VC since it checked before performing SA
    assert(outvc != -1);
    m_input_unit[inport]->grant_outvc(invc, outvc);
    return outvc;
}

// Wakeup the router next cycle to perform SA again
// if there are flits ready.
void
SwitchAllocator::check_for_wakeup()
{
    Cycles nextCycle = m_router->curCycle() + Cycles(1);

    for (int i = 0; i < m_num_inports; i++) {
        for (int j = 0; j < m_num_vcs; j++) {
            if (m_input_unit[i]->need_stage(j, SA_, nextCycle)) {
                m_router->schedule_wakeup(Cycles(1));
                return;
            }
        }
    }
}

int
SwitchAllocator::get_vnet(int invc)
{
    int vnet = invc/m_vc_per_vnet;
    assert(vnet < m_router->get_num_vnets());
    return vnet;
}


// Clear the request vector within the allocator at end of SA-II.
// Was populated by SA-I.
void
SwitchAllocator::clear_request_vector()
{
    for (int i = 0; i < m_num_outports; i++) {
        for (int j = 0; j < m_num_inports; j++) {
            m_port_requests[i][j] = false;
        }
    }
}

void SwitchAllocator::send_probes()
{
  while(probeQueue->isReady(m_router->curCycle()))
    {
      flit *probe = probeQueue->getTopFlit();
      int outport = probe->get_outport();
      int inport = probe->get_inport();

      assert( (outport>=0) && (outport<m_num_outports) );

      if(outport_reservations[outport])
	{
	  if( (m_router->get_id() == 37) || (probe->get_source_id() == 37) || (m_router->get_id() == 29) )
	    {
	      std::cout<<"probe "<<probe->get_id()<<" from router "<<probe->get_source_id()
		       <<" dropped in SWA of router "<<m_router->get_id()
		       <<" for outport "<<m_router->getOutportDirection(outport)
		       <<" in cycle "<<m_router->curCycle()<<endl<<std::flush;
	    }

	  probes_dropped++;
	  delete probe;
	}
      else
	{
	  outport_reservations[outport] = true;
	  probe->advance_stage(ST_, m_router->curCycle());
	  m_router->grant_switch(inport, probe);
	  m_output_arbiter_activity++; 

	  if( (m_router->get_id() == 37) || (probe->get_source_id() == 37) || (m_router->get_id() == 29) )
	    {
	      std::cout<<"probe "<<probe->get_id()<<" from router "<<probe->get_source_id()
		       <<" forwarded in SWA of router "<<m_router->get_id()
		       <<" for outport "<<m_router->getOutportDirection(outport)
		       <<" in cycle "<<m_router->curCycle()<<endl<<std::flush;
	    }
	}
    }
}

void SwitchAllocator::send_check_probe()
{
  assert(check_probeQueue->get_num_flits() <= 1);

  if(check_probeQueue->isReady(m_router->curCycle()))
    {
      flit *check_probe = check_probeQueue->getTopFlit();
      int outport = check_probe->get_outport();
      int inport = check_probe->get_inport();

      //counter should be in either the frozen or check_probe state
      assert( (m_router->get_counter_state() == s_frozen) || (m_router->get_counter_state() == s_check_probe) );

      assert(!outport_reservations[outport]);

      outport_reservations[outport] = true;
      check_probe->advance_stage(ST_, m_router->curCycle());
      m_router->grant_switch(inport, check_probe);
      m_output_arbiter_activity++;


      /*if(m_router->get_id() == 37)
	std::cout<<"move "<<move->get_id()<<" from router "<<move->get_source_id()
		 <<" forwarded in SWA of router "<<m_router->get_id()
		 <<" for outport "<<m_router->getOutportDirection(outport)
		 <<" in cycle "<<m_router->curCycle()<<endl<<std::flush;*/
    }
}

void SwitchAllocator::send_move()
{
  assert(moveQueue->get_num_flits() <= 1);

  if(moveQueue->isReady(m_router->curCycle()))
    {
      flit *move = moveQueue->getTopFlit();
      int outport = move->get_outport();
      int inport = move->get_inport();

      assert( (m_router->get_counter_state() == s_frozen) || (m_router->get_counter_state() == s_move) );
      assert(!outport_reservations[outport]);

      outport_reservations[outport] = true;
      move->advance_stage(ST_, m_router->curCycle());
      m_router->grant_switch(inport, move);
      m_output_arbiter_activity++;


      if(m_router->get_id() == 37)
	std::cout<<"move "<<move->get_id()<<" from router "<<move->get_source_id()
		 <<" forwarded in SWA of router "<<m_router->get_id()
		 <<" for outport "<<m_router->getOutportDirection(outport)
		 <<" in cycle "<<m_router->curCycle()<<endl<<std::flush;
    }
}

void SwitchAllocator::send_kill_move()
{
  while(kill_moveQueue->isReady(m_router->curCycle()))
    {
      flit *kill_move = kill_moveQueue->getTopFlit();
      int outport = kill_move->get_outport();
      int inport = kill_move->get_inport();

      
      if(kill_move->get_must_send())
	{
	  //if this is a must_send msg., the outport shouldn't be reserved
	  assert(!outport_reservations[outport]);

	  outport_reservations[outport] = true;
	  kill_move->advance_stage(ST_, m_router->curCycle());
	  m_router->grant_switch(inport, kill_move);
	  m_output_arbiter_activity++;

	  if(m_router->get_id() == 37)
	    std::cout<<"kill_move "<<kill_move->get_id()<<" from router "<<kill_move->get_source_id()
		     <<" forwarded in SWA of router "<<m_router->get_id()
		     <<" for outport "<<m_router->getOutportDirection(outport)
		     <<" in cycle "<<m_router->curCycle()<<endl<<std::flush;
	  
	}
      else
	{
	  if(outport_reservations[outport])
	    {
	      kill_move_dropped++;
	      delete kill_move;
	    }
	  else
	    {
	      outport_reservations[outport] = true;
	      kill_move->advance_stage(ST_, m_router->curCycle());
	      m_router->grant_switch(inport, kill_move);
	      m_output_arbiter_activity++;


	  if(m_router->get_id() == 37)
	    std::cout<<"kill_move "<<kill_move->get_id()<<" from router "<<kill_move->get_source_id()
		     <<" forwarded in SWA of router "<<m_router->get_id()
		     <<" for outport "<<m_router->getOutportDirection(outport)
		     <<" in cycle "<<m_router->curCycle()<<endl<<std::flush;
	    }
	}
    }
}

//cout<<"hola inside eff. move at router: "<<m_router->get_id()<<endl<<flush;

void SwitchAllocator::effectuate_move()
{
  //cycle through all entries in the move_registry
  //and send them out flit-by-flit 
  
  assert( (m_router->get_counter_state() == s_frozen) || (m_router->get_counter_state() == s_forward_progress) );

  const std::vector<move_info *>& registry = m_router->get_move_registry();
  assert(registry.size() >= 1);
  
  for(int i=0; i<registry.size(); i++)
    {
      if(registry[i]->tail_moved)
	continue;
	  
      // remove flit from Input VC
      flit *t_flit = m_input_unit[registry[i]->inport]->getTopFlit(registry[i]->vc);
      
      //set outport : get the outport latched at router, not from the VC as
      // this will change while doing the move when u push the new flit
      int outport = registry[i]->outport;
      assert(outport != -1);
      t_flit->set_outport(outport);
      
      //reserve the outport for this flit in this cycle
      outport_reservations[outport] = true;

      //set vc to vc frozen at downstream router
      assert(registry[i]->vc_at_downstream_router != -1);
      t_flit->set_vc(registry[i]->vc_at_downstream_router);

      //set part_of_move bit in the flit. required for assertion checks
      // at the downstream router
      t_flit->set_part_of_move();

      // credit management is done when the tail is sent out.
      // don't do credit management per flit
      
      t_flit->advance_stage(ST_, m_router->curCycle());
      m_router->grant_switch(registry[i]->inport, t_flit);

      //update the move_count for this flit
      registry[i]->cur_move_count++;

      if ( (t_flit->get_type() == TAIL_) || (t_flit->get_type() == HEAD_TAIL_) )
	{
	  //all flits of the packet have been moved.
	  //stop moving more flits.
	  //do the credit managment: set credit equal to cur_move_count 
	  //since u sent these many packets downstream
	  
	  registry[i]->tail_moved = true;
	  m_output_unit[outport]->decrement_credit_by(registry[i]->cur_move_count, registry[i]->vc_at_downstream_router);
	}

      cout<<"Inside eff. move: Router "<<m_router->get_id()<<" moved flit of type:"<<t_flit->get_type()<<" from vc:"
	  <<registry[i]->vc<<" in cycle "<<m_router->curCycle()<<" to outport: "
	  <<m_router->getOutportDirection(outport)<<" inport: "<<m_router->getInportDirection(registry[i]->inport)<<endl<<flush;
    }
  

  int inport = registry[0]->inport;
  int vc = registry[0]->vc;
  
  //get the max number of cycles the move is supposed to be carried out
  // +1 req. so that the "move" msg. gets sent out in the next cycle.
  int max_move = m_output_unit[inport]->get_max_credit_count(vc) + 1;
  cur_move_count++;


  if(cur_move_count == max_move)
    {      
      cur_move_count = 0;
      
      if(m_router->get_counter_state() == s_forward_progress)
	{
	  num_spin_cycles += max_move;
	  
	  if(max_move > max_spin_cycles)
	    max_spin_cycles = max_move;
	}
      
      m_router->move_complete();
    }
  else
    {
      m_router->schedule_wakeup(Cycles(1));
    }
}

void SwitchAllocator::clear_reservations()
{
  for (int i = 0; i < m_num_outports; i++)
    {
      outport_reservations[i] = false;
    }
}

void  SwitchAllocator::reset_stats()
{
  num_spin_cycles = 0;
  max_spin_cycles = 0;
  probes_dropped = 0;
  kill_move_dropped = 0;
}
