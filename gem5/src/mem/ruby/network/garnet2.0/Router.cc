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


#include "mem/ruby/network/garnet2.0/Router.hh"

#include "base/stl_helpers.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/garnet2.0/CreditLink.hh"
#include "mem/ruby/network/garnet2.0/CrossbarSwitch.hh"
#include "mem/ruby/network/garnet2.0/GarnetNetwork.hh"
#include "mem/ruby/network/garnet2.0/InputUnit.hh"
#include "mem/ruby/network/garnet2.0/NetworkLink.hh"
#include "mem/ruby/network/garnet2.0/OutputUnit.hh"
#include "mem/ruby/network/garnet2.0/RoutingUnit.hh"
#include "mem/ruby/network/garnet2.0/SwitchAllocator.hh"

using namespace std;
using m5::stl_helpers::deletePointers;

Router::Router(const Params *p)
    : BasicRouter(p), Consumer(this)
{
    m_latency = p->latency;
    m_virtual_networks = p->virt_nets;
    m_vc_per_vnet = p->vcs_per_vnet;
    m_num_vcs = m_virtual_networks * m_vc_per_vnet;

    m_routing_unit = new RoutingUnit(this);
    m_sw_alloc = new SwitchAllocator(this);
    m_switch = new CrossbarSwitch(this);

    m_input_unit.clear();
    m_output_unit.clear();

    //spin scheme 
    init_spin_scheme_ptr();
}

Router::~Router()
{
    deletePointers(m_input_unit);
    deletePointers(m_output_unit);
    delete m_routing_unit;
    delete m_sw_alloc;
    delete m_switch;

    //spin scheme
    delete m_counter;
    delete m_path_buffer;
    delete m_source_id_buffer;
    delete probeQueue;
    delete moveQueue;
    delete kill_moveQueue;
}

void
Router::init()
{
    BasicRouter::init();

    m_sw_alloc->init();
    m_switch->init();

    //std::cout<<" num outports at router "<<m_id<<": "<<get_num_outports()<<endl<<std::flush;

    //spin scheme
    m_move = false;
    m_is_sb_node = false;
    //invalidate_move_vc_at_downstream_router();
    reset_start_move();
    reset_spin_stats();
    //invalidate_move_outport();
    move_registry.clear();

    for (int inport = 0; inport < m_input_unit.size(); inport++) 
      {
        m_input_unit[inport]->init_spin_scheme();
      }


    //spin scheme : sb placement

    if(m_network_ptr->is_sb_placement_enabled())
      {
	int num_rows=m_network_ptr->getNumRows();
	int num_cols=m_network_ptr->getNumCols();
    
	int my_x=m_id%num_cols;
	int my_y=m_id/num_cols;
	
	int max_y=num_rows-1;
	int max_x=num_cols-1;
	
	int my_sum=my_x+my_y;
	int max_sum=max_x+max_y;
	
	bool alternate=false;
	
	if((my_x!=0)&&(my_y!=max_y))
	  {
	    for(int i=max_y;i>0;i=i-2)
	      {
		if(my_sum==i)
		  {
		    if(alternate)
		      {
			if(my_x%2!=0)
			  {
			    m_is_sb_node=true;
			    // cout<<"Router "<<m_id<<" is a sb node\n"<<flush;
			    break;
			  }
		      }
		    else
		      {
			m_is_sb_node=true;
			//cout<<"Router "<<m_id<<" is a sb node\n"<<flush;
			break;
		      }
		  }
		alternate= !alternate;
	      }
	    
	    alternate=false;
	    
	    for(int i=max_y;i<max_sum;i=i+2)
	      {
		if(my_sum==i)
		  {
		    if(alternate)
		      {
			if(my_x%2!=0)
			  {
			    m_is_sb_node=true;
			    //cout<<"Router "<<m_id<<" is a sb node\n"<<flush;
			    break;
			  }
		      }
		    else
		      {
			m_is_sb_node=true;
			//cout<<"Router "<<m_id<<" is a sb node\n"<<flush;
			break;
		      }
		  }
		alternate= !alternate;
	      }
	  }
	
      }

    //debug
    if( (m_id == 59) || (m_id == 37) || (m_id == 34) || (m_id == 38) || (m_id == 30) || (m_id == 29) || 
	(m_id == 28) || (m_id == 27) || (m_id == 26) )
      schedule_wakeup(Cycles(798000));
    //end debug
}

void
Router::wakeup()
{
  //if(m_id == 43)
  //std::cout<<" router:"<<m_id<<" counter_state:"<<get_counter_state()<<" cycle: "<<curCycle()<<endl<<flush;

    DPRINTF(RubyNetwork, "Router %d woke up\n", m_id);


    //SPIN scheme : output_unit credit processing loop has been moved 
    //              before flit processing loop, to solve the corner case of a 
    //              two router u-turn based deadlock.

    // check for incoming credits
    // Note: the credit update is happening before SA
    // buffer turnaround time =
    //     credit traversal (1-cycle) + SA (1-cycle) + Link Traversal (1-cycle)
    // if we want the credit update to take place after SA, this loop should
    // be moved after the SA request
    for (int outport = 0; outport < m_output_unit.size(); outport++) {
        m_output_unit[outport]->wakeup();
    }

    // check for incoming flits
    for (int inport = 0; inport < m_input_unit.size(); inport++) {
        m_input_unit[inport]->wakeup();
    }


    //spin scheme
    if(spin_scheme_enabled())
      check_counter_timeout(); //check for timeouts

    // Switch Allocation
    m_sw_alloc->wakeup();

    // Switch Traversal
    m_switch->wakeup();

    //spin scheme : reset the "kill_move_processed_this_cycle" bit
    if(spin_scheme_enabled())
      reset_kill_move_processed_this_cycle();
}

void
Router::addInPort(PortDirection inport_dirn,
                  NetworkLink *in_link, CreditLink *credit_link)
{
    int port_num = m_input_unit.size();
    InputUnit *input_unit = new InputUnit(port_num, inport_dirn, this);

    input_unit->set_in_link(in_link);
    input_unit->set_credit_link(credit_link);
    in_link->setLinkConsumer(this);
    credit_link->setSourceQueue(input_unit->getCreditQueue());

    m_input_unit.push_back(input_unit);

    m_routing_unit->addInDirection(inport_dirn, port_num);
}

void
Router::addOutPort(PortDirection outport_dirn,
                   NetworkLink *out_link,
                   const NetDest& routing_table_entry, int link_weight,
                   CreditLink *credit_link)
{
    int port_num = m_output_unit.size();
    OutputUnit *output_unit = new OutputUnit(port_num, outport_dirn, this);

    output_unit->set_out_link(out_link);
    output_unit->set_credit_link(credit_link);
    credit_link->setLinkConsumer(this);
    out_link->setSourceQueue(output_unit->getOutQueue());

    m_output_unit.push_back(output_unit);

    m_routing_unit->addRoute(routing_table_entry);
    m_routing_unit->addWeight(link_weight);
    m_routing_unit->addOutDirection(outport_dirn, port_num);
}

PortDirection
Router::getOutportDirection(int outport)
{
    return m_output_unit[outport]->get_direction();
}

PortDirection
Router::getInportDirection(int inport)
{
    return m_input_unit[inport]->get_direction();
}

int
Router::route_compute(RouteInfo *route, int inport, PortDirection inport_dirn, int invc)
{
  return m_routing_unit->outportCompute(route, inport, inport_dirn, invc);
}

void
Router::grant_switch(int inport, flit *t_flit)
{
    m_switch->update_sw_winner(inport, t_flit);
}

bool
Router::has_free_vc(int outport, int vnet)
{
    return m_output_unit[outport]->has_free_vc(vnet);
}

void
Router::schedule_wakeup(Cycles time)
{
    // wake up after time cycles
    scheduleEvent(time);
}

std::string
Router::getPortDirectionName(PortDirection direction)
{
    // PortDirection is actually a string
    // If not, then this function should add a switch
    // statement to convert direction to a string
    // that can be printed out
    return direction;
}

void
Router::regStats()
{
    BasicRouter::regStats();

    m_buffer_reads
        .name(name() + ".buffer_reads")
        .flags(Stats::nozero)
    ;

    m_buffer_writes
        .name(name() + ".buffer_writes")
        .flags(Stats::nozero)
    ;

    m_crossbar_activity
        .name(name() + ".crossbar_activity")
        .flags(Stats::nozero)
    ;

    m_sw_input_arbiter_activity
        .name(name() + ".sw_input_arbiter_activity")
        .flags(Stats::nozero)
    ;

    m_sw_output_arbiter_activity
        .name(name() + ".sw_output_arbiter_activity")
        .flags(Stats::nozero)
    ;


    //spin scheme
    m_num_probes_sent
      .name(name() + ".num_probes_sent")
      ;

    m_num_move_sent
      .name(name() + ".num_move_sent")
      ;

    m_num_kill_move_sent
      .name(name() + ".num_kill_move_sent")
      ;

    m_num_check_probe_sent
      .name(name() + ".num_check_probe_sent")
      ;

    m_num_probes_dropped
      .name(name() + ".num_probes_dropped")
      ;

    m_num_move_dropped
      .name(name() + ".num_move_dropped")
      ;

    m_num_kill_move_dropped
      .name(name() + ".num_kill_move_dropped")
      ;

    m_num_check_probe_dropped
      .name(name() + ".num_check_probe_dropped")
      ;

    m_num_spins
      .name(name() + ".num_spins")
      ;

    m_num_total_spin_cycles
      .name(name() + ".num_spin_cycles")
      ;

    m_max_spin_cycles
      .name(name() + ".max_spin_cycles")
      ;

    m_max_deadlock_path_length
      .name(name() + ".max_deadlock_path_length")
      ;

    m_deadlock_path_length_sum
      .name(name() + ".deadlock_path_length_sum")
      ;
}

void
Router::collateStats()
{
    for (int j = 0; j < m_virtual_networks; j++) {
        for (int i = 0; i < m_input_unit.size(); i++) {
            m_buffer_reads += m_input_unit[i]->get_buf_read_activity(j);
            m_buffer_writes += m_input_unit[i]->get_buf_write_activity(j);
        }
    }

    m_sw_input_arbiter_activity = m_sw_alloc->get_input_arbiter_activity();
    m_sw_output_arbiter_activity = m_sw_alloc->get_output_arbiter_activity();
    m_crossbar_activity = m_switch->get_crossbar_activity();

    //spin scheme stats
    m_num_probes_sent = num_probes_sent;
    m_num_move_sent = num_move_sent;
    m_num_kill_move_sent = num_kill_move_sent;
    m_num_check_probe_sent = num_check_probe_sent;

    m_num_probes_dropped = get_num_probes_dropped();
    m_num_move_dropped = get_num_move_dropped();
    m_num_kill_move_dropped = get_num_kill_move_dropped();
    m_num_check_probe_dropped = get_num_check_probe_dropped();

    m_num_spins = num_spins;
    m_num_total_spin_cycles = get_num_spin_cycles();
    m_max_spin_cycles = get_max_spin_cycles();

    m_max_deadlock_path_length = max_deadlock_path_length;
    m_deadlock_path_length_sum = deadlock_path_length_sum;
}

void
Router::resetStats()
{
    for (int j = 0; j < m_virtual_networks; j++) {
        for (int i = 0; i < m_input_unit.size(); i++) {
            m_input_unit[i]->resetStats();
        }
    }
}

void
Router::printFaultVector(ostream& out)
{
    int temperature_celcius = BASELINE_TEMPERATURE_CELCIUS;
    int num_fault_types = m_network_ptr->fault_model->number_of_fault_types;
    float fault_vector[num_fault_types];
    get_fault_vector(temperature_celcius, fault_vector);
    out << "Router-" << m_id << " fault vector: " << endl;
    for (int fault_type_index = 0; fault_type_index < num_fault_types;
         fault_type_index++) {
        out << " - probability of (";
        out <<
        m_network_ptr->fault_model->fault_type_to_string(fault_type_index);
        out << ") = ";
        out << fault_vector[fault_type_index] << endl;
    }
}

void
Router::printAggregateFaultProbability(std::ostream& out)
{
    int temperature_celcius = BASELINE_TEMPERATURE_CELCIUS;
    float aggregate_fault_prob;
    get_aggregate_fault_probability(temperature_celcius,
                                    &aggregate_fault_prob);
    out << "Router-" << m_id << " fault probability: ";
    out << aggregate_fault_prob << endl;
}

uint32_t
Router::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;
    num_functional_writes += m_switch->functionalWrite(pkt);

    for (uint32_t i = 0; i < m_input_unit.size(); i++) {
        num_functional_writes += m_input_unit[i]->functionalWrite(pkt);
    }

    for (uint32_t i = 0; i < m_output_unit.size(); i++) {
        num_functional_writes += m_output_unit[i]->functionalWrite(pkt);
    }

    return num_functional_writes;
}

Router *
GarnetRouterParams::create()
{
    return new Router(this);
}

//spin scheme

/*counter* Router::create_counter()
{
  counter *cntr = new counter;
  cntr->count = 0;
  cntr->thresh = Cycles(0);
  cntr->state = s_off;
  
  cntr->cptr = new pointer;
  
  return cntr; 
}*/

 /*Router::path_buffer* Router::create_path_buffer()
{
  path_buffer *pbuf = new path_buffer;
  pbuf->path.clear();
  pbuf->valid = false;

  return pbuf;
  }*/

  /*Router::source_id_buffer* Router::create_source_id_buffer()
{
  source_id_buffer *sibuf = new source_id_buffer;
  sibuf->valid = false;
  sibuf->source_id = 0;

  return sibuf;
  }*/

void Router::set_counter(unsigned input_port, unsigned vc, Counter_state state, unsigned thresh)
{
  m_counter->cptr->input_port = input_port;

  unsigned vnet = vc/m_vc_per_vnet;

  m_counter->cptr->vnet = vnet;
  m_counter->cptr->vc = vc;
  m_counter->state = state;
  m_counter->count = 0;
  
  switch(state)
    {
    case s_move : m_counter->thresh = curCycle() + get_loop_delay();
      break;

    case s_check_probe : m_counter->thresh = curCycle() + get_loop_delay();
      break;

    case s_frozen : m_counter->thresh = curCycle() + Cycles(thresh);
      std::cout<<"router "<<m_id<<" set to state frozen, thresh="<<m_counter->thresh
	       <<" in cycle "<<curCycle()<<endl<<std::flush;
      break;

    case s_deadlock_detection : m_counter->thresh = curCycle() + Cycles(get_my_dd_thresh());
      break;

    case s_forward_progress : 
      m_counter->thresh = curCycle() + get_loop_delay();
      num_spins++;
      deadlock_path_length_sum += get_path_buffer_length();
      if(max_deadlock_path_length < get_path_buffer_length())
	max_deadlock_path_length = get_path_buffer_length();
      break;

    default : m_counter->thresh = Cycles(INFINITE_);
    }

  if(state != s_off)
    {
      assert( (m_counter->thresh - curCycle()) > 0);
      schedule_wakeup(Cycles(m_counter->thresh - curCycle()));
    }
    
}

void Router::increment_counter_ptr()
{

  //don't point to a VC waiting for ejection or is IDLE

  unsigned cur_inp_port = m_counter->cptr->input_port;
  unsigned cur_vc = m_counter->cptr->vc;
  
  //assert(getInportDirection(cur_inp_port) != "Local");

  for(unsigned i=cur_vc+1; i<m_num_vcs; i++)
    {
      int t_outport = m_input_unit[cur_inp_port]->get_outport(i);

      if( (m_input_unit[cur_inp_port]->get_vc_state(i) == ACTIVE_) && 
	  (getOutportDirection(t_outport) != "Local") )
	{
	  set_counter(cur_inp_port, i, s_deadlock_detection, 0);
	  return;
	}
    }

  for(unsigned i=cur_inp_port+1; i<m_input_unit.size(); i++)
    {
      if(getInportDirection(i) == "Local")
	continue;

      for(unsigned j=0; j<m_num_vcs; j++)
	{
	  int t_outport = m_input_unit[i]->get_outport(j);

	  if( (m_input_unit[i]->get_vc_state(j) == ACTIVE_) &&
	      (getOutportDirection(t_outport) != "Local") )
	    {
	      set_counter(i, j, s_deadlock_detection, 0);
	      return;
	    }
	}
    }

  for(unsigned i=0; i<cur_inp_port; i++)
    {
      if(getInportDirection(i) == "Local")
	continue;

      for(unsigned j=0; j<m_num_vcs; j++)
	{
	  int t_outport = m_input_unit[i]->get_outport(j);

	  if( (m_input_unit[i]->get_vc_state(j) == ACTIVE_) &&
	      (getOutportDirection(t_outport) != "Local") )
	    {
	      set_counter(i, j, s_deadlock_detection, 0);
	      return;
	    }
	}
    }

  for(unsigned i=0; i<=cur_vc; i++)
    {
      int t_outport = m_input_unit[cur_inp_port]->get_outport(i);

      if( (m_input_unit[cur_inp_port]->get_vc_state(i) == ACTIVE_) &&
	  (getOutportDirection(t_outport) != "Local") )
	{
	  set_counter(cur_inp_port, i, s_deadlock_detection, 0);
	  return;
	}
    }

  set_counter(cur_inp_port, cur_vc, s_off, 0);
  
}


bool Router::check_counter_ptr(unsigned inport, unsigned invc)
{
  return ((inport == m_counter->cptr->input_port) && (invc == m_counter->cptr->vc));
}

void Router::send_probe()
{
  //|| ( ( (m_id==46) || (m_id==49) ) && (num_probes_sent>10) )

  if( ( sb_placement_enabled() && (!is_sb_node()) )  )
    return;

  int inport = m_counter->cptr->input_port;
  int vc = m_counter->cptr->vc;

  assert( (inport >=0) && (inport<m_input_unit.size()) );

  int outport = m_input_unit[inport]->get_outport(vc);
  int vnet = vc/m_vc_per_vnet;

  // the outport of this router is buffered in the probe. This is done automatically in the constructor
  // for probe.

  flit *probe = new flit(get_id(), inport, vc, outport, vnet, PROBE_, curCycle() + m_latency - Cycles(1), get_path_buffer_path(), m_network_ptr);
  probe->add_delay(m_latency);
  probeQueue->insert(probe);

  if(m_latency > 1)
    schedule_wakeup(Cycles(m_latency - Cycles(1)));

  num_probes_sent++;


  //  if(m_id == 37)
  // std::cout<<" router:"<<m_id<<" send probe for outport: "<<getOutportDirection(outport)
  //	     <<" in cycle: "<<curCycle()<<" inport: "<<getInportDirection(inport)<<endl<<flush;
}

void Router::fork_probes(flit *t_flit, const std::vector<bool> &fork_vector)
{
  bool at_least_one = false;

  for(int i=0; i<fork_vector.size(); i++)
    {
      if(fork_vector[i])
	{
	  flit *probe = new flit(t_flit, curCycle() + m_latency - Cycles(1), i);

	  //add delay of this router
	  probe->add_delay(m_latency);	  
	  probeQueue->insert(probe);

	  at_least_one = true;
	}
    }
  
  if( at_least_one && (m_latency > 1) )
    schedule_wakeup(Cycles(m_latency - Cycles(1)));
}

int Router::send_move_msg(int inport, int vc)
{
  int vnet = vc/m_vc_per_vnet;

  // get outport from the path buffer. this is done automatically in the flit constructor
  // for move, kill_move n check_probe msgs. So just pass -1 as output port here.

  int outport = -1;

  flit *move = new flit(get_id(), inport, vc, outport, vnet, MOVE_, 
			curCycle() + m_latency - Cycles(1), get_path_buffer_path(), get_net_ptr());

  //move carries 2*loop_delay
  move->add_delay(get_loop_delay());
  move->add_delay(get_loop_delay());

  std::cout<<"router "<<m_id<<" trying to send a move in cycle:"<<curCycle()<<", loop delay is "
	   <<get_loop_delay()<<endl<<std::flush;

  //subtract the delay of current router
  move->sub_delay(m_latency);

  assert(move->get_outport() != -1);

  moveQueue->insert(move);

  if(m_latency > 1)
    schedule_wakeup(Cycles(m_latency - Cycles(1)));

  num_move_sent++;

  //  set_move_outport(peek_path_top());

  //if(m_id == 28)
  //std::cout<<"router "<<m_id<<" sent a move in cycle:"<<curCycle()<<endl;

  return move->get_id();
}

void Router::send_check_probe(int inport, int vc)
{
  int vnet = vc/m_vc_per_vnet;
  
  // get outport from the path buffer. this is done automatically in the flit constructor
  // for move, kill_move n check_probe msgs. So just pass -1 as output port here.

  int outport = -1;

  flit *check_probe = new flit(get_id(), inport, vc, outport, vnet, CHECK_PROBE_, 
			       curCycle() + m_latency - Cycles(1), get_path_buffer_path(), get_net_ptr());

  //check_probe carries 2*loop_delay
  check_probe->add_delay(get_loop_delay());
  check_probe->add_delay(get_loop_delay());

  //subtract the delay of current router
  check_probe->sub_delay(m_latency);

  assert(check_probe->get_outport() != -1);

  check_probeQueue->insert(check_probe);

  if(m_latency > 1)
    schedule_wakeup(Cycles(m_latency - Cycles(1)));

  num_check_probe_sent++;

  std::cout<<"router "<<m_id<<" sent a check_probe in cycle:"<<curCycle()<<endl;

}

void Router::forward_move(flit *move)
{
  //subtract delay
  move->sub_delay(m_latency);
  int outport = move->get_path_top();
  //  set_move_outport(outport);
  move->set_outport(outport);
  move->set_time(curCycle() + m_latency - Cycles(1));

  moveQueue->insert(move);

  if(m_latency > 1)
    schedule_wakeup(Cycles(m_latency - Cycles(1)));
}

void Router::forward_check_probe(flit *check_probe)
{
  //subtract delay
  check_probe->sub_delay(m_latency);
  int outport = check_probe->get_path_top();
  
  check_probe->set_outport(outport);
  check_probe->set_time(curCycle() + m_latency - Cycles(1));
  
  check_probeQueue->insert(check_probe);
  
  if(m_latency > 1)
    schedule_wakeup(Cycles(m_latency - Cycles(1)));
}

void Router::send_kill_move(int inport)
{
  flit *kill_move = new flit(get_id(), get_path_buffer_path(), curCycle() + m_latency - Cycles(1), inport);

  //if(m_id == 43)
  //{
  std::cout<<" router "<<m_id<<" sent kill_move in cycle:"<<curCycle()<<endl<<std::flush;
      //}

  //set kill_move processed this cycle, so that no new move msg. get processed this cycle by any Inputunit
  set_kill_move_processed_this_cycle();
  
  //make this a must send flit
  kill_move->set_must_send();

  kill_moveQueue->insert(kill_move);

  if(m_latency > 1)
    schedule_wakeup(Cycles(m_latency - Cycles(1)));

  num_kill_move_sent++;
}

void Router::forward_kill_move(flit *kill_move)
{
  int outport = kill_move->get_path_top();
  kill_move->set_outport(outport);
  kill_move->set_time(curCycle() + m_latency - Cycles(1));
  
  kill_moveQueue->insert(kill_move);

  if(m_latency > 1)
    schedule_wakeup(Cycles(m_latency - Cycles(1)));
}

void Router::check_counter_timeout()
{
  if( (curCycle() < m_counter->thresh) || (m_counter->state == s_off) )
    return;

  switch(m_counter->state)
    {
    case s_deadlock_detection: 
      send_probe();
      increment_counter_ptr();
      break;

    case s_move:
      send_kill_move(m_counter->cptr->input_port);
      invalidate_path_buffer();
      invalidate_source_id_buffer();
      //invalidate_move_vc_at_downstream_router();
      //invalidate_move_outport();
      clear_move_registry();
      increment_counter_ptr();
      //if(m_id == 28)
      //std::cout<<" counter expired in move state at router 28 in cycle: "<<curCycle()<<endl<<std::flush;
      break;

    case s_frozen:
      if(get_move_bit())
	{
	  std::cout<<" counter expired in frozen state at router "<<m_id<<" in cycle: "<<curCycle()<<endl<<std::flush;
	  set_start_move();
	}
      break;

    case s_forward_progress:
      if(get_move_bit())
	{
	  set_start_move();
	}
      break;

    case s_check_probe:
      send_kill_move(m_counter->cptr->input_port);
      invalidate_path_buffer();
      invalidate_source_id_buffer();
      clear_move_registry();
      increment_counter_ptr();
      //if(m_id == 28)
      //std::cout<<" counter expired in check_probe state at router 28 in cycle: "<<curCycle()<<endl<<std::flush;
      break;

    default: assert(false);
    }
}

void Router::invalidate_path_buffer()
{
  m_path_buffer->valid = false;

  while(!m_path_buffer->path.empty())
    m_path_buffer->path.pop();
}

bool Router::check_source_id_buffer(int source_id, int move_id)
{
  if(!m_source_id_buffer->valid)
    return false;

  return ( (m_source_id_buffer->source_id == source_id) && (m_source_id_buffer->move_id == move_id) );
}

bool Router::partial_check_source_id_buffer(int source_id)
{
  if(!m_source_id_buffer->valid)
    return false;

  return (m_source_id_buffer->source_id == source_id);
}

/*bool Router::is_vc_frozen(int inport, int vc)
{
  if( !((get_counter_state() == s_frozen) || (get_counter_state() == s_forward_progress)) )
    return false;

  return check_counter_ptr(inport, vc);
  }*/

 /*void Router::set_move_vc_at_downstream_router(int vc) 
{ 
  //there should be only one OutputUnit setting this

  if(move_vc_at_downstream_router != -1)
    {
      int outport = m_input_unit[m_counter->cptr->input_port]->get_outport(m_counter->cptr->vc);

      cout<<" move vc assetion failure at router: "<<m_id
	  <<" counter state: "<<get_counter_state()<<" for outport: "<<getOutportDirection(outport)<<endl;
    }
  assert(move_vc_at_downstream_router == -1);

  move_vc_at_downstream_router = vc; 
  }*/

void Router::move_complete()
{
  //reset start_move bit
  //change counter state to move
  //send out move msg. : do dependency check and setting of move bit
  //after "move" msg. is received.
  //set the source-id buffer

  reset_start_move();
  reset_move_bit();

  if(get_counter_state() == s_forward_progress)
    {
      //there should only be one entry in the move registry
      //the counter pointer should be the same as the move registry entry
      assert(move_registry.size() == 1);
      assert((move_registry[0]->inport == m_counter->cptr->input_port));
      assert((move_registry[0]->vc == m_counter->cptr->vc));

      //source-id buffer at source should already be set
      assert(partial_check_source_id_buffer(m_id));

      //send check_probe and change counter state to s_check_probe
      send_check_probe(m_counter->cptr->input_port, m_counter->cptr->vc);
      set_counter(m_counter->cptr->input_port, m_counter->cptr->vc, s_check_probe, 0);
      clear_move_registry();
      create_move_info_entry(m_counter->cptr->input_port, m_counter->cptr->vc, peek_path_top());
    }
  else
    {
      invalidate_move_vcs();
    }
}

void Router::reset_spin_stats()
{
  num_spins = 0;
  num_probes_sent = 0;
  num_move_sent = 0;
  num_kill_move_sent = 0;
  deadlock_path_length_sum = 0;
  max_deadlock_path_length = 0;
  num_check_probe_sent = 0;
}

int Router::get_path_buffer_length()
{
  assert(m_path_buffer->valid);

  return m_path_buffer->path.size();
}
unsigned Router::get_num_probes_dropped()
{
  unsigned num_probes_dropped = 0;

  //get num_probes_dropped from all input units
  for (int i = 0; i < m_input_unit.size(); i++)
    {
      num_probes_dropped += m_input_unit[i]->get_num_probes_dropped();
    }

  //get it from switch allocator too
  num_probes_dropped += m_sw_alloc->get_num_probes_dropped();

  return num_probes_dropped;
}

unsigned Router::get_num_move_dropped()
{
  unsigned num_move_dropped = 0;

  //only input unit drops move msgs.

  for (int i = 0; i < m_input_unit.size(); i++)
    {
      num_move_dropped += m_input_unit[i]->get_num_move_dropped();
    }

  return num_move_dropped;
}

unsigned Router::get_num_check_probe_dropped()
{
  unsigned num_check_probe_dropped = 0;

  //only input unit drops move msgs.

  for (int i = 0; i < m_input_unit.size(); i++)
    {
      num_check_probe_dropped += m_input_unit[i]->get_num_check_probe_dropped();
    }

  return num_check_probe_dropped;
}

unsigned Router::get_num_kill_move_dropped()
{
  return m_sw_alloc->get_num_kill_move_dropped();
}

unsigned Router::get_num_spin_cycles() 
{ 
  return m_sw_alloc->get_num_spin_cycles(); 
}
  
unsigned Router::get_max_spin_cycles() 
{ 
  return m_sw_alloc->get_max_spin_cycles(); 
}

void Router::init_spin_scheme_ptr()
{
  //create the counter
  m_counter = new counter;
  m_counter->count = 0;
  m_counter->thresh = Cycles(0);
  m_counter->state = s_off;
  m_counter->cptr = new pointer;

  //create the path buffer
  m_path_buffer = new path_buffer;

  while(!m_path_buffer->path.empty())
    m_path_buffer->path.pop();
  
  m_path_buffer->valid = false;

  //create source-id buffer
  m_source_id_buffer = new source_id_buffer;
  m_source_id_buffer->valid = false;
  m_source_id_buffer->source_id = 0;

  probeQueue = new flitBuffer();
  moveQueue = new flitBuffer();
  kill_moveQueue = new flitBuffer();
  check_probeQueue = new flitBuffer();
}

void Router::create_move_info_entry(int inport, int vc, int outport)
{
  move_info *mi = new move_info;
  mi->inport = inport;
  mi->vc = vc;
  mi->outport = outport;

  mi->vc_at_downstream_router = -1;
  mi->tail_moved = false;
  mi->cur_move_count = 0;

  move_registry.push_back(mi);

  //also freeze the vc
  m_input_unit[inport]->freeze_vc(vc);

  if(move_registry.size() == 2)
    std::cout<<" CROSS-OVER router in deadlock : "<<m_id<<endl<<std::flush; 
}

void Router::update_move_info_entry(int inport, int vc, int outport)
{
  bool found = false;

  for(int i=0; i<move_registry.size(); i++)
    {
      if(move_registry[i]->outport == outport)
	{
	  //found it
	  //there can be only one entry per outport
	  assert(move_registry[i]->inport == inport);

	  //un-freeze the vc in present entry
	  m_input_unit[inport]->un_freeze_vc(move_registry[i]->vc);

	  //debug
	  if(m_id==30)
	    std::cout<<" vc was updated at router "<<m_id<<" from "<<move_registry[i]->vc
		     <<" to "<<vc<<" in cycle "<<curCycle()<<endl;
	  //end debug

	  //update the vc entry
	  move_registry[i]->vc = vc;
	  
	  //freeze the new vc
	  m_input_unit[inport]->freeze_vc(vc);

	  if(found)
	    assert(false);
	  else
	    found = true;
	}
    }

  if(!found)
    assert(false);
}

void Router::clear_move_registry()
{
  int size = move_registry.size();

  for(int i=0; i<size; i++)
    {
      move_info *mi = move_registry[i];
      
      //also un-freeze the vc
      m_input_unit[mi->inport]->un_freeze_vc(mi->vc);

      delete mi;
    }

  move_registry.clear();
}

void Router::update_move_vc_at_downstream_router(int vc, int outport)
{
  bool found = false;
  for(int i=0; i<move_registry.size(); i++)
    {
      if(move_registry[i]->outport == outport)
	{
	  move_registry[i]->vc_at_downstream_router = vc;

	  if(found)
	    assert(false);
	  else
	    found = true;
	}
    }

  if(!found)
    assert(false);
}

void Router::invalidate_move_vcs()
{
  //invalidate the move vcs at downstream router
  //set tail_moved to false
  //chnage credit_used_count to 0
  for(int i=0; i<move_registry.size(); i++)
    {
      move_registry[i]->vc_at_downstream_router = -1;
      move_registry[i]->tail_moved = false;
      move_registry[i]->cur_move_count = 0;
    }
}

unsigned Router::get_my_dd_thresh()
{
  unsigned thresh = m_network_ptr->get_dd_thresh();
  
  if(m_network_ptr->is_variable_dd_enabled())
    return (thresh + 3*m_id);
  
  return thresh;
}

void Router::latch_source_id_buffer(int source_id, int move_id)
{
  m_source_id_buffer->source_id = source_id;
  m_source_id_buffer->move_id = move_id;
  m_source_id_buffer->valid = true;
}

void Router::invalidate_source_id_buffer()
{
  m_source_id_buffer->source_id = -1;
  m_source_id_buffer->move_id = -1;
  m_source_id_buffer->valid = false;
}

void Router::invalidate_move_registry_entry(int inport, int outport)
{
  bool found = false;
  int size = move_registry.size();

  int entry_num = -1;

  for(int i=0; i<size; i++)
    {
      if(move_registry[i]->outport == outport)
	{
	  entry_num = i;

	  if(found)
	    assert(false);
	  else
	    found = true;
	}
    }

  if(!found)
    assert(false);

  move_info *mi = move_registry[entry_num];

  for(int i=0; i<entry_num; i++)
    {
      move_registry[i] = move_registry[i+1];
    }

  for(int i=entry_num; i<size-1; i++)
    {
      move_registry[i] = move_registry[i+1];
    }

  //unfreeze the vc
  m_input_unit[mi->inport]->un_freeze_vc(mi->vc);
  delete mi;

  move_registry.pop_back();
}

bool Router::check_outport_entry_in_move_registry(int outport)
{
  for(int i=0; i<move_registry.size(); i++)
    {
      if(move_registry[i]->outport == outport)
	return true;
    }

  return false;
}

int Router::get_num_active_vcs(int outport, int vnet) 
{ 
  return m_output_unit[outport]->get_num_active_vcs(vnet); 
}

int Router::get_num_hops(int vnet, NetDest net_dest)
{
  return m_routing_unit->get_num_hops(vnet, net_dest);
}

//1-vc-per-vnet routing
Cycles Router::get_vc_active_time(int outport, int vc)
{
  return m_output_unit[outport]->get_vc_active_time(vc);
}
