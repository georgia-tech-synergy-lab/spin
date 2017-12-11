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


#include "mem/ruby/network/garnet2.0/flit.hh"
#include <iostream>

//static variable

GarnetNetwork *flit::m_net_ptr;

using namespace std;

//spin scheme
int flit::probe_id = 0;
int flit::move_id = 0;
int flit::kill_move_id = 0;
int flit::check_probe_id = 0;

// Constructor for the flit
flit::flit(int id, int  vc, int vnet, RouteInfo route, int size,
	   MsgPtr msg_ptr, Cycles curTime)
{
    m_size = size;
    m_msg_ptr = msg_ptr;
    m_enqueue_time = curTime;
    m_time = curTime;
    m_id = id;
    m_vnet = vnet;
    m_vc = vc;
    m_route = route;
    m_stage.first = I_;
    m_stage.second = m_time;

    if (size == 1) {
        m_type = HEAD_TAIL_;
        return;
    }
    if (id == 0)
        m_type = HEAD_;
    else if (id == (size - 1))
        m_type = TAIL_;
    else
        m_type = BODY_;

    //spin scheme
    part_of_move = false;
    must_send = false;

    // cout<<"hello from flit constructor 2\n"<<flush;
}

flit::flit(int source_id, int source_inp_port, int source_vc, int source_out_port, int vnet, flit_type type, 
	   Cycles curTime, const std::queue<int> &new_path, GarnetNetwork *net_ptr)
{
  //this constructor is used for creating probe n move msgs.

  this->source_id = source_id;
  this->source_inp_port = source_inp_port;
  this->source_vc = source_vc;

  m_time = curTime;
  m_enqueue_time = curTime;
  m_type = type;
  m_vnet = vnet;
  delay = Cycles(0);
  m_outport = source_out_port;
  inport = source_inp_port;
  m_net_ptr = net_ptr;

  while(!path.empty())
    path.pop();

  if(type == PROBE_)
    {
      path.push(source_out_port);
      m_id = probe_id++;
    }
  else if(type == MOVE_)
    {
      path = new_path;
      m_outport = get_path_top();
      m_id = move_id++;
    }
  else if(type == CHECK_PROBE_)
    {
      path = new_path;
      m_outport = get_path_top();
      m_id = check_probe_id++;      
    }

  must_send = false;
  part_of_move = false;

  // cout<<"hello from flit constructor 3\n"<<flush;
}

flit::flit(flit *t_flit, Cycles curTime, int outport)
{
  //this constructor is used for probe forking

  source_id = t_flit->get_source_id();

  assert(source_id < t_flit->get_net_ptr()->getNumRouters());

  source_inp_port = t_flit->get_source_inp_port();
  source_vc = t_flit->get_source_vc();

  m_time = curTime;
  m_enqueue_time = t_flit->get_enqueue_time();
  m_type = PROBE_;
  m_vnet = t_flit->get_vnet();
  m_net_ptr = t_flit->get_net_ptr();

  //preserve the delay accumulated thus far
  delay = t_flit->get_delay();

  m_outport = outport;

  //inport has been updated in the probe in InputUnit.cc
  inport = t_flit->get_inport();

  while(!path.empty())
    path.pop();

  //copy over the path thus far
  path = t_flit->get_path();
  
  //append the current outport to the path
  path.push(outport);

  must_send = false;
  part_of_move = false;
  
  m_id = t_flit->get_id();
  
  //cout<<"hello from flit constructor 4\n"<<flush;
}

flit::flit(int source_id, const std::queue<int> &new_path, Cycles curTime, int inport)
{
  //this constructor is used for creating kill_move msg
  this->source_id = source_id;

  m_time = curTime;
  m_enqueue_time = curTime;
  m_type = KILL_MOVE_;
  this->inport = inport;

  path = new_path;
  m_outport = get_path_top();

  must_send = false;
  part_of_move = false;

  m_id = kill_move_id++;

  //cout<<"hello from flit constructor 5\n"<<flush;
}

// Flit can be printed out for debugging purposes
void
flit::print(std::ostream& out) const
{
    out << "[flit:: ";
    out << "Id=" << m_id << " ";
    out << "Type=" << m_type << " ";
    out << "Vnet=" << m_vnet << " ";
    out << "VC=" << m_vc << " ";
    out << "Src NI=" << m_route.src_ni << " ";
    out << "Src Router=" << m_route.src_router << " ";
    out << "Dest NI=" << m_route.dest_ni << " ";
    out << "Dest Router=" << m_route.dest_router << " ";
    out << "Enqueue Time=" << m_enqueue_time << " ";
    out << "]";
}

bool
flit::functionalWrite(Packet *pkt)
{
    Message *msg = m_msg_ptr.get();
    return msg->functionalWrite(pkt);
}

//spin scheme
int flit::get_path_top() 
{
  int i = path.front();
  path.pop();
  return i;
}

bool flit::greater(flit* n1, flit* n2)
{
  //spin scheme
  if( (n1->get_type() == PROBE_) && (n2->get_type() == PROBE_) )
    {
      /* priority order : first time, then higher source_id 
       * if rotating priority is enabled, higher priority
       */
	
      if (n1->get_time() == n2->get_time()) 
	{
	  if(n1->get_source_id() == n2->get_source_id())
	    {
	      if(n1->get_outport() == n2->get_outport())
		{
		  return (n1->get_inport() < n2->get_inport());
		}
	      else
		{
		  return (n1->get_outport() < n2->get_outport());
		}
	    }
	  else
	    {
	      if(m_net_ptr->is_rotating_priority_enabled())
		{
		  /*
		    std::cout<<"function called from flit.cc \n"<<std::flush;

		    if(n1 != NULL)
		    std::cout<<"n1 is of type "<<<<std::flush;
		    
		  if(n2 == NULL)
		  std::cout<<"n2 is NULL\n"<<std::flush;*/
		  
		  return m_net_ptr->is_my_priority_greater
		    (n2->get_source_id(), n1->get_source_id());
		}
	      else
		{
		  return (n1->get_source_id() < n2->get_source_id());
		}
	    }
	}
      else 
	{
	  return (n1->get_time() > n2->get_time());
	}
    }
    
  //there can be only one move in the queue, so this should be enough
  if(n1->get_type() == MOVE_ && n2->get_type() == MOVE_)
    {
      return (n1->get_time() > n2->get_time());
    }
  
  if(n1->get_type() == KILL_MOVE_ && n2->get_type() == KILL_MOVE_)
    {
      //first time, then must_send, then source_id
      
      if (n1->get_time() == n2->get_time()) 
	{
	  //both can't be must send :there can be at max one must_send in the queue
	  assert( !(n1->get_must_send() && n2->get_must_send()) );
	  
	  if(n1->get_must_send())
	    return false;
	  
	  if(n2->get_must_send())
	    return true;
	  
	  //if neither is a must_send, then sort on basis of source_id
	  return (n1->get_source_id() < n2->get_source_id());
	} 
      else 
	{
	  return (n1->get_time() > n2->get_time());
	}
    }
  
    
  if (n1->get_time() == n2->get_time()) {
    //assert(n1->flit_id != n2->flit_id);
    return (n1->get_id() > n2->get_id());
  } else {
    return (n1->get_time() > n2->get_time());
  }
}
