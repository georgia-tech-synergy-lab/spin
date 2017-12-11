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


#include "mem/ruby/network/garnet2.0/NetworkLink.hh"

#include "mem/ruby/network/garnet2.0/CreditLink.hh"

NetworkLink::NetworkLink(const Params *p)
    : ClockedObject(p), Consumer(this), m_id(p->link_id),
      m_type(NUM_LINK_TYPES_),
      m_latency(p->link_latency),
      linkBuffer(new flitBuffer()), link_consumer(nullptr),
      link_srcQueue(nullptr), m_link_utilized(0),
      m_vc_load(p->vcs_per_vnet * p->virt_nets)
{
  //spin scheme
  reset_spin_scheme_stats();
}

NetworkLink::~NetworkLink()
{
    delete linkBuffer;
}

void
NetworkLink::setLinkConsumer(Consumer *consumer)
{
    link_consumer = consumer;
}

void
NetworkLink::setSourceQueue(flitBuffer *srcQueue)
{
    link_srcQueue = srcQueue;
}

void
NetworkLink::wakeup()
{
    while (link_srcQueue->isReady(curCycle())) {
        flit *t_flit = link_srcQueue->getTopFlit();

	//spin scheme 
	switch(t_flit->get_type())
	  {
	  case PROBE_: probe_link_utilisation++;
	    t_flit->add_delay(m_latency);
	    break;
	    
	  case MOVE_: move_link_utilisation++;
	    t_flit->sub_delay(m_latency);
	    break;
	    
	  case KILL_MOVE_: kill_move_utilisation++;
	    break;

	  case CHECK_PROBE_ : check_probe_link_utilisation++;
	    t_flit->sub_delay(m_latency);
	    break;
	    
	  default: flit_link_utilisation++;
	  }

        t_flit->set_time(curCycle() + m_latency);
        linkBuffer->insert(t_flit);
        link_consumer->scheduleEventAbsolute(clockEdge(m_latency));
        m_link_utilized++;

	if( (t_flit->get_type() == HEAD_) || (t_flit->get_type() == BODY_) || (t_flit->get_type() == TAIL_) || 
	    (t_flit->get_type() == HEAD_TAIL_) )
	  m_vc_load[t_flit->get_vc()]++;
    }
}

NetworkLink *
NetworkLinkParams::create()
{
    return new NetworkLink(this);
}

CreditLink *
CreditLinkParams::create()
{
    return new CreditLink(this);
}

uint32_t
NetworkLink::functionalWrite(Packet *pkt)
{
    return linkBuffer->functionalWrite(pkt);
}

//spin scheme
void NetworkLink::reset_spin_scheme_stats()
{
  probe_link_utilisation = 0;
  move_link_utilisation = 0;
  kill_move_utilisation = 0;
  flit_link_utilisation = 0;
  check_probe_link_utilisation = 0;
}
