/**
Copyright 2009-2017 National Technology and Engineering Solutions of Sandia, 
LLC (NTESS).  Under the terms of Contract DE-NA-0003525, the U.S.  Government 
retains certain rights in this software.

Sandia National Laboratories is a multimission laboratory managed and operated
by National Technology and Engineering Solutions of Sandia, LLC., a wholly 
owned subsidiary of Honeywell International, Inc., for the U.S. Department of 
Energy's National Nuclear Security Administration under contract DE-NA0003525.

Copyright (c) 2009-2017, NTESS

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

    * Neither the name of Sandia Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Questions? Contact sst-macro-help@sandia.gov
*/

#ifndef sumi_api_PING_H
#define sumi_api_PING_H

#include <sumi/monitor.h>
#include <sumi/transport_fwd.h>
#include <sumi/timeout.h>

#ifdef FEATURE_TAG_SUMI_RESILIENCE

namespace sumi {

/**
 * @class pinger
 * The pinger object exists as a sort of refcounted class
 * so that you're never sending out more than one ping.
 * Multiple ping requests to the same machine all get
 * funneled into the same ping.
 */
class pinger
{  
 public:
  ~pinger();

  pinger(transport* api, int dst, double timeout);

  void execute();

  /**
   * Notify the pinger that its ping has safely arrived
   */
  void arrived();

  /**
   * Send out the first ping and start things going
   */
  void start();

  /**
   * Create the event that waits on a successful ping
   */
  void wait();

  /**
   * @brief cancel
   * @param tag
   */
  void cancel(timeout_function* func);

  /**
   * The number of independent listeners, i.e.
   * collectives or functions depending on results
   * from this piong
   * @return
   */
  int refcount(){
    return functions_.refcount();
  }

  bool has_arrived() const {
    return arrived_;
  }

  bool has_failed() const {
    return failed_;
  }

  bool is_expired(double wtime) const {
    return (start_time_ + timeout_) < wtime;
  }

  double start_time() const {
    return start_time_;
  }

  void maybe_renew(double wtime);

  /**
   * Attach a new listener (timeout function) to this ping
   * @param func
   */
  void attach_listener(timeout_function* func);

 protected:
  void timeout_all_listeners();

  void schedule_next();

  void schedule_next(double wtime);

 protected:
  function_set functions_;

  transport* my_api_;
  int dst_;
  double timeout_;
  double start_time_;
  bool failed_;
  bool arrived_;

};

class ping_monitor :
    public activity_monitor
{
  FactoryRegister("ping", activity_monitor, ping_monitor)
 public:
  ping_monitor(sprockit::sim_parameters* params,
               transport* tport);

  void ping(int dst, timeout_function* func);

  void renew_pings(double wtime);

  void cancel_ping(int dst, timeout_function* func);

  void message_received(message* msg);

  void validate_done();

  void validate_all_pings();

 protected:
  /**
   * @brief pingers_
   * Map where key is destination (rank)
   * and value is a ping object.  Multiple
   * #timeout_function objects can be attached to a single pinger.
   * The pinger object exists as a sort of refcounted class
   * so that you're never sending out more than one ping.
   * Multiple ping requests to the same machine all get
   * funneled into the same ping.
   */
  std::map<int, pinger*> pingers_;

  double timeout_;
};


}

#endif

#endif // PING_H