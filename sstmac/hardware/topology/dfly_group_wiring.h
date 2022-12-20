/**
Copyright 2009-2018 National Technology and Engineering Solutions of Sandia, 
LLC (NTESS).  Under the terms of Contract DE-NA-0003525, the U.S.  Government 
retains certain rights in this software.

Sandia National Laboratories is a multimission laboratory managed and operated
by National Technology and Engineering Solutions of Sandia, LLC., a wholly 
owned subsidiary of Honeywell International, Inc., for the U.S. Department of 
Energy's National Nuclear Security Administration under contract DE-NA0003525.

Copyright (c) 2009-2018, NTESS

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

    * Neither the name of the copyright holder nor the names of its
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

#ifndef SSTMAC_HARDWARE_NETWORK_TOPOLOGY_dfly_group_wiring_H_INCLUDED
#define SSTMAC_HARDWARE_NETWORK_TOPOLOGY_dfly_group_wiring_H_INCLUDED

#include <sstmac/hardware/topology/cartesian_topology.h>

namespace sstmac {
namespace hw {

class InterGroupWiring {
 public:
  SPKT_DECLARE_BASE(InterGroupWiring)
  SPKT_DECLARE_CTOR(
    SST::Params&,
    int, /* a=num switches per group */
    int, /* g=num groups */
    int /* h=num group links per switch */)

  virtual ~InterGroupWiring() {}

  /**
   * @brief group_port
   * @param srcA
   * @param srcG
   * @param dstG
   * @return The port on which router (srcA, srcG) connects to group dstG
   */
  virtual int inputGroupPort(int srcA, int srcG, int srcH, int dstA, int dstG) const = 0;

  /**
   * @brief connected_routers
   * @param a The src router index within the group
   * @param g The src router group
   * @param connected [in-out] The routers (switch id) for each inter-group interconnection
   * @return The number of routers in connected array
   */
  virtual void connectedRouters(int a, int g, std::vector<int>& connected) const = 0;

  /**
   * @brief connected_to_group
   * @param srcG
   * @param dstG
   * @param connected [in-out] The list of all intra-group routers in range (0 ... a-1)
   *                  that have connections to a router in group dstG
   * @return The number of routers in group srcG with connections to dstG
   */
  virtual void connectedToGroup(int srcG, int dstG, std::vector<std::pair<int,int>>& connected) const = 0;

  virtual SwitchId randomIntermediate(Router* rtr, SwitchId current_sw, SwitchId dest_sw, uint32_t seed);

 protected:
  /**
   * @brief inter_group_wiring
   * @param params
   * @param a  The number of routers per group
   * @param g  The number of groups
   * @param h  The number of group links per router
   */
  InterGroupWiring(SST::Params& params, int a, int g, int h);

 protected:
  /** Number of routers per group */
  int a_;
  /** Number of groups */
  int g_;
  /** Number of group connections per router */
  int h_;
};

}
} //end of namespace sstmac

#endif
