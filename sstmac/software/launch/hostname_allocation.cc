/**
Copyright 2009-2022 National Technology and Engineering Solutions of Sandia,
LLC (NTESS).  Under the terms of Contract DE-NA-0003525, the U.S. Government
retains certain rights in this software.

Sandia National Laboratories is a multimission laboratory managed and operated
by National Technology and Engineering Solutions of Sandia, LLC., a wholly
owned subsidiary of Honeywell International, Inc., for the U.S. Department of
Energy's National Nuclear Security Administration under contract DE-NA0003525.

Copyright (c) 2009-2022, NTESS

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

#include <sstmac/backends/common/parallel_runtime.h>
#include <sstmac/software/launch/hostname_allocation.h>
#include <sstmac/software/launch/node_allocator.h>
#include <sstmac/common/sstmac_config.h>
#include <sstmac/hardware/interconnect/interconnect.h>
#include <sstmac/hardware/topology/cartesian_topology.h>
#include <sprockit/keyword_registration.h>
#include <sprockit/util.h>
#include <sprockit/fileio.h>
#include <sprockit/sim_parameters.h>
#include <sstream>

RegisterKeywords(
{ "hostfile", "a file containing a line-by-line list of hostnames for each node in system" },
);

namespace sstmac {
namespace sw {

HostnameAllocation::HostnameAllocation(SST::Params& params) :
  NodeAllocator(params)
{
  hostfile_ = params.find<std::string>("hostfile");
}

void
HostnameAllocation::readHostFile(
  ParallelRuntime*  /*rt*/,
  const char*  /*here*/,
  const std::string &hostfile,
  std::vector<std::string>& hosts)
{
  debug_printf(sprockit::dbg::allocation,
    "hostname_allocation: reading host file %s",
     hostfile.c_str());

  hosts.clear();
  std::ifstream in(hostfile);
  std::string name;
  while(in >> name)
    hosts.push_back(name);
}

bool
HostnameAllocation::allocate(int  /*nnode_requested*/,
 const ordered_node_set&  /*available*/,
 ordered_node_set &allocation) const
{
  std::vector< std::string > hosts;
  readHostFile(rt_, "hostname_allocation::allocate", hostfile_, hosts);

  if (!topology_) {
    spkt_throw_printf(sprockit::ValueError, "hostname_allocation::allocate: null topology");
  }

  for (auto& str : hosts){
    allocation.insert(topology_->nodeNameToId(str));
  }

  return true;
}

}
}
