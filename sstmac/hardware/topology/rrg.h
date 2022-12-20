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
// rrg.h: Implementation of Random Regular Graph
//
// Author: Md Atiqul Mollah <mollah@oakland.edu>
// Author: Md Nahid Newaz <mdnahidnewaz@oakland.edu>

#ifndef SSTMAC_HARDWARE_NETWORK_TOPOLOGY_RRG_H_INCLUDED
#define SSTMAC_HARDWARE_NETWORK_TOPOLOGY_RRG_H_INCLUDED

#include <sstmac/hardware/topology/structured_topology.h>

namespace sstmac {
namespace hw {

/* class rrg (random regular graph)

*/
class rrg :public StructuredTopology
{
	public:
	SPKT_REGISTER_DERIVED(
    Topology,
    rrg,
    "macro",
    "rrg",
    "Random Regular Graph topology")
  int xwe; 
  long long int x13=0;
  long long int y13=0;

  long long int* total_minimal_path()
   {
         long long int *x1=&x13;
         return x1;
   }
  long long int* total_non_minimal_path()
   {
         long long int *y1=&y13;
         return y1;
   }
 
  std::string toString() const override {
    return "rrg";
  }

  rrg(SST::Params& params);
  virtual ~rrg()
  {
      printf("Total Number of Global Minimal Path = %lld\n",x13);
      printf("Total Number of Global Non-Minimal Path = %lld\n",y13);

  }
  
  //.........................
   //for debuging 
     int x12=0;
     int y12=0;
     int* x()
     {
         int *x1=&x12;
         return x1;
     }
     int* y()
     {
         int *y1=&y12;
         return y1;
     }

  //.........................  
  /**** BEGIN PURE VIRTUAL INTERFACE *****/
  /**
     Structured topologies can be direct (torus) or indirect (fat tree).
     We therefore need to distinguish the total number of switches and
     the number of leaf switches - i.e. those directly connected to nodes.
     For direct topologies, num_switches and num_leaf_switches are the same.
     For indirect, num_leaf_switches < num_switches.
     @return The number of leaf switches directly connected to compute nodes
  */
  //number of switches that has compute node attached to it.
  SwitchId numLeafSwitches() const override{
	 return N_;
  }

  SwitchId numSwitches() const override{
         return N_;
  }  

  /**
     For a given input switch, return all nodes connected to it.
     This return vector might be empty if the
     switch is an internal switch not connected to any nodes
     @return The nodes connected to switch for ejection
  */
  void endpointsConnectedToInjectionSwitch(SwitchId swid,std::vector<InjectionPort>& nodes) const override;
  /**
   * @brief connected_outports
   *        Given a 3D torus e.g., the connection vector would contain
   *        6 entries, a +/-1 for each of 3 dimensions.
   * @param src   Get the source switch in the connection
   * @param conns The set of output connections with dst SwitchId
   *              and the port numbers for each connection
   */					   
  void connectedOutports(SwitchId src, std::vector<Connection>& conns) const override;
					   
  //this function returns the number of compute nodes per switch
  int concentration() const {
    return concentration_;
  }
  
  //this function returns total number of compute nodes in the system
  NodeId numNodes() const override {
    return concentration_ * numLeafSwitches();
  }

  /**
   * @brief Return the maximum number of ports on any switch in the network
   * @return 
  */
  int maxNumPorts() const override{
       return concentration_+r_;
  }

  int numHopsToNode(NodeId src, NodeId dst) const override{
        return 3;
  }


  //this function returns the switch id of a given compute node
   SwitchId endpointToSwitch(NodeId nid) const override {
      return nid / concentration_;
  }

  //this function returns total number of switches in the system
  SwitchId maxSwitchId() const override {
    return N_;
  }
  
  //this function returns total number of compute nodes in the system
  NodeId maxNodeId() const override {
    return numNodes();
  }

  //this function returns the diameter of the network
  int diameter() const override{
	  return diameter_;
  }
  
  /**** END PURE VIRTUAL INTERFACE *****/
  
  //this function returns 1 if two given switch are neighbour to each other otherwise returns 0
  int port_to_neighbor(const SwitchId src, const SwitchId dst) const;

  //this function reads the topology from the file and load it into graph_ data structure
  void read_topology_from_file();

  //track the inport numbers of source destination pair connection
  void generate_inport_mapping();
  
  //this function reads the allpaths in the given topology and load it into Allpaths data structure
  virtual void read_allpaths_from_file();

  // returns all the paths between source and destination switch
  void find_paths(SwitchId src,SwitchId dst,std::vector<std::vector<int>>& pathlists);

  protected:
  long N_; //the number of routers
  long r_; //'network degree' of each router
  long K_; //path spread to be used in KSP routing
  int diameter_; 
  int** graph_;
  bool is_slimfly_instance_;
  bool are_routes_biased_;
  
  std::map<std::pair<SwitchId, SwitchId>, int> inport_map_;
  std::map<std::pair<SwitchId, SwitchId>, int> inport_map1_;
  //maps all the paths between source and destination switch pairs
  std::map< std::pair<SwitchId, SwitchId>, std::vector<std::vector<int>> > path_lists;
   //maps all the shortest paths between source and destination switch pairs
  std::map< std::pair<SwitchId, SwitchId>, std::vector<std::vector<int>> > short_path_lists;
  
 };

}
}

#endif
