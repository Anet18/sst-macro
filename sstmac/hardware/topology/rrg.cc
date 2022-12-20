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

#include<bits/stdc++.h>
#include <sstmac/hardware/topology/rrg.h>
#include <sprockit/sim_parameters.h>
#include <sprockit/keyword_registration.h>
#include <sstream>
#include <fstream>

RegisterKeywords(
{ "N_", "the number of routers" },
{ "r_", "network degree of each router"},
{ "K_", "path spread to be used in KSP routing"},
);

namespace sstmac {
  namespace hw {
	  
rrg::rrg(SST::Params& params) :
  StructuredTopology(params)
{
	std::vector<int> args;
    params.find_array("geometry", args);
    N_ = args[0];
    r_ = args[1];
    K_ = args[2];
	
	is_slimfly_instance_=false;
    if(args.size()>3){
      if(args[3]==1)
        is_slimfly_instance_ = true;
    }
	
	//allocate memory for graph
    graph_ = new int *[N_];
    for(int i=0;i<N_;i++)
      graph_[i] = new int[r_];


    for(int i=0;i<N_;i++)
      for(int j=0;j<r_;j++)
        graph_[i][j] = -1;  //-1:unconnected
	
	read_topology_from_file();
    generate_inport_mapping();

    diameter_=3;
    read_allpaths_from_file();

    printf("RRG: constructor complete\n");
	
}	

//Destructor: deallocated the graph structure
/*rrg::~rrg()
{
  for(int i=0;i<N_;i++)
    delete[] graph_[i];

  delete[] graph_;    
}*/

void rrg::read_topology_from_file()
{
  char topo_filename[100];
  std::ifstream ifs;

  if(is_slimfly_instance_==true)
    sprintf(topo_filename,"sfly_q%d.topology", (int)sqrt(N_/2) );
  else
    sprintf(topo_filename,"rrg_%ld_%ld.topology", N_, r_);

  ifs.open(topo_filename);
  if(!ifs.is_open()){
    spkt_abort_printf("[RRG: read_topology_from_file]: error reading from file %s\n", topo_filename);
  }
  
  std::string line;
  int linecount=0;
  while( getline(ifs, line)){ 
	//top_debug("neighbors of node %d : %d",linecount,line);
    std::stringstream ss(line);
    int neighborsw;
    int neighborcount=0;
    while( ss >> neighborsw){
      graph_[linecount][neighborcount] = neighborsw;    
      neighborcount++;
    }
    linecount++;
  }

  if(linecount<N_)
  {
    spkt_abort_printf("[RRG: read_topology_from_file]: should be atleast %d lines, there are %d",N_, linecount);
  }else{
    top_debug("[RRG: read_topology_from_file]: Success");
  }

}


void rrg::read_allpaths_from_file()
{

  char allpath_filename[100];
  std::ifstream ifs;
  if(is_slimfly_instance_==true)
    sprintf(allpath_filename,"sfly_q%d_K%d.allroutes", (int)sqrt(N_/2), K_);
  else 
    sprintf(allpath_filename,"rrg_%ld_%ld_K%d.allroutes", N_, r_,K_);

  ifs.open(allpath_filename);
  if(!ifs.is_open()){
    spkt_abort_printf("[read_allpaths_from_file]: error reading from file %s\n", allpath_filename);
  }
  int s,d,pathcount;
  while(ifs >> s >> d >> pathcount){
    if(pathcount<K_){
      spkt_abort_printf("[rrg::read_allpaths_from_file]:Sd pair (%d,%d) has only %d paths, required Kvalue is %d\n",s,d,pathcount,K_ );
    }
    ifs.ignore(200,'\n'); 
    top_debug("Debug: %d %d %d",s,d,pathcount);

    //the following <pathcount> lines will contain routes from s to d
	std::vector<std::vector<int>> pathlists_between_s_and_d;
	std::vector<std::vector<int>> shortpathlists_between_s_and_d;
    for(int i=0; i<pathcount; i++){
      std::string line;
      getline(ifs, line);
      //std::cout<<"path line: " << line<<std::endl;
	  std::vector <int> single_path;
      std::stringstream ss(line);
      int currenthop, nexthop;
      ss >> currenthop;
	  single_path.push_back(currenthop);
      //std::cout<<"Hops: "<< currenthop <<" ";
	  int hop_counter=0;
	  while(ss >> nexthop){
        //std::cout<<nexthop <<" ";
        single_path.push_back(nexthop);
		hop_counter++;
     }
	 pathlists_between_s_and_d.push_back(single_path);
     if(hop_counter <=diameter_) 
	 {
		 shortpathlists_between_s_and_d.push_back(single_path);
	 }
     single_path.clear();
    } //end for
	path_lists[std::make_pair(s,d)] = pathlists_between_s_and_d;
        short_path_lists[std::make_pair(s,d)] = shortpathlists_between_s_and_d;
    pathlists_between_s_and_d.clear();
	shortpathlists_between_s_and_d.clear();
  } //end while
  top_debug("Successfully read all paths from file %s\n",allpath_filename);
  ifs.close();
}


int rrg::port_to_neighbor(const SwitchId src, const SwitchId dst) const
{
  //if(src==dst) return -1;  
  for (int i=0; i < r_; ++i){
    if( graph_[src][i] ==dst )
      return i;
  }
  top_debug("[rrg::port_to_neighbor]:No port found to switch %d from %d", dst, src);
  return -1;

}

/*for example if the switch id is 0 and concentration is 4 then 
this function returns an array that contains all the nodes connected to switch 0
nodes number will be 0,1,2,3 for switch 0
and 4,5,6,7 for switch 1 etc.
switch port numbers that are connected to nodes will be 0,1,2,3 for all switches 
*/
void
rrg::endpointsConnectedToInjectionSwitch(SwitchId swid,
                                   std::vector<InjectionPort>& nodes) const
{
  nodes.resize(concentration_);
  for (int i = 0; i < concentration_; i++) {
    InjectionPort& port = nodes[i];
    port.nid = swid*concentration_ + i;
    port.switch_port = i;
    port.ep_port = 0;
  }
}

//function for getting all paths between a source and destination router
void rrg::find_paths(SwitchId src,SwitchId dst,std::vector<std::vector<int>>& pathlists)
{
    for(int i=0;i<path_lists.at(std::make_pair(src,dst)).size();i++)
    {
        pathlists.push_back(path_lists.at(std::make_pair(src,dst)).at(i));
    }
}


void
rrg::connectedOutports(SwitchId src, std::vector<Connection>& conns) const
{
  conns.clear();
  conns.resize(r_); //each switch has upto r_ outgoing links
  int dst_id, cidx = 0;
  for (int i=0; i < r_; ++i){
    dst_id =  graph_[src][i] ;
    if(dst_id==src) continue; //possible loopback link
    if(dst_id==-1) break;     //reached the unused ports
    auto sdpair = std::make_pair(dst_id, src);
    if(inport_map_.count(sdpair)==0)
      spkt_abort_printf("[connected_outports]: no dst inport mapping for %d from neighbor %d\n", dst_id,src); 
    conns[cidx].src = src;
    conns[cidx].dst = dst_id;
    conns[cidx].src_outport = concentration_+i;
    conns[cidx].dst_inport = concentration_+inport_map_.at(std::make_pair(dst_id, src));
    ++cidx;
  }
  conns.resize(cidx);
}

void rrg::generate_inport_mapping()
{
  int src,i,dst_id;
  int *port_tracker = new int[N_];
  for(i=0;i<N_;i++) port_tracker[i]=0;

  for(src=0;src<N_;src++){
    for(i=0;i<r_;i++){
      dst_id =  graph_[src][i] ;
      if(dst_id==src) continue;
      if(dst_id==-1) break;
      inport_map_[std::make_pair(dst_id,src)] = port_tracker[dst_id];
      port_tracker[dst_id]++;      
    }
  }
  delete [] port_tracker;

}


}
} //end of namespace sstmac







