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
//..................................................
// Author: Md Atiqul Mollah <mollah@oakland.edu>
// Author: Md Nahid Newaz <mdnahidnewaz@oakland.edu>
//..................................................


#include <sstmac/hardware/router/router.h>
#include <sstmac/hardware/switch/network_switch.h>
#include <sstmac/hardware/topology/rrg.h>
#include <sstmac/hardware/interconnect/interconnect.h>
#include <sprockit/keyword_registration.h>
#include <sstmac/common/event_manager.h>
#include <sprockit/util.h>
#include <sprockit/sim_parameters.h>
#include <cmath>

#define rrg_rter_debug(...) \
  rter_debug("RRG: %s", sprockit::printf(__VA_ARGS__).c_str())

namespace sstmac {
namespace hw {

struct uniform_multipath_router : public Router {
  SST_ELI_REGISTER_DERIVED(
    Router,
    uniform_multipath_router,
    "macro",
    "uniform_multipath",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "router implementing uniform multipath routing in RRG")
	
    static const char initial_stage = 0;
    static const char intermediate_stage = 1;
    static const char final_stage = 2;
	//int y=0;
  struct header : public Packet::Header {
    uint8_t stage_number : 4;
    uint16_t port_array[10];
    uint8_t index;
  };

 public:
  uniform_multipath_router(SST::Params& params, Topology* top,
                           NetworkSwitch* netsw) :
    Router(params, top, netsw)
  {
    rrg_ = dynamic_cast<rrg*>(top);
    if (!rrg_){
      spkt_abort_printf("uniform_multipath_router can only be used with rrg topology");
    }

  }

  int numVC() const override {
    return 7;
  }

  std::string toString() const override {
    return "rrg uniform multipath router";
  }

  void route(Packet *pkt) override
  {    
                                                   
    auto* hdr = pkt->rtrHeader<header>();       
    //find destination switch address
    SwitchId dst = pkt->toaddr() / rrg_->concentration();

	//if current router is the destination router
    if (dst == my_addr_){
      hdr->edge_port = pkt->toaddr() % rrg_->concentration();
      hdr->deadlock_vc = 0;
    }
    //current router is a source router
    else if(hdr->stage_number != intermediate_stage)
    {
	    	
        //we need to find all paths between source and destination switch
	    std::vector<std::vector<int>> all_paths_between_s_and_d;
	    rrg_->find_paths(my_addr_,dst,all_paths_between_s_and_d);
	    int num_of_paths = all_paths_between_s_and_d.size();

	    //randomly select any path from all_paths 
	    int path_index= rand()%num_of_paths;
	    std::vector<int>path;
	    for(int i=0;i<all_paths_between_s_and_d.at(path_index).size();i++)
	    {
		    path.push_back(all_paths_between_s_and_d.at(path_index).at(i));
	    }
	    	
		//calculate all the edge ports on this randomly selected path
		//std::vector<int>ports;
		for(int i=0;i<path.size()-1;i++)//we need to discard destination switch thats why path.size()-1
		{
			std::vector<Topology::Connection> conns;
			rrg_->connectedOutports(path.at(i), conns);
			for (Topology::Connection& conn : conns)
			{
				if(conn.dst==path.at(i+1))//node i will connected to i+1 in the path
				{
					//ports.push_back(conn.src_outport);
					hdr->port_array[i]=conn.src_outport;
				}       
			}
		}
         		
		//now things to do on the packet on source router
		hdr->deadlock_vc = 0;
        hdr->stage_number = intermediate_stage;
        hdr->index = initial_stage;
        hdr->edge_port= hdr->port_array[hdr->index];
        hdr->index++;

	}
	else //intermediate router
	{           
        hdr->edge_port= hdr->port_array[hdr->index];
		hdr->index++;
		hdr->deadlock_vc = hdr->deadlock_vc+1;
	}

  }
 protected:
  rrg* rrg_;

};

class adaptive_multipath_router : public uniform_multipath_router {
 public:
  SST_ELI_REGISTER_DERIVED(
    Router,
    adaptive_multipath_router,
    "macro",
    "adaptive_multipath",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "router implementing adaptive multipath routing in rrg")

  std::string toString() const override {
    return "RRG adaptive multipath router";
  }
  
  int numVC() const override {
    return 7;
  }
  
  adaptive_multipath_router(SST::Params& params, Topology *top,
                       NetworkSwitch *netsw)
    : uniform_multipath_router(params, top, netsw)
  {
      rrg_ = safe_cast(rrg, top);
  }
  
  void route(Packet *pkt) override
  {
       
    long long int *x1= rrg_->total_minimal_path();
    long long int *y1= rrg_->total_non_minimal_path();
                                                
    auto* hdr = pkt->rtrHeader<header>();       
    //find destination switch address
     SwitchId dst = pkt->toaddr() / rrg_->concentration();

	//if current router is the destination router
    if (dst == my_addr_){
      hdr->edge_port = pkt->toaddr() % rrg_->concentration();
      hdr->deadlock_vc = 0;
    }

    //current router is a source router
    else if(hdr->stage_number != intermediate_stage)
    {
	    	
        //we need to find all paths between source and destination switch
	    std::vector<std::vector<int>> all_paths_between_s_and_d;
	    rrg_->find_paths(my_addr_,dst,all_paths_between_s_and_d);
	    int num_of_paths = all_paths_between_s_and_d.size();
		
		//find all the minimal paths and non minimal paths between source and destination
		std::vector<std::vector<int>> all_minimal_paths;
		std::vector<std::vector<int>> all_non_minimal_paths;
                //int hops = all_paths_between_s_and_d.at(0).size();
		
                        int first = rand()% num_of_paths;
			all_minimal_paths.push_back(all_paths_between_s_and_d.at(first));
			int second;
			do{
			second = rand()% num_of_paths;
			}while(first==second);

			all_non_minimal_paths.push_back(all_paths_between_s_and_d.at(second));
	    
	    //randomly select a minimal path
	    int min_index= rand()%all_minimal_paths.size();
		
		std::vector<int>min_path;
	    for(int i=0;i<all_minimal_paths.at(min_index).size();i++)
	    {
		    min_path.push_back(all_minimal_paths.at(min_index).at(i));
	    }
		//if there exists at least 1 non minimal path
		if(all_non_minimal_paths.size()!=0){
		//randomly select a non minimal path
		int non_min_index= rand()%all_non_minimal_paths.size();
		
		//UGAL decision
		int M = all_minimal_paths.at(min_index).size()-1;
		int N = all_non_minimal_paths.at(non_min_index).size()-1;
		
		std::vector<int>non_min_path;
	    for(int i=0;i<all_non_minimal_paths.at(non_min_index).size();i++)
	    {
		    non_min_path.push_back(all_non_minimal_paths.at(non_min_index).at(i));
	    }
		
		//find the minimal port number of source router
		int minimal_port;
		std::vector<Topology::Connection> conns;
		rrg_->connectedOutports(min_path.at(0), conns);
		for (Topology::Connection& conn : conns)
		{
			if(conn.dst==min_path.at(1))
			{
				minimal_port=conn.src_outport;
			}       
		}

        //find the non minimal port number of source router
		int nonminimal_port;
		std::vector<Topology::Connection> conns1;
		rrg_->connectedOutports(non_min_path.at(0), conns1);
		for (Topology::Connection& conn : conns1)
		{
			if(conn.dst==non_min_path.at(1))
			{
				nonminimal_port=conn.src_outport;
			}       
		}
		
		//UGAL comparison
		if(M*netsw_->queueLength(minimal_port, all_vcs)<= N*netsw_->queueLength(nonminimal_port, all_vcs))
		{
                         *x1=*x1+1;
			//calculate all the edge ports on minimal path
		    for(int i=0;i<min_path.size()-1;i++)//we need to discard destination switch thats why min_path.size()-1
		    {
			    std::vector<Topology::Connection> con;
			    rrg_->connectedOutports(min_path.at(i), con);
			    for (Topology::Connection& conn : con)
			    {
				    if(conn.dst==min_path.at(i+1))//node i will connected to i+1 in the path
				    {
					    hdr->port_array[i]=conn.src_outport;
				    }       
			    }
		    }
		}
		else
		{
                         *y1=*y1+1;
			//calculate all the edge ports on non-minimal path
		    for(int i=0;i<non_min_path.size()-1;i++)//we need to discard destination switch thats why non_min_path.size()-1
		    {
			    std::vector<Topology::Connection> con;
			    rrg_->connectedOutports(non_min_path.at(i), con);
			    for (Topology::Connection& conn : con)
			    {
				    if(conn.dst==non_min_path.at(i+1))//node i will connected to i+1 in the path
				    {
					    hdr->port_array[i]=conn.src_outport;
				    }       
			    }
		    }
		}
        }//end of if there exits at least one non minimal path
        else if(all_non_minimal_paths.size()==0) //all paths are minimal paths
        {
                     *x1=*x1+1;
			//calculate all the edge ports on minimal path
		    for(int i=0;i<min_path.size()-1;i++)//we need to discard destination switch thats why min_path.size()-1
		    {
			    std::vector<Topology::Connection> con;
			    rrg_->connectedOutports(min_path.at(i), con);
			    for (Topology::Connection& conn : con)
			    {
				    if(conn.dst==min_path.at(i+1))//node i will connected to i+1 in the path
				    {
					    hdr->port_array[i]=conn.src_outport;
				    }       
			    }
		    }

        }			
		//now things to do on the packet on source router
		hdr->deadlock_vc = 0;
        hdr->stage_number = intermediate_stage;
        hdr->index = initial_stage;
        hdr->edge_port= hdr->port_array[hdr->index];
        hdr->index++;

	}
	else //intermediate router
	{           
        hdr->edge_port= hdr->port_array[hdr->index];
		hdr->index++;
		hdr->deadlock_vc = hdr->deadlock_vc+1;
	}

  }
 protected:
  rrg* rrg_;
  

};


}
}//end of namespace
