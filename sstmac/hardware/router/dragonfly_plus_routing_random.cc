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

#include <sstmac/hardware/router/router.h>
#include <sstmac/hardware/switch/network_switch.h>
#include <sstmac/hardware/topology/dragonfly.h>
#include <sstmac/hardware/topology/dragonfly_plus.h>
#include <sprockit/util.h>
#include <sprockit/sim_parameters.h>
#include <cmath>

#define ftree_rter_debug(...) \
  rter_debug("fat tree: %s", sprockit::printf(__VA_ARGS__).c_str())
using namespace std;
namespace sstmac {
namespace hw {

class DragonflyPlusAlltoallMinimalRouter : public Router {
 public:
  struct header : public Packet::Header {};

  SST_ELI_REGISTER_DERIVED(
    Router,
    DragonflyPlusAlltoallMinimalRouter,
    "macro",
    "dragonfly_plus_alltoall_minimal",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "router implementing minimal routing for dragonfly+")

  DragonflyPlusAlltoallMinimalRouter(SST::Params& params, Topology *top,
                         NetworkSwitch *netsw)
    : Router(params, top, netsw)
  {
    dfly_ = safe_cast(DragonflyPlus, top);
    num_leaf_switches_ = dfly_->g() * dfly_->a();
    //stagger by switch id
    rotater_ = (my_addr_) % dfly_->a();
    grp_rotaters_.resize(dfly_->g());
    for (int i=0; i < dfly_->g(); ++i){
      grp_rotaters_[i] = 0;
    }

    my_g_ = (my_addr_%num_leaf_switches_) / dfly_->a();
    my_row_ = my_addr_ / num_leaf_switches_;

   //figure out which groups it is directly connected to and check of correctness
    std::vector<int> connected;
    my_a = dfly_->computeA(my_addr_);
    dfly_->groupWiring()->connectedRouters(my_a, my_g_, connected);
    static_route_ = params.find<bool>("static", false);
  }

  int numVC() const override {
    return 1;
  }

  std::string toString() const override {
    return "dragonfly+ minimal circulant router";
  }

  void route(Packet *pkt) override {
    auto* hdr = pkt->rtrHeader<header>();
    SwitchId ej_addr = pkt->toaddr() / dfly_->concentration();
    if (ej_addr == my_addr_){
      hdr->edge_port = pkt->toaddr() % dfly_->concentration() + dfly_->a();
      hdr->deadlock_vc = 0;
      return;
    }
    int dstG = (ej_addr % num_leaf_switches_) / dfly_->a();
	std::vector<std::pair<int,int>> groupConnections;
	if(my_g_ != dstG){
	dfly_->groupWiring()->connectedToGroup(my_g_, dstG, groupConnections);
	if (groupConnections.size() == 0){
        spkt_abort_printf("Got zero group connections from %d->%d", my_g_, dstG);
      }
	}
    if (my_row_ == 0){
      if (static_route_){
		  if(my_g_ == dstG)
			hdr->edge_port = ej_addr % dfly_->a();
          else
			hdr->edge_port = groupConnections[0].first;
      } else {
		if(my_g_ == dstG)
		{
			hdr->edge_port = rotater_;
            rotater_ = (rotater_ + 1) % dfly_->a();
		}
		else
		{
			hdr->edge_port = groupConnections[rotate_g].first;
            rotate_g = (rotate_g + 1) % groupConnections.size();
		}
      }
    } else if (my_g_ == dstG){
      int dstA = ej_addr % dfly_->a();
      hdr->edge_port = dstA;
    } else {
	  int port_no;
	  std::vector<int> con; 
	  for(int i=0;i<groupConnections.size();i++)
	  {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	  }
	  int con_index = rand()%con.size();
	  port_no = con[con_index];
	  int port = (port_no%dfly_->h())+ dfly_->a();
	  hdr->edge_port = port;
    }
    hdr->deadlock_vc = 0;
  }

 protected:
  int rotate_g = 0;
  int num_leaf_switches_;
  int rotater_;
  int my_a;
  int my_g_;
  int my_row_;
  std::vector<int> grp_rotaters_;
  int covering_;
  DragonflyPlus* dfly_;
  bool static_route_;
};

class DragonflyPlusParRouter : public DragonflyPlusAlltoallMinimalRouter {
  struct header : public DragonflyPlusAlltoallMinimalRouter::header {
    uint8_t stage_number : 4;
  };
 public:
  SST_ELI_REGISTER_DERIVED(
    Router,
    DragonflyPlusParRouter,
    "macro",
    "dragonfly_plus_par",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "router implementing PAR for dragonfly+")

  static const char initial_stage = 0;
  static const char valiant_stage1 = 1;
  static const char valiant_stage2 = 2;
  static const char final_stage = 3;

  std::string toString() const override {
    return "dragonfly+ PAR router";
  }

  DragonflyPlusParRouter(SST::Params& params, Topology *top,
                       NetworkSwitch *netsw)
    : DragonflyPlusAlltoallMinimalRouter(params, top, netsw)
  {
    dfly_ = safe_cast(DragonflyPlus, top);
    my_row_ = my_addr_ / dfly_->numLeafSwitches();
    my_g_ = (my_addr_ % dfly_->numLeafSwitches()) / dfly_->a();
	my_a = dfly_->computeA(my_addr_);
    //covering_ = dfly_->h() / (dfly_->g() - 1);
    grp_rotaters_.resize(dfly_->g());
    for (int i=0; i < dfly_->g(); ++i){
      grp_rotaters_[i] = 0;
    }
    up_rotater_ = 0;
	rotate_g = 0;
    num_leaf_switches_ = dfly_->numLeafSwitches();
  }

  void route(Packet *pkt) override {
    long long int *x1= dfly_->total_minimal_path();
    long long int *y1= dfly_->total_non_minimal_path(); 
    long long int *z1= dfly_->total_local_up_path();
    long long int *w1= dfly_->total_local_down_path();	
    SwitchId ej_addr = pkt->toaddr() / dfly_->concentration();
	
	int dstG = (ej_addr % num_leaf_switches_) / dfly_->a();
	std::vector<std::pair<int,int>> groupConnections;
	if(my_g_ != dstG){
	dfly_->groupWiring()->connectedToGroup(my_g_, dstG, groupConnections);
	if (groupConnections.size() == 0){
        spkt_abort_printf("Got zero group connections from %d->%d", my_g_, dstG);
      }
	}
	
    auto hdr = pkt->rtrHeader<header>();
    if (my_row_ == 0 && hdr->stage_number != valiant_stage2){ //source leaf node
	   //source leaf node to source spine node no deadlock VC change
       hdr->deadlock_vc = 0;
      if (ej_addr == my_addr_){
        hdr->edge_port = pkt->toaddr() % dfly_->concentration() + dfly_->a();
      } else {
        //gotta route up
		*z1=*z1+1;
        //rter_debug("routing up on %d", up_rotater_);
		if(my_g_ == dstG)
		{
			hdr->edge_port = up_rotater_;
            up_rotater_ = (up_rotater_ + 1) % dfly_->a();
		}
        else
		{
			hdr->edge_port = groupConnections[rotate_g].first;
            rotate_g = (rotate_g + 1) % groupConnections.size();
		}
      }
    } 
	else if (my_row_ == 0 && hdr->stage_number == valiant_stage2){//intermediate leaf node
	    //intermediate leaf node to intermediate spine node deadlock VC should be 1
		*z1=*z1+1;
		hdr->deadlock_vc = 1;
		hdr->edge_port = groupConnections[rotate_g].first;
        rotate_g = (rotate_g + 1) % groupConnections.size();
	}
	else {
      if (my_g_ == dstG && hdr->stage_number != final_stage){
        //go down to the eject stage
		//packet reached destination group minimally so no deadlock VC change in going eject stage
		*w1=*w1+1;
        int dstA = ej_addr % dfly_->a();
        hdr->edge_port = dstA;
      } 
	  else if (my_g_ == dstG && hdr->stage_number == final_stage){
        //go down to the eject stage
		//packet reached destination group non-minimally so deadlock VC should 1 in going eject stage
		hdr->deadlock_vc = 1;
		*w1=*w1+1;
        int dstA = ej_addr % dfly_->a();
        hdr->edge_port = dstA;
      } 
	  else if (hdr->stage_number == valiant_stage1) {
		  int port_no;
		  std::vector<int> con; 
	    for(int i=0;i<groupConnections.size();i++)
	    {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	    }
		if(con.size()==0){
		  //down route packet
          //packet goes from intermediate spine to intermediate leaf so no deadlock VC change
		  *w1=*w1+1;
		  int dstA = rand() % dfly_->a();
		  hdr->stage_number = valiant_stage2;
          hdr->edge_port = dstA;
		}
		else{
		//packet goes directly from intermediate spine to destination spine so deadlock VC should 1
        hdr->deadlock_vc = 1;
		int con_index = rand()%con.size();
	    port_no = con[con_index];
	    int port = (port_no%dfly_->h())+ dfly_->a();
	    hdr->edge_port = port;
		hdr->stage_number = final_stage;
		}
      } else if(hdr->stage_number == valiant_stage2){
		//packet goes from intermediate spine to destination spine after down routing so deadlock VC should 1
	    hdr->deadlock_vc = 1;	
        int port_no;
	    std::vector<int> con; 
	    for(int i=0;i<groupConnections.size();i++)
	    {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	    }
	    int con_index = rand()%con.size();
	    port_no = con[con_index];
	    int port = (port_no%dfly_->h())+ dfly_->a();
	    hdr->edge_port = port;
	    hdr->stage_number = final_stage;
		
	  }else {
	  int port_no;
	  std::vector<int> con; 
	  for(int i=0;i<groupConnections.size();i++)
	  {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	  }
	  int con_index = rand()%con.size();
	  port_no = con[con_index];
        int minimalPort = (port_no%dfly_->h())+ dfly_->a();
        if (dfly_->g() > 2){
          //we must have an intermediate group - otherwise we just have minimal
          //we must make a ugal decision here
          int valiantPort;
		  do{  
		  valiantPort = (rand()%dfly_->h()) + dfly_->a();
		  } while (valiantPort==minimalPort);
		  
          int valiantMetric = 2*netsw_->queueLength(valiantPort, all_vcs);
          int minimalMetric = netsw_->queueLength(minimalPort, all_vcs);

          //rter_debug("comparing minimal(%d) %d against non-minimal(%d) %d",
                     //minimalPort, minimalMetric, valiantPort, valiantMetric);

          if (minimalMetric <= valiantMetric){
            // packet goes minimally so no deadlock VC change
			*x1=*x1+1;
            hdr->edge_port = minimalPort;
          } else {
			*y1=*y1+1;
			//packet goes non minimally deadlock VC should be 1
            hdr->deadlock_vc = 0;
            hdr->edge_port = valiantPort;
            hdr->stage_number = valiant_stage1;
          }
        } else { //no intermediate group - must go minimal
          //if packet goes minimally then no deadlock VC change
		  *x1=*x1+1;
          hdr->edge_port = minimalPort;
        }
      }
    }
  }

  int numVC() const override {
    return 2;
  }

 private:
  DragonflyPlus* dfly_;
  int rotate_g;
  int my_a;
  int my_row_;
  int my_g_;
  int up_rotater_;
  int num_leaf_switches_;
  std::vector<int> grp_rotaters_;
  int covering_;

};

//..........................................................
 class DragonflyPlusPiggyBackRouter1 : public DragonflyPlusAlltoallMinimalRouter {
  struct header : public DragonflyPlusAlltoallMinimalRouter::header {
    uint8_t stage_number : 4;
  };
 public:
  SST_ELI_REGISTER_DERIVED(
    Router,
    DragonflyPlusPiggyBackRouter1,
    "macro",
    "dragonfly_plus_piggyback1",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "router implementing piggyback1 for dragonfly+")

  static const char initial_stage = 0;
  static const char valiant_stage1 = 1;
  static const char valiant_stage2 = 2;
  static const char valiant_stage3 = 3;
  static const char final_stage = 4;

  std::string toString() const override {
    return "dragonfly+ piggyback1 router";
  }


  DragonflyPlusPiggyBackRouter1(SST::Params& params, Topology *top,
                       NetworkSwitch *netsw)
    : DragonflyPlusAlltoallMinimalRouter(params, top, netsw)
  {
    dfly_ = safe_cast(DragonflyPlus, top);
    my_row_ = my_addr_ / dfly_->numLeafSwitches();
    my_g_ = (my_addr_ % dfly_->numLeafSwitches()) / dfly_->a();
    my_a = dfly_->computeA(my_addr_);
    //covering_ = dfly_->h() / (dfly_->g() - 1);
    grp_rotaters_.resize(dfly_->g());
    for (int i=0; i < dfly_->g(); ++i){
      grp_rotaters_[i] = 0;
    }
    up_rotater_ = 0;
    rotate_r = 0;
    num_leaf_switches_ = dfly_->numLeafSwitches();
	
	all_link_state = dfly_->global_array();
    all_link_state_timestamp = dfly_->global_timestamp_array();
    PB_latency=params.find<double>("pb_latency", 0.0);

    rter_debug("At router constructor current time is %lf", netsw_->now().time.sec());
    rter_debug("Piggybacking latency is set to %lf seconds", PB_latency);
  }

  void route(Packet *pkt) override {
    long long int *x1= dfly_->total_minimal_path();
    long long int *y1= dfly_->total_non_minimal_path(); 
    long long int *z1= dfly_->total_local_up_path();
    long long int *w1= dfly_->total_local_down_path();
    SwitchId ej_addr = pkt->toaddr() / dfly_->concentration();
    auto hdr = pkt->rtrHeader<header>();
	
    int dstG = (ej_addr % num_leaf_switches_) / dfly_->a();
    std::vector<std::pair<int,int>> groupConnections;
    if(my_g_ != dstG){
    dfly_->groupWiring()->connectedToGroup(my_g_, dstG, groupConnections);
    if (groupConnections.size() == 0){
        spkt_abort_printf("Got zero group connections from %d->%d", my_g_, dstG);
      }
	}

    //leaf level, source and destination  group routing	
    if (my_row_ == 0 && hdr->stage_number != valiant_stage3){//source leaf
      //source leaf to source spine no deadlock VC change
      hdr->deadlock_vc = 0;

      if (ej_addr == my_addr_){
        hdr->edge_port = pkt->toaddr() % dfly_->concentration() + dfly_->a();
      } else {
		*z1=*z1+1;
        //Intra-group routing, gotta route up
        if (my_g_ == dstG)
        {
            //rter_debug("routing up on %d", up_rotater_);
            hdr->edge_port = up_rotater_;
            up_rotater_ = (up_rotater_ + 1) % dfly_->a();
        }
        else{
            //piggyback stage
            int ct=0;
			//std::vector<std::pair<int,int>> uncongested;
            vector < int > uncongested;
            vector < int > val;
            //find the connected routers
			for(int i=0;i<groupConnections.size();i++)
			{
				int a = groupConnections[i].first;
				int r = dfly_->getUid(1,a,my_g_);
				int port_no = groupConnections[i].second;
				int port = (port_no%dfly_->h())+ dfly_->a();
				if((*all_link_state)[r][port]==1)
               {
                   ct++;
                   val.push_back(a);
               }
               else
               {
                   //uncongested.emplace_back(a,port);
				   uncongested.push_back(a);
               }
				
			}
            if(ct==groupConnections.size())
            {
                //all link are congested need to go valiant state
                //rter_debug("routing up on %d", up_rotater_);
                hdr->edge_port = up_rotater_;
                up_rotater_ = (up_rotater_ + 1) % dfly_->a();
                hdr->stage_number = valiant_stage1;
            }
            else
            {
                //randomly pick any K minimal global connected router
                int RANDOM = 6;
                vector < int >random_minimal;
                //srand(time(NULL));
                for(int k=0;k<RANDOM;k++)
                {
                    int port_index=rand()%uncongested.size();
                    random_minimal.push_back(uncongested[port_index]);
                }
                //rter_debug("..%d %d .. %d %d..",congested.size(),val.size(),random_minimal.size(),random_val.size());
                int min_random_minimal=random_minimal.at(0);
                int port1,port2;
                //calculate least locally congested random minimal port
                for(int k=1;k<RANDOM;k++)
                {
                    port1=min_random_minimal;
                    port2=random_minimal.at(k);
                    if(netsw_->queueLength(port1, all_vcs)>netsw_->queueLength(port2, all_vcs))
                    {
                        min_random_minimal=port2;
                    }

                }
				hdr->edge_port = min_random_minimal;
            }


        }
      }
    }
    else if (my_row_ == 0 && hdr->stage_number == valiant_stage3){//intermediate leaf
      //intermediate leaf to intermediate spine deadlock VC should be 1
	  *z1=*z1+1;
	  hdr->deadlock_vc = 1;
      hdr->edge_port = groupConnections[rotate_r].first;
      rotate_r = (rotate_r + 1) % groupConnections.size();
    }		
    else {
        //calculate the congestion every time it enters the global level
        int p=0;
        int total_queue_length=0;
        for(int i=0;i<dfly_->a();i++)
        {
            total_queue_length=total_queue_length+netsw_->queueLength(i+dfly_->a(),all_vcs);
        }
        float average_queue_length=total_queue_length/dfly_->a();
		
		for(int j=0;j<dfly_->a();j++)
        {
          double last_updated = (*all_link_state_timestamp)[my_addr_][j+dfly_->a()];
          if(netsw_->now().time.sec() >= last_updated + PB_latency){

                //copy from old, saved information
                (*all_link_state)[my_addr_][j+dfly_->a()]= (*all_link_state)[my_addr_][dfly_->maxNumPorts()+j+dfly_->a()];

                int x= netsw_->queueLength(j+dfly_->a(), all_vcs);

                if(x>2*average_queue_length)
                {
                    (*all_link_state)[my_addr_][dfly_->maxNumPorts()+j+dfly_->a()]=1; 
                }
                else
                {
                    (*all_link_state)[my_addr_][dfly_->maxNumPorts()+j+dfly_->a()]=0;
                }

                (*all_link_state_timestamp)[my_addr_][j+dfly_->a()] = netsw_->now().time.sec();
          }
        }
      //int dstG = (ej_addr % num_leaf_switches_) / dfly_->a();


      if (my_g_ == dstG && hdr->stage_number != final_stage){
        //go down to the eject stage
		//packet comes minimally to the destination group no deadlock VC change
		*w1=*w1+1;
        int dstA = ej_addr % dfly_->a();
        hdr->edge_port = dstA;
      }
	  else if (my_g_ == dstG && hdr->stage_number == final_stage){
        //go down to the eject stage
		//packet comes non-minimally to the destination group deadlock VC should be 1
		*w1=*w1+1;
        hdr->deadlock_vc = 1;
        int dstA = ej_addr % dfly_->a();
        hdr->edge_port = dstA;
      }
      else if(hdr->stage_number == valiant_stage1)
      {
		//source spine to intermediate spine no deadlock VC change
		*y1=*y1+1;
		int port = rand()%dfly_->h();
		hdr->stage_number = valiant_stage2;
		hdr->edge_port = port+dfly_->a(); 
		//rter_debug("continuing non-minimal path from %d to %d on port %d not port %d",my_g_,dstG,intermediate_groups[port_inter],port);
      }
      else if (hdr->stage_number == valiant_stage2){
            int port_no;
            std::vector<int> con; 
	    for(int i=0;i<groupConnections.size();i++)
	    {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	    }
	    if(con.size()==0){
		  //down route packet
          //intermediate spine to intermediate leaf no deadlock VC change
		  *w1=*w1+1;
		  int dstA = rand() % dfly_->a();
		  hdr->stage_number = valiant_stage3;
          hdr->edge_port = dstA;
	    }
	    else{
          //packet directly goes from intermediate spine to destination spine deadlock VC should be 1
          hdr->deadlock_vc = 1;
	      int con_index = rand()%con.size();
	      port_no = con[con_index];
	      int port = (port_no%dfly_->h())+ dfly_->a();
	      hdr->edge_port = port;
	      hdr->stage_number = final_stage;
		}
	   }
	  else if (hdr->stage_number == valiant_stage3) {
        //packet goes intermediate spine to destination spine after down routing deadlock VC should be 1
        hdr->deadlock_vc = 1;
        int port_no;
	    std::vector<int> con; 
	    for(int i=0;i<groupConnections.size();i++)
	    {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	    }
	    int con_index = rand()%con.size();
	    port_no = con[con_index];
	    int port = (port_no%dfly_->h())+ dfly_->a();
	    hdr->edge_port = port;
        hdr->stage_number = final_stage;
        //rter_debug("continuing non-minimal path from %d to %d on port %d ",my_g_,dstG,port);
      } else {
      //packet goes minimally from source spine to destination spine no change in deadlock VC
	  *x1=*x1+1;
      int port_no;
	  std::vector<int> con; 
	  for(int i=0;i<groupConnections.size();i++)
	  {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	  }
	  int con_index = rand()%con.size();
	  port_no = con[con_index];
	  int port = (port_no%dfly_->h())+ dfly_->a();
	  hdr->edge_port = port;
     }
    }
  }

  int numVC() const override {
    return 2;
  }

 private:
  DragonflyPlus* dfly_;
  int rotate_r;
  int my_row_;
  int my_a;
  int my_g_;
  int up_rotater_;
  int num_leaf_switches_;
  std::vector<int> grp_rotaters_;
  int covering_;
  
  mytype all_link_state;
  mytype1 all_link_state_timestamp;
  double PB_latency;

};

//........................................................................................
class DragonflyPlusPiggyBackRouter2 : public DragonflyPlusAlltoallMinimalRouter {
  struct header : public DragonflyPlusAlltoallMinimalRouter::header {
    uint8_t stage_number : 4;
	int randomG;
  };
 public:
  SST_ELI_REGISTER_DERIVED(
    Router,
    DragonflyPlusPiggyBackRouter2,
    "macro",
    "dragonfly_plus_piggyback2",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "router implementing piggyback2 for dragonfly+")
  
  static const char initial_stage = 0;
  static const char valiant_stage1 = 1;
  static const char valiant_stage11 = 5;
  static const char valiant_stage2 = 2;
  static const char valiant_stage3 = 3;
  static const char final_stage = 4;

  std::string toString() const override {
    return "dragonfly+ piggyback2 router";
  }
DragonflyPlusPiggyBackRouter2(SST::Params& params, Topology *top,
                       NetworkSwitch *netsw)
    : DragonflyPlusAlltoallMinimalRouter(params, top, netsw)
  {
    dfly_ = safe_cast(DragonflyPlus, top);
    my_row_ = my_addr_ / dfly_->numLeafSwitches();
    my_g_ = (my_addr_ % dfly_->numLeafSwitches()) / dfly_->a();
	my_a = dfly_->computeA(my_addr_);
    //covering_ = dfly_->h() / (dfly_->g() - 1);
    grp_rotaters_.resize(dfly_->g());
    for (int i=0; i < dfly_->g(); ++i){
      grp_rotaters_[i] = 0;
    }
    up_rotater_ = 0;
	rotate_r = 0;
    num_leaf_switches_ = dfly_->numLeafSwitches();all_link_state = dfly_->global_array();
	
	all_link_state = dfly_->global_array();
    all_link_state_timestamp = dfly_->global_timestamp_array();
    PB_latency=params.find<double>("pb_latency", 0.0);

    rter_debug("At router constructor current time is %lf", netsw_->now().time.sec());
    rter_debug("Piggybacking latency is set to %lf seconds", PB_latency);
	
  }
 void route(Packet *pkt) override {
    long long int *x1= dfly_->total_minimal_path();
    long long int *y1= dfly_->total_non_minimal_path(); 
    long long int *z1= dfly_->total_local_up_path();
    long long int *w1= dfly_->total_local_down_path();
    SwitchId ej_addr = pkt->toaddr() / dfly_->concentration();
    auto hdr = pkt->rtrHeader<header>();
	
	int dstG = (ej_addr % num_leaf_switches_) / dfly_->a();
	std::vector<std::pair<int,int>> groupConnections;
	if(my_g_ != dstG){
	dfly_->groupWiring()->connectedToGroup(my_g_, dstG, groupConnections);
	if (groupConnections.size() == 0){
        spkt_abort_printf("Got zero group connections from %d->%d", my_g_, dstG);
      }
	}
	
	
    if (my_row_ == 0 && hdr->stage_number != valiant_stage3){//source leaf
      hdr->deadlock_vc = 0;
      if (ej_addr == my_addr_){
        hdr->edge_port = pkt->toaddr() % dfly_->concentration() + dfly_->a();
      } else {
        //gotta route up
		*z1=*z1+1;
        if (my_g_ == dstG)
        {
            //rter_debug("routing up on %d", up_rotater_);
            hdr->edge_port = up_rotater_;
            up_rotater_ = (up_rotater_ + 1) % dfly_->a();
        }
        else
        {
            //piggyback stage
            int ct=0;
			//std::vector<std::pair<int,int>> uncongested;
            vector < int > uncongested;
            vector < int > val;
            //find the connected routers
			for(int i=0;i<groupConnections.size();i++)
			{
				int a = groupConnections[i].first;
				int r = dfly_->getUid(1,a,my_g_);
				int port_no = groupConnections[i].second;
				int port = (port_no%dfly_->h())+ dfly_->a();
				if((*all_link_state)[r][port]==1)
               {
                   ct++;
                   val.push_back(a);
               }
               else
               {
                   //uncongested.emplace_back(a,port);
				   uncongested.push_back(a);
               }
				
			}
	    if(ct==groupConnections.size())
            {
                //all link are congested need to go valiant state
                //rter_debug("routing up on %d", up_rotater_);
                hdr->edge_port = up_rotater_;
                up_rotater_ = (up_rotater_ + 1) % dfly_->a();
                hdr->stage_number = valiant_stage1;

            }
            else{
            //randomly pick any K minimal global connected router
            int RANDOM = 6;
            vector < int >random_minimal;
            //srand(time(NULL));
            for(int k=0;k<RANDOM;k++)
            {
                int port_index=rand()%uncongested.size();
                random_minimal.push_back(uncongested[port_index]);
            }
            //rter_debug("..%d %d .. %d %d..",congested.size(),val.size(),random_minimal.size(),random_val.size());
            int min_random_minimal=random_minimal.at(0);
            int port1,port2;
            //calculate least locally congested random minimal port
            for(int k=1;k<RANDOM;k++)
            {
                port1=min_random_minimal;
                port2=random_minimal.at(k);
                if(netsw_->queueLength(port1, all_vcs)>netsw_->queueLength(port2, all_vcs))
                {
                    min_random_minimal=port2;
                }

            }
			//random group selection
            int randG;
            do{
             randG=rand()%dfly_->g();
			 hdr->randomG = randG;
            }while(randG==dstG || randG==my_g_);

            int ct1=0;
            vector < int > uncongested1;
            vector < int > val1;
			std::vector<std::pair<int,int>> groupConnections1;
			dfly_->groupWiring()->connectedToGroup(my_g_, randG, groupConnections1);
            //find the connected routers
            for(int i=0;i<groupConnections1.size();i++)
			{
				int a = groupConnections1[i].first;
				int r = dfly_->getUid(1,a,my_g_);
				int port_no = groupConnections1[i].second;
				int port = (port_no%dfly_->h())+ dfly_->a();
				if((*all_link_state)[r][port]==1)
               {
                   ct1++;
                   val1.push_back(a);
               }
               else
               {
                   //uncongested.emplace_back(a,port);
				   uncongested1.push_back(a);
               }
				
			}
            if(ct1==groupConnections1.size())
            {
                hdr->edge_port = min_random_minimal;
            }
            else{
            //randomly pick any K minimal global connected router
            int RANDOM1 = 6;
            vector < int >random_minimal1;
            //srand(time(NULL));
            for(int k=0;k<RANDOM1;k++)
            {
                int port_index=rand()%uncongested1.size();
                random_minimal1.push_back(uncongested1[port_index]);
            }
            //rter_debug("..%d %d .. %d %d..",congested.size(),val.size(),random_minimal.size(),random_val.size());
            int min_random_minimal1=random_minimal1.at(0);
            int port11,port21;
            //calculate least locally congested random minimal port
            for(int k=1;k<RANDOM1;k++)
            {
                port11=min_random_minimal1;
                port21=random_minimal1.at(k);
                if(netsw_->queueLength(port11, all_vcs)>netsw_->queueLength(port21, all_vcs))
                {
                    min_random_minimal1=port21;
                }

            }
            //UGAL decision
            if(netsw_->queueLength(min_random_minimal, all_vcs)<=2*netsw_->queueLength(min_random_minimal1, all_vcs))
            {
                hdr->edge_port = min_random_minimal;
            }
            else
            {
                hdr->edge_port = min_random_minimal1;
                hdr->stage_number = valiant_stage11;
            }
            }
          }
        }
      }
    } 
	else if (my_row_ == 0 && hdr->stage_number == valiant_stage3){ //intermediate leaf
	    *z1=*z1+1;
		hdr->deadlock_vc = 1;
		hdr->edge_port = groupConnections[rotate_r].first;
        rotate_r = (rotate_r + 1) % groupConnections.size();
	}
	else {
       //calculate the congestion every time it enters the global level
        int s=0;
        int total_queue_length=0;
        for(int i=0;i<dfly_->a();i++)
        {
            total_queue_length=total_queue_length+netsw_->queueLength(i+dfly_->a(),all_vcs);
        }
        float average_queue_length=total_queue_length/dfly_->a();
		
        for(int j=0;j<dfly_->a();j++)
        {
          double last_updated = (*all_link_state_timestamp)[my_addr_][j+dfly_->a()];
          if(netsw_->now().time.sec() >= last_updated + PB_latency){

                //copy from old, saved information
                (*all_link_state)[my_addr_][j+dfly_->a()]= (*all_link_state)[my_addr_][dfly_->maxNumPorts()+j+dfly_->a()];

                int x= netsw_->queueLength(j+dfly_->a(), all_vcs);

                if(x>2*average_queue_length)
                {
                    (*all_link_state)[my_addr_][dfly_->maxNumPorts()+j+dfly_->a()]=1; 
                }
                else
                {
                    (*all_link_state)[my_addr_][dfly_->maxNumPorts()+j+dfly_->a()]=0;
                }

                (*all_link_state_timestamp)[my_addr_][j+dfly_->a()] = netsw_->now().time.sec();
          }
        }
		
		//int dstG = (ej_addr % num_leaf_switches_) / dfly_->a();
      if (my_g_ == dstG && hdr->stage_number != final_stage){
        //go down to the eject stage
		//packet comes minimally no change in VC
		*w1=*w1+1;
        int dstA = ej_addr % dfly_->a();
        hdr->edge_port = dstA;
      }
	  else if (my_g_ == dstG && hdr->stage_number == final_stage){
        //go down to the eject stage
		//packet comes non-minimally change in VC
		*w1=*w1+1;
		hdr->deadlock_vc = 1;
        int dstA = ej_addr % dfly_->a();
        hdr->edge_port = dstA;
      }
	  else if (hdr->stage_number == valiant_stage11) {
		//non minimal(based on local congestion) source spine to intermediate spine no VC change
        *y1=*y1+1;
	    int port;
        std::vector<std::pair<int,int>> groupConnections1;
		dfly_->groupWiring()->connectedToGroup(my_g_, hdr->randomG, groupConnections1);
		for(int i=0;i<groupConnections1.size();i++)
	    {
		  if(groupConnections1[i].first == my_a)
		  {
			  port = groupConnections1[i].second;
		  }
	    }
		hdr->stage_number = valiant_stage2;
		hdr->edge_port = (port%dfly_->h())+ dfly_->a();  
      }
      else if (hdr->stage_number == valiant_stage1) {
		  //non minimal(based on global congestion) source spine to intermediate spine no VC change
            *y1=*y1+1;
	        int port = rand()%dfly_->h();
            hdr->stage_number = valiant_stage2;
            hdr->edge_port = port+dfly_->a();   
      }
      else if (hdr->stage_number == valiant_stage2) {
	     int port_no;
         std::vector<int> con; 
	    for(int i=0;i<groupConnections.size();i++)
	    {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	    }
		if(con.size()==0){
		  //down route packet
		  *w1=*w1+1;
		  //intermediate spine to intermediate leaf no VC change
		  int dstA = rand() % dfly_->a();
		  hdr->stage_number = valiant_stage3;
          hdr->edge_port = dstA;
		}
		else{
	    //directly goes from intermediate spine to destination spine change in VC
		hdr->deadlock_vc = 1;
		int con_index = rand()%con.size();
	    port_no = con[con_index];
	    int port = (port_no%dfly_->h())+ dfly_->a();
	    hdr->edge_port = port;
		hdr->stage_number = final_stage;
		}
      }
	  else if (hdr->stage_number == valiant_stage3) {
      //packet goes from intermediate spine to destination spine after down route change in VC
	  hdr->deadlock_vc = 1;
      int port_no;
	  std::vector<int> con; 
	  for(int i=0;i<groupConnections.size();i++)
	  {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	  }
	  int con_index = rand()%con.size();
	  port_no = con[con_index];
	    int port = (port_no%dfly_->h())+ dfly_->a();
	    hdr->edge_port = port;
		hdr->stage_number = final_stage;
      }
	  else {
        //packet goes minimally from source spine to destination spine no change in VC
		*x1=*x1+1;
        int port_no;
	  std::vector<int> con; 
	  for(int i=0;i<groupConnections.size();i++)
	  {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	  }
	  int con_index = rand()%con.size();
	  port_no = con[con_index];
	    int port = (port_no%dfly_->h())+ dfly_->a();
	    hdr->edge_port = port;
     }
    }
  }

  int numVC() const override {
    return 2;
  }

 private:
  DragonflyPlus* dfly_;
  int rotate_r;
  int my_row_;
  int my_a;
  int my_g_;
  int up_rotater_;
  int num_leaf_switches_;
  std::vector<int> grp_rotaters_;
  int covering_;
  
  mytype all_link_state;
  mytype1 all_link_state_timestamp;
  double PB_latency;

};

//........................................................................................
class DragonflyPlusUGAL : public DragonflyPlusAlltoallMinimalRouter {
  struct header : public DragonflyPlusAlltoallMinimalRouter::header {
    uint8_t stage_number : 4;
	int randomG;
  };
 public:
  SST_ELI_REGISTER_DERIVED(
    Router,
    DragonflyPlusUGAL,
    "macro",
    "dragonfly_plus_ugal",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "router implementing UGAL for dragonfly+")

  static const char initial_stage = 0;
  static const char valiant_stage1 = 1;
  static const char valiant_stage2 = 2;
  static const char valiant_stage11 = 4;
  static const char final_stage = 3;

  std::string toString() const override {
    return "dragonfly+ UGAL router";
  }

  DragonflyPlusUGAL(SST::Params& params, Topology *top,
                       NetworkSwitch *netsw)
    : DragonflyPlusAlltoallMinimalRouter(params, top, netsw)
  {
    dfly_ = safe_cast(DragonflyPlus, top);
    my_row_ = my_addr_ / dfly_->numLeafSwitches();
    my_g_ = (my_addr_ % dfly_->numLeafSwitches()) / dfly_->a();
	my_a = dfly_->computeA(my_addr_);
    //covering_ = dfly_->h() / (dfly_->g() - 1);
    grp_rotaters_.resize(dfly_->g());
    for (int i=0; i < dfly_->g(); ++i){
      grp_rotaters_[i] = 0;
    }
    up_rotater_ = 0;
	rotate_g = 0;
    num_leaf_switches_ = dfly_->numLeafSwitches();
  }

  void route(Packet *pkt) override {
	
    long long int *x1= dfly_->total_minimal_path();
    long long int *y1= dfly_->total_non_minimal_path(); 
    long long int *z1= dfly_->total_local_up_path();
    long long int *w1= dfly_->total_local_down_path();
 	
    SwitchId ej_addr = pkt->toaddr() / dfly_->concentration();
	
	int dstG = (ej_addr % num_leaf_switches_) / dfly_->a();
	//this vector is for destination group
	std::vector<std::pair<int,int>> groupConnections;
	//this vector is for non-minimal intermediate group
	//std::vector<std::pair<int,int>> groupConnections1;
	if(my_g_ != dstG){
	dfly_->groupWiring()->connectedToGroup(my_g_, dstG, groupConnections);
	if (groupConnections.size() == 0){
        spkt_abort_printf("Got zero group connections from %d->%d", my_g_, dstG);
      }
	}
	
    auto hdr = pkt->rtrHeader<header>();
    if (my_row_ == 0 && hdr->stage_number != valiant_stage2){ //source leaf node
	   //source leaf node to source spine node no deadlock VC change
       hdr->deadlock_vc = 0;
      if (ej_addr == my_addr_){
        hdr->edge_port = pkt->toaddr() % dfly_->concentration() + dfly_->a();
      } else {
        //gotta route up
		*z1=*z1+1;
        //rter_debug("routing up on %d", up_rotater_);
		if(my_g_ == dstG)
		{
			
			hdr->edge_port = up_rotater_;
            up_rotater_ = (up_rotater_ + 1) % dfly_->a();
		}
        else //UGAL decesion here
		{
			//groupconnections has all the directly connected spine level routers with destination group
			int rt = rand()%groupConnections.size();
			int minp = groupConnections[rt].first;
			
			//randomly select an intermediate group
			int randG;
            do{
             randG=rand()%dfly_->g();
			 hdr->randomG = randG;
            }while(randG==dstG || randG==my_g_);
			
			//groupconnection1 has all the directly connected spine level routers with randomly selected intermediate group
			std::vector<std::pair<int,int>> groupConnections1;
			dfly_->groupWiring()->connectedToGroup(my_g_, randG, groupConnections1);
			int rt1 = rand()%groupConnections1.size();
			int nonminp = groupConnections1[rt1].first; 
			
			//UGAL coparison here
		    int valiantMetric = 2*netsw_->queueLength(nonminp, all_vcs);
            int minimalMetric = netsw_->queueLength(minp, all_vcs);
			if (minimalMetric <= valiantMetric){
            // packet goes minimally
			hdr->edge_port = minp;
			}
			else{
			//packet goes non-minimally
			hdr->edge_port = nonminp;
			hdr->stage_number = valiant_stage11;
			}
		}
      }
    } 
	else if (my_row_ == 0 && hdr->stage_number == valiant_stage2){//intermediate leaf node
	    //intermediate leaf node to intermediate spine node deadlock VC should be 1
		*z1=*z1+1;
		hdr->deadlock_vc = 1;
		hdr->edge_port = groupConnections[rotate_g].first;
        rotate_g = (rotate_g + 1) % groupConnections.size();
	}
	else {
      if (my_g_ == dstG && hdr->stage_number != final_stage){
        //go down to the eject stage
		//packet reached destination group minimally so no deadlock VC change in going eject stage
		*w1=*w1+1;
        int dstA = ej_addr % dfly_->a();
        hdr->edge_port = dstA;
      } 
	  else if (my_g_ == dstG && hdr->stage_number == final_stage){
        //go down to the eject stage
		//packet reached destination group non-minimally so deadlock VC should 1 in going eject stage
		hdr->deadlock_vc = 1;
		*w1=*w1+1;
        int dstA = ej_addr % dfly_->a();
        hdr->edge_port = dstA;
      } 
	  else if (hdr->stage_number == valiant_stage1) {
		  int port_no;
		  std::vector<int> con; 
	    for(int i=0;i<groupConnections.size();i++)
	    {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	    }
		if(con.size()==0){
		  //down route packet
          //packet goes from intermediate spine to intermediate leaf so no deadlock VC change
		  *w1=*w1+1;
		  int dstA = rand() % dfly_->a();
		  hdr->stage_number = valiant_stage2;
          hdr->edge_port = dstA;
		}
		else{
		//packet goes directly from intermediate spine to destination spine so deadlock VC should 1
        hdr->deadlock_vc = 1;
		int con_index = rand()%con.size();
	    port_no = con[con_index];
	    int port = (port_no%dfly_->h())+ dfly_->a();
	    hdr->edge_port = port;
		hdr->stage_number = final_stage;
		}
      } else if(hdr->stage_number == valiant_stage2){
		//packet goes from intermediate spine to destination spine after down routing so deadlock VC should 1
	    hdr->deadlock_vc = 1;	
        int port_no;
	    std::vector<int> con; 
	    for(int i=0;i<groupConnections.size();i++)
	    {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	    }
	    int con_index = rand()%con.size();
	    port_no = con[con_index];
	    int port = (port_no%dfly_->h())+ dfly_->a();
	    hdr->edge_port = port;
	    hdr->stage_number = final_stage;
		
	  }
	  else if(hdr->stage_number ==valiant_stage11)//source spine with nonminimal
	  {
        std::vector<std::pair<int,int>> groupConnections1;
		dfly_->groupWiring()->connectedToGroup(my_g_, hdr->randomG, groupConnections1);
		int port_no;
	  std::vector<int> con; 
	  for(int i=0;i<groupConnections1.size();i++)
	  {
		  if(groupConnections1[i].first == my_a)
		  {
			  con.push_back(groupConnections1[i].second);
		  }
	  }
	  int con_index = rand()%con.size();
	  port_no = con[con_index];
      int nonminimalPort = (port_no%dfly_->h())+ dfly_->a();
	  hdr->stage_number = valiant_stage1;
      hdr->edge_port = nonminimalPort; 
	  }
	  else { //source spine minimal
	  int port_no;
	  std::vector<int> con; 
	  for(int i=0;i<groupConnections.size();i++)
	  {
		  if(groupConnections[i].first == my_a)
		  {
			  con.push_back(groupConnections[i].second);
		  }
	  }
	  int con_index = rand()%con.size();
	  port_no = con[con_index];
      int minimalPort = (port_no%dfly_->h())+ dfly_->a();
      hdr->edge_port = minimalPort;
      }
    }
  }

  int numVC() const override {
    return 2;
  }

 private:
  DragonflyPlus* dfly_;
  int rotate_g;
  int my_a;
  int my_row_;
  int my_g_;
  int up_rotater_;
  int num_leaf_switches_;
  std::vector<int> grp_rotaters_;
  int covering_;

};

//..............................................................................................
class DragonflyPlusFPAR : public DragonflyPlusAlltoallMinimalRouter {
  struct header : public DragonflyPlusAlltoallMinimalRouter::header {
    uint8_t stage_number : 4;
	int randomG;
  };
 public:
  SST_ELI_REGISTER_DERIVED(
    Router,
    DragonflyPlusFPAR,
    "macro",
    "dragonfly_plus_FPAR",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "router implementing FPAR for dragonfly+")
	
	static const char initial_stage = 1;
    static const char valiant_stage = 2;
	static const char final_stage = 3;

  std::string toString() const override {
    return "dragonfly+ FPAR router";
  }
DragonflyPlusFPAR(SST::Params& params, Topology *top,
                       NetworkSwitch *netsw)
    : DragonflyPlusAlltoallMinimalRouter(params, top, netsw)
  {
    dfly_ = safe_cast(DragonflyPlus, top);
    my_row_ = my_addr_ / dfly_->numLeafSwitches();
    my_g_ = (my_addr_ % dfly_->numLeafSwitches()) / dfly_->a();
	my_a = dfly_->computeA(my_addr_);
    //covering_ = dfly_->h() / (dfly_->g() - 1);
    grp_rotaters_.resize(dfly_->g());
    for (int i=0; i < dfly_->g(); ++i){
      grp_rotaters_[i] = 0;
    }
    up_rotater_ = 0;
	rotate_r = 0;
	rotate_g = 0;
    num_leaf_switches_ = dfly_->numLeafSwitches();all_link_state = dfly_->global_array();
	
  }
 void route(Packet *pkt) override {
	 
    //long long int *x1= dfly_->total_minimal_path();
    //long long int *y1= dfly_->total_non_minimal_path(); 
    //long long int *z1= dfly_->total_local_up_path();
    //long long int *w1= dfly_->total_local_down_path();
	
	//int T = dfly_->get_queue_threshold(); //new
          int T = 6;
    SwitchId ej_addr = pkt->toaddr() / dfly_->concentration();
	SwitchId sr_addr = pkt->fromaddr() / dfly_->concentration(); //new
    auto hdr = pkt->rtrHeader<header>();
	
	int dstG = (ej_addr % num_leaf_switches_) / dfly_->a();
	int srcG = (sr_addr % num_leaf_switches_) / dfly_->a(); //new
	std::vector<std::pair<int,int>> groupConnections;
	if(my_g_ != dstG){
	dfly_->groupWiring()->connectedToGroup(my_g_, dstG, groupConnections);
	if (groupConnections.size() == 0){
        spkt_abort_printf("Got zero group connections from %d->%d", my_g_, dstG);
      }
	}
	
	
    if (my_row_ == 0){ 
	      
		  if(sr_addr == my_addr_){//source leaf
     
      if (ej_addr == my_addr_){ // Destination under leaf: min hop down
        hdr->edge_port = pkt->toaddr() % dfly_->concentration() + dfly_->a();
      } 
	  else{
		//*z1=*z1+1;
        if (my_g_ == dstG) // Destination in Group : Adaptive Up
        {
            //rter_debug("routing up on %d", up_rotater_);
			hdr->deadlock_vc = 0;
			hdr->stage_number = initial_stage;
            hdr->edge_port = up_rotater_;
            up_rotater_ = (up_rotater_ + 1) % dfly_->a();
        }
        else{ // Destination in other Group : Min hop Up (packet sent to those routers that have a global connection to destination)
		    hdr->deadlock_vc = 0;
			hdr->stage_number = initial_stage;
			hdr->edge_port = groupConnections[rotate_g].first;
            rotate_g = (rotate_g + 1) % groupConnections.size();
		}
	  }       
    }	
	else{ //intermediate leaf or destination leaf
	    //*z1=*z1+1;
		if (ej_addr == my_addr_){ // Destination leaf
        hdr->edge_port = pkt->toaddr() % dfly_->concentration() + dfly_->a();
        }
		else{ // intermediate leaf
		hdr->deadlock_vc = 1;
		hdr->stage_number = final_stage;
		hdr->edge_port = groupConnections[rotate_g].first;
        rotate_g = (rotate_g + 1) % groupConnections.size();
		}
	 }
	}
	else { // spine router 

      if (my_g_ == dstG && hdr->stage_number == initial_stage){ // Packet arrives directly. Destination in group : static min hop down
        //go down to the eject stage
		//*w1=*w1+1;
		hdr->deadlock_vc = 0;
        int dstA = ej_addr % dfly_->a();
        hdr->edge_port = dstA;
      }
	  else if (my_g_ == dstG && hdr->stage_number != initial_stage){ // Packet arrives via intermediate. Destination in group : static min hop down
        //go down to the eject stage
		//*w1=*w1+1;
		hdr->deadlock_vc = 1;
        int dstA = ej_addr % dfly_->a();
        hdr->edge_port = dstA;
      }
	  else if (my_g_ == srcG){ // source spine router 
	  
        // find all minimal ports to destination group from this router
		std::vector<int> con; 
	    for(int i=0;i<groupConnections.size();i++)
	    {
		    if(groupConnections[i].first == my_a)
		    {
			    con.push_back(groupConnections[i].second);
		    }
	    }
		
		// find all minimal and non-minimal ports. value 1 means minimal port, value 0 means non minimal port
		int ports[dfly_->h()];
		std::vector<int> min_ports; 
		std::vector<int> non_min_ports; 
		for(int i=0;i<dfly_->h();i++)
		{
			ports[i]= 0;
		}
		for (int i=0;i<con.size();i++)
		{
			int con_index = con[i];
			int port = con_index%dfly_->h();
			ports[port] = 1;
			
		}
		for(int i=0;i<dfly_->h();i++)
		{
			if(ports[i]==1) // min port 
			    min_ports.push_back(i+ dfly_->a());
			else
				non_min_ports.push_back(i+ dfly_->a());
		}
		
		// find a minimal port that has queue length less than threshold value
		int flag = 0;
		
		//generate random permutation of min ports
		for(int i=0;i<min_ports.size();i++)
        {
		    int index = rand()%(min_ports.size()-i) + i;
            int temp = min_ports[i];
            min_ports[i]=min_ports[index];
            min_ports[index]=temp;
        }
			
		for (int i=0;i<min_ports.size();i++)
		{
			
			int port = min_ports[i];
			
			if(netsw_->queueLength(port, 0)<T) // route minimally from source spine to destination spine
			{
				hdr->deadlock_vc = 0;
				hdr->stage_number = initial_stage;
				hdr->edge_port = port;
				flag = 1;
				break;
			}
		}
		
		if(flag==0) // all minimal path has congestion greater than T. route non minimally
		{
			int port = rand()% non_min_ports.size(); // randomly select a non minimal port 
			hdr->deadlock_vc = 0;
			hdr->stage_number = valiant_stage;
			hdr->edge_port = non_min_ports[port];	
		}
      }
	  
	  else { // intermediate spine router
	  
	    // find all minimal ports to destination group from this router
		std::vector<int> con; 
	    for(int i=0;i<groupConnections.size();i++)
	    {
		    if(groupConnections[i].first == my_a)
		    {
			    con.push_back(groupConnections[i].second);
		    }
	    }
		
		if(con.size()==0){ // no conncetion from this router to destination group. Must down route the packet
		  int dstA = rand() % dfly_->a();
		  hdr->deadlock_vc = 0;
          hdr->edge_port = dstA;
		}
		else{// router has a connection to destination group
		  // find all minimal and non-minimal ports. value 1 means minimal port, value 0 means non minimal port
		  int ports[dfly_->h()];
		  std::vector<int> min_ports; 
		  std::vector<int> non_min_ports; 
		  for(int i=0;i<dfly_->h();i++)
		  {
			  ports[i]= 0;
		  }
		  for (int i=0;i<con.size();i++)
		  {
			  int con_index = con[i];
			  int port = con_index%dfly_->h();
			  ports[port] = 1;
			
		  }
		  for(int i=0;i<dfly_->h();i++)
		  {
			  if(ports[i]==1) // min port 
			      min_ports.push_back(i+ dfly_->a());
			  else
				  non_min_ports.push_back(i+ dfly_->a());
		  }
		  
		  if(hdr->stage_number == valiant_stage){ // first time routed non-minimally
		  
		    // find a minimal port that has queue length less than threshold value
		    int flag = 0;
			
			//generate random permutation of min ports
			for(int i=0;i<min_ports.size();i++)
            {
				int index = rand()%(min_ports.size()-i) + i;
                int temp = min_ports[i];
                min_ports[i]=min_ports[index];
                min_ports[index]=temp;
            }
		    for (int i=0;i<min_ports.size();i++)
		    {
			
			    int port = min_ports[i];
			
			    if(netsw_->queueLength(port, 1)<T) // route directly from intermediate spine to destination spine
			    {
				    hdr->deadlock_vc = 1;
				    hdr->edge_port = port;
				    flag = 1;
				    break;
			    }
		    }
		    if(flag==0) // all direct destination paths from intermediate are congested . Down route the packet through intermediate leaf node
		    {
			    int dstA = rand() % dfly_->a();
				hdr->deadlock_vc = 0;
                hdr->edge_port = dstA;	
		    }
		  }
		  else { // already routed through the intermediate leaf. Must route minimally
			int port = rand()% min_ports.size();
			hdr->deadlock_vc = 1;
			hdr->edge_port=min_ports[port];
		  }
			
		}
		  
	  }
	} 
 } 

  int numVC() const override {
    return 2;
  }

 private:
  DragonflyPlus* dfly_;
  int rotate_r;
  int rotate_g;
  int my_row_;
  int my_a;
  int my_g_;
  int up_rotater_;
  int num_leaf_switches_;
  std::vector<int> grp_rotaters_;
  int covering_;
  int x6;  
  mytype all_link_state;
  mytype1 all_link_state_timestamp;

};


}
}
