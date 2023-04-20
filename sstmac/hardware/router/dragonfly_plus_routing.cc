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
  struct header : public Packet::Header {
   uint8_t q_length=0;
    uint8_t st=0;
};

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

    covering_ = dfly_->h() / (dfly_->g() - 1);
    if (covering_ == 0){
      spkt_abort_printf("dragonfly+ minimal router for alltoall wiring does"
                        " not have full covering");
    }

    int mod = dfly_->h() % (dfly_->g() - 1);
    if (mod != 0){
      spkt_abort_printf("dragonfly+ group connections h=%d is not evenly divided by N-1 for N=%d groups",
                        dfly_->h(), dfly_->g());
    }

    grp_rotaters_.resize(dfly_->g());
    for (int i=0; i < dfly_->g(); ++i){
      grp_rotaters_[i] = 0;
    }
    int x;
    my_g_ = (my_addr_%num_leaf_switches_) / dfly_->a();
    my_row_ = my_addr_ / num_leaf_switches_;

   //figure out which groups it is directly connected to and check of correctness
    std::vector<int> connected;
    int my_a = dfly_->computeA(my_addr_);
    dfly_->groupWiring()->connectedRouters(my_a, my_g_, connected);
    for (int p=0; p < connected.size(); ++p){
      int my_expected_g = p / covering_;
      if (my_expected_g >= my_g_){
        ++my_expected_g;
      }
      int dst = connected[p];
      int actual_g = dfly_->computeG(dst);
      if (my_expected_g != actual_g){
        spkt_abort_printf("Router %d expected group %d on port %d, but got group %d",
                          my_addr_, my_expected_g, p, actual_g);
      }
    }

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
    long long int  *hops = dfly_->hop_array();
    long long int *ql = dfly_->q_l_array();
    SwitchId ej_addr = pkt->toaddr() / dfly_->concentration();
    if (ej_addr == my_addr_){

      if(hdr->st==0){
       *(hops+0)= *(hops+0)+1;
     }else{
       *(hops+hdr->st)= *(hops+hdr->st)+1;
     }

      int e_port = pkt->toaddr() % dfly_->concentration() + dfly_->a();
      hdr->q_length=hdr->q_length+netsw_->queueLength(e_port, all_vcs);
           *(ql+hdr->q_length)= *(ql+hdr->q_length)+1;

      hdr->edge_port = pkt->toaddr() % dfly_->concentration() + dfly_->a();
      hdr->deadlock_vc = 0;
      return;
    }

    int dstG = (ej_addr % num_leaf_switches_) / dfly_->a();
    if (my_row_ == 0){
      if (static_route_){
        hdr->st=hdr->st+1;
        int e_port = ej_addr % dfly_->a();
        hdr->q_length=hdr->q_length+netsw_->queueLength(e_port, all_vcs);
        hdr->edge_port = ej_addr % dfly_->a();
      } else {
        hdr->st=hdr->st+1;
        int e_port = rotater_;
        hdr->q_length=hdr->q_length+netsw_->queueLength(e_port, all_vcs);
        hdr->edge_port = rotater_;
        rotater_ = (rotater_ + 1) % dfly_->a();
      }
    } else if (my_g_ == dstG){
      hdr->st=hdr->st+1;
      int dstA = ej_addr % dfly_->a();
      int e_port = dstA;
      hdr->q_length=hdr->q_length+netsw_->queueLength(e_port, all_vcs);
      hdr->edge_port = dstA;
    } else {
      int rotater;
      int grpOffset = my_g_ < dstG ? dstG - 1 : dstG;
      if (static_route_){
        rotater = ej_addr % covering_;
      } else {
        rotater = grp_rotaters_[dstG];
        grp_rotaters_[dstG] = (grp_rotaters_[dstG] + 1) % covering_;
      }
      hdr->st=hdr->st+1;
      int port = grpOffset*covering_ + rotater + dfly_->a();
      hdr->q_length=hdr->q_length+netsw_->queueLength(port, all_vcs);
      hdr->edge_port = port;
    }
    hdr->deadlock_vc = 0;
  }

 protected:
  int num_leaf_switches_;
  int rotater_;
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
  static const char valiant_stage = 1;
  static const char final_stage = 2;

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
    covering_ = dfly_->h() / (dfly_->g() - 1);
    grp_rotaters_.resize(dfly_->g());
    for (int i=0; i < dfly_->g(); ++i){
      grp_rotaters_[i] = 0;
    }
    up_rotater_ = 0;
    num_leaf_switches_ = dfly_->numLeafSwitches();
  }

  void route(Packet *pkt) override {
    SwitchId ej_addr = pkt->toaddr() / dfly_->concentration();
     //int *x1= dfly_->total_minimal_path();
     //int *y1= dfly_->total_non_minimal_path();
    auto hdr = pkt->rtrHeader<header>();
    long long int  *hops = dfly_->hop_array();
    long long int *ql = dfly_->q_l_array();

    if (my_row_ == 0){
      if (ej_addr == my_addr_){

       if(hdr->st==0){
       *(hops+0)= *(hops+0)+1;
       }else{
       *(hops+hdr->st)= *(hops+hdr->st)+1;
       }
        int e_port = pkt->toaddr() % dfly_->concentration() + dfly_->a();
        hdr->q_length=hdr->q_length+netsw_->queueLength(e_port, all_vcs);
           *(ql+hdr->q_length)= *(ql+hdr->q_length)+1;
        hdr->edge_port = pkt->toaddr() % dfly_->concentration() + dfly_->a();
        hdr->deadlock_vc = 0;
      } else {
        //gotta route up
        //rter_debug("routing up on %d", up_rotater_);
        hdr->st=hdr->st+1;
        int e_port = up_rotater_;
        hdr->q_length=hdr->q_length+netsw_->queueLength(e_port, all_vcs);
        hdr->edge_port = up_rotater_;
        up_rotater_ = (up_rotater_ + 1) % dfly_->a();
        hdr->deadlock_vc = 0;
      }
    } else {
      int dstG = (ej_addr % num_leaf_switches_) / dfly_->a();
      if (my_g_ == dstG){
        //go down to the eject stage
        int dstA = ej_addr % dfly_->a();
        hdr->st=hdr->st+1;
        int e_port = dstA;
        hdr->q_length=hdr->q_length+netsw_->queueLength(e_port, all_vcs);
        hdr->edge_port = dstA;
        //don't change the vc
        //rter_debug("routing down to %d", int(hdr->edge_port));
      } else if (hdr->stage_number == valiant_stage) {
        int grpOffset = my_g_ < dstG ? dstG - 1 : dstG;
        int port = grpOffset*covering_ + grp_rotaters_[dstG] + dfly_->a();
        grp_rotaters_[dstG] = (grp_rotaters_[dstG] + 1) % covering_;
        hdr->st=hdr->st+1;
        hdr->q_length=hdr->q_length+netsw_->queueLength(port, all_vcs);
        hdr->edge_port = port;
        hdr->deadlock_vc = 1; //yep - here now
        //rter_debug("continuing non-minimal path on %d", port);
      } else {
        int minimalPort = grp_rotaters_[dstG] + dfly_->a()
            + ((my_g_ < dstG ? (dstG - 1) : dstG)*covering_);
        if (dfly_->g() > 2){
          //we must have an intermediate group - otherwise we just have minimal
          //we must make a ugal decision here
          int interG = my_g_;
          int valiantPort;
          uint32_t attempt = 0;
          uint32_t seed = netsw_->now().time.ticks();
          int numTestPorts = covering_ * dfly_->g();
          while (interG == my_g_ || interG == dstG){
            valiantPort = randomNumber(numTestPorts, attempt, seed);
            ++attempt;
            interG = valiantPort / covering_;
            //a little weird - we skip ports to ourselves
            if (interG > my_g_) valiantPort -= covering_;
          }
          valiantPort += dfly_->a(); //global ports offset by a


          int valiantMetric = 2*netsw_->queueLength(valiantPort, all_vcs);
          int minimalMetric = netsw_->queueLength(minimalPort, all_vcs);

          //rter_debug("comparing minimal(%d) %d against non-minimal(%d) %d",
                     //minimalPort, minimalMetric, valiantPort, valiantMetric);

          if (minimalMetric <= valiantMetric){
               // *x1=*x1+1;
                //rter_debug("Total minimal path x = %d",*x1);
            hdr->st=hdr->st+1;
            hdr->q_length=hdr->q_length+netsw_->queueLength( minimalPort, all_vcs);
            hdr->edge_port = minimalPort;
            hdr->stage_number = final_stage;
            grp_rotaters_[dstG] = (grp_rotaters_[dstG] + 1) % covering_;
          } else {
              //*y1=*y1+1;
              //rter_debug("Total non-minimal path y = %d",*y1);
            hdr->st=hdr->st+1;
            hdr->q_length=hdr->q_length+netsw_->queueLength(valiantPort, all_vcs);
            hdr->edge_port = valiantPort;
            hdr->stage_number = valiant_stage;
          }
        } else { //no intermediate group - must go minimal
          hdr->st=hdr->st+1;
          hdr->q_length=hdr->q_length+netsw_->queueLength( minimalPort, all_vcs);
          hdr->edge_port = minimalPort;
          hdr->stage_number = final_stage;
          grp_rotaters_[dstG] = (grp_rotaters_[dstG] + 1) % covering_;
        }
        hdr->deadlock_vc = 0;
      }
    }
  }

  int numVC() const override {
    return 2;
  }

 private:
  DragonflyPlus* dfly_;
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
  static const char final_stage = 3;

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
    covering_ = dfly_->h() / (dfly_->g() - 1);
    grp_rotaters_.resize(dfly_->g());
    for (int i=0; i < dfly_->g(); ++i){
      grp_rotaters_[i] = 0;
    }
    up_rotater_ = 0;
    num_leaf_switches_ = dfly_->numLeafSwitches();
  }

  void route(Packet *pkt) override {
     //Global array to store piggyback information
     typedef int (*mytype)[2200][64];
     mytype all_link_state = dfly_->global_array();
     //int *x1= dfly_->total_minimal_path();
     //int *y1= dfly_->total_non_minimal_path();

    SwitchId ej_addr = pkt->toaddr() / dfly_->concentration();
    auto hdr = pkt->rtrHeader<header>();
    long long int  *hops = dfly_->hop_array();
    long long int *ql = dfly_->q_l_array();

    if (my_row_ == 0){
      if (ej_addr == my_addr_){

        if(hdr->st==0){
       *(hops+0)= *(hops+0)+1;
       }else{
       *(hops+hdr->st)= *(hops+hdr->st)+1;
       }
        int e_port = pkt->toaddr() % dfly_->concentration() + dfly_->a();
        hdr->q_length=hdr->q_length+netsw_->queueLength(e_port, all_vcs);
           *(ql+hdr->q_length)= *(ql+hdr->q_length)+1;

        hdr->edge_port = pkt->toaddr() % dfly_->concentration() + dfly_->a();
        hdr->deadlock_vc = 0;
      } else {
        //gotta route up
        //............................................
        int dstG = (ej_addr % num_leaf_switches_) / dfly_->a();
        if (my_g_ == dstG)
        {
            //rter_debug("routing up on %d", up_rotater_);
            hdr->st=hdr->st+1;
            int e_port = up_rotater_;
            hdr->q_length=hdr->q_length+netsw_->queueLength(e_port, all_vcs);
            hdr->edge_port = up_rotater_;
            up_rotater_ = (up_rotater_ + 1) % dfly_->a();
            hdr->deadlock_vc = 0;
        }
        else
        {
            //piggyback stage
            int f=0;
            int ct=0;
            vector < int > uncongested;
            vector < int > val;
            //find the connected routers
            for (int a=0; a < dfly_->a(); ++a)
            {
                int dest=dfly_->getUid(1,a,my_g_);
                int dest_port= grp_rotaters_[dstG] + dfly_->a()
            + ((my_g_ < dstG ? (dstG - 1) : dstG)*covering_);
			    grp_rotaters_[dstG] = (grp_rotaters_[dstG] + 1) % covering_;
               if((*all_link_state)[dest][dest_port]==1)
               {
                   ct++;
                   val.push_back(a);
               }
               else
               {
                   uncongested.push_back(a);
               }
            }
            if(ct==dfly_->a())
            {
                //all link are congested need to go valiant state
                //rter_debug("routing up on %d", up_rotater_);
                hdr->st=hdr->st+1;
                int e_port = up_rotater_;
                hdr->q_length=hdr->q_length+netsw_->queueLength(e_port, all_vcs);
                hdr->edge_port = up_rotater_;
                up_rotater_ = (up_rotater_ + 1) % dfly_->a();
                hdr->deadlock_vc = 0;
                hdr->stage_number = valiant_stage1;
                //*y1=*y1+1;
                //rter_debug("Total non-minimal path y = %d",*y1);

            }
            else
            {
                //randomly pick any K minimal global connected router
                int RANDOM = 2;
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
                //*x1=*x1+1;
                //rter_debug("Total minimal path x = %d",*x1);
                                hdr->st=hdr->st+1;
                                hdr->q_length=hdr->q_length+netsw_->queueLength( min_random_minimal, all_vcs);
				hdr->edge_port = min_random_minimal;
            }


        }
      }
    } else {
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
                int x= netsw_->queueLength(j+dfly_->a(), all_vcs);
                if(x>2*average_queue_length)
                {
                    (*all_link_state)[my_addr_][j+dfly_->a()]=1;
                }
                else
                {
                    (*all_link_state)[my_addr_][j+dfly_->a()]=0;
                }
        }
      int dstG = (ej_addr % num_leaf_switches_) / dfly_->a();
      if (my_g_ == dstG){
        //go down to the eject stage
        int dstA = ej_addr % dfly_->a();
        hdr->st=hdr->st+1;
        hdr->q_length=hdr->q_length+netsw_->queueLength(dstA, all_vcs);
        hdr->edge_port = dstA;
        //don't change the vc
        //rter_debug("routing down to %d", int(hdr->edge_port));
      }
      else if(hdr->stage_number == valiant_stage1)
	  {
		int grpOffset = my_g_ < dstG ? dstG - 1 : dstG;
        int port = grpOffset*covering_ + grp_rotaters_[dstG] + dfly_->a();
        grp_rotaters_[dstG] = (grp_rotaters_[dstG] + 1) % covering_;
		vector < int > intermediate_groups;
		for(int i=0;i<dfly_->a();i++)
		{
			int port_i=i+dfly_->a();
			if(port_i!=port)
			{
				intermediate_groups.push_back(port_i);
			}
		}
		//srand(time(NULL));
		int port_inter= rand()%intermediate_groups.size();
		hdr->stage_number = valiant_stage2;
                hdr->st=hdr->st+1;
                int e_port = intermediate_groups[port_inter];
                hdr->q_length=hdr->q_length+netsw_->queueLength(e_port, all_vcs);
		hdr->edge_port = intermediate_groups[port_inter];
		//rter_debug("continuing non-minimal path from %d to %d on port %d not port %d",my_g_,dstG,intermediate_groups[port_inter],port);
	  }
	  else if (hdr->stage_number == valiant_stage2) {
        int grpOffset = my_g_ < dstG ? dstG - 1 : dstG;
        int port = grpOffset*covering_ + grp_rotaters_[dstG] + dfly_->a();
        grp_rotaters_[dstG] = (grp_rotaters_[dstG] + 1) % covering_;
        hdr->st=hdr->st+1;
        hdr->q_length=hdr->q_length+netsw_->queueLength(port, all_vcs);
        hdr->edge_port = port;
        hdr->deadlock_vc = 1; //yep - here now
        //rter_debug("continuing non-minimal path from %d to %d on port %d ",my_g_,dstG,port);
      } else {
        int minimalPort = grp_rotaters_[dstG] + dfly_->a()
            + ((my_g_ < dstG ? (dstG - 1) : dstG)*covering_);
        hdr->st=hdr->st+1;
        hdr->q_length=hdr->q_length+netsw_->queueLength(minimalPort, all_vcs);
        hdr->edge_port = minimalPort;
        hdr->deadlock_vc = 0;
     }
    }
  }

  int numVC() const override {
    return 2;
  }

 private:
  DragonflyPlus* dfly_;
  int my_row_;
  int my_g_;
  int up_rotater_;
  int num_leaf_switches_;
  std::vector<int> grp_rotaters_;
  int covering_;

};

//........................................................................................
class DragonflyPlusPiggyBackRouter2 : public DragonflyPlusAlltoallMinimalRouter {
  struct header : public DragonflyPlusAlltoallMinimalRouter::header {
    uint8_t stage_number : 4;
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
  static const char valiant_stage2 = 2;
  static const char final_stage = 3;

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
    covering_ = dfly_->h() / (dfly_->g() - 1);
    grp_rotaters_.resize(dfly_->g());
    for (int i=0; i < dfly_->g(); ++i){
      grp_rotaters_[i] = 0;
    }
    up_rotater_ = 0;
    num_leaf_switches_ = dfly_->numLeafSwitches();
  }
 void route(Packet *pkt) override {
     //Global array to store piggyback information
     //typedef int (*mytype)[2200][64];
       typedef int (*mytype)[2200][64];
     mytype all_link_state = dfly_->global_array();

    SwitchId ej_addr = pkt->toaddr() / dfly_->concentration();
    auto hdr = pkt->rtrHeader<header>();
    long long int  *hops = dfly_->hop_array();
    long long int *ql = dfly_->q_l_array();

    if (my_row_ == 0){
      if (ej_addr == my_addr_){

        if(hdr->st==0){
       *(hops+0)= *(hops+0)+1;
       }else{
       *(hops+hdr->st)= *(hops+hdr->st)+1;
       }
        int e_port = pkt->toaddr() % dfly_->concentration() + dfly_->a();
        hdr->q_length=hdr->q_length+netsw_->queueLength(e_port, all_vcs);
           *(ql+hdr->q_length)= *(ql+hdr->q_length)+1;

        hdr->edge_port = pkt->toaddr() % dfly_->concentration() + dfly_->a();
        hdr->deadlock_vc = 0;
      } else {
        //gotta route up
        //............................................
        int dstG = (ej_addr % num_leaf_switches_) / dfly_->a();
        if (my_g_ == dstG)
        {
            //rter_debug("routing up on %d", up_rotater_);
            hdr->st=hdr->st+1;
            int e_port = up_rotater_;
            hdr->q_length=hdr->q_length+netsw_->queueLength(e_port, all_vcs);
            hdr->edge_port = up_rotater_;
            up_rotater_ = (up_rotater_ + 1) % dfly_->a();
            hdr->deadlock_vc = 0;
        }
        else
        {
            //piggyback stage
            int ct=0;
            vector < int > uncongested;
            vector < int > val;
            //find the connected routers
            for (int a=0; a < dfly_->a(); ++a)
            {
                int dest=dfly_->getUid(1,a,my_g_);
                int dest_port= grp_rotaters_[dstG] + dfly_->a()
            + ((my_g_ < dstG ? (dstG - 1) : dstG)*covering_);
                            grp_rotaters_[dstG] = (grp_rotaters_[dstG] + 1) % covering_;
               if((*all_link_state)[dest][dest_port]==1)
               {
                   ct++;
                   val.push_back(a);
               }
               else
               {
                   uncongested.push_back(a);
               }
            }
			if(ct==dfly_->a())
            {
                //all link are congested need to go valiant state
                //rter_debug("routing up on %d", up_rotater_);
                hdr->st=hdr->st+1;
                int e_port = up_rotater_;
                hdr->q_length=hdr->q_length+netsw_->queueLength(e_port, all_vcs);
                hdr->edge_port = up_rotater_;
                up_rotater_ = (up_rotater_ + 1) % dfly_->a();
                hdr->deadlock_vc = 0;
                hdr->stage_number = valiant_stage1;

            }
            else{
            //randomly pick any K minimal global connected router
            int RANDOM = 2;
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
             randG=rand()%(dfly_->a()+1);
            }while(randG==dstG);

            int ct1=0;
            vector < int > uncongested1;
            vector < int > val1;
            //find the connected routers
            for (int a=0; a < dfly_->a(); ++a)
            {
                int dest=dfly_->getUid(1,a,my_g_);
                int dest_port= grp_rotaters_[randG] + dfly_->a()
            + ((my_g_ < randG ? (randG - 1) : randG)*covering_);
                            grp_rotaters_[randG] = (grp_rotaters_[randG] + 1) % covering_;
               if((*all_link_state)[dest][dest_port]==1)
               {
                   ct1++;
                   val1.push_back(a);
               }
               else
               {
                   uncongested1.push_back(a);
               }
            }
            if(ct1==dfly_->a())
            {
                hdr->st=hdr->st+1;
                hdr->q_length=hdr->q_length+netsw_->queueLength(min_random_minimal, all_vcs);
                hdr->edge_port = min_random_minimal;
            }
            else{
            //randomly pick any K minimal global connected router
            int RANDOM1 = 2;
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
            if(netsw_->queueLength(min_random_minimal, all_vcs)>2*netsw_->queueLength(min_random_minimal1, all_vcs))
            {
                hdr->st=hdr->st+1;
                hdr->q_length=hdr->q_length+netsw_->queueLength(min_random_minimal, all_vcs);
                hdr->edge_port = min_random_minimal;
            }
            else
            {
                hdr->st=hdr->st+1;
                hdr->q_length=hdr->q_length+netsw_->queueLength(min_random_minimal1, all_vcs);
                hdr->edge_port = min_random_minimal1;
                hdr->stage_number = valiant_stage1;
            }
            }
          }
        }
      }
    } else {
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
                int x= netsw_->queueLength(j+dfly_->a(), all_vcs);
                if(x>2*average_queue_length)
                {
                    (*all_link_state)[my_addr_][j+dfly_->a()]=1;
                }
                else
                {
                    (*all_link_state)[my_addr_][j+dfly_->a()]=0;
                }
        }
		int dstG = (ej_addr % num_leaf_switches_) / dfly_->a();
      if (my_g_ == dstG){
        //go down to the eject stage
        int dstA = ej_addr % dfly_->a();
        hdr->st=hdr->st+1;
        hdr->q_length=hdr->q_length+netsw_->queueLength(dstA, all_vcs);
        hdr->edge_port = dstA;
        //don't change the vc
        //rter_debug("routing down to %d", int(hdr->edge_port));
      }
       else if(hdr->stage_number == valiant_stage1)
          {
                int grpOffset = my_g_ < dstG ? dstG - 1 : dstG;
        int port = grpOffset*covering_ + grp_rotaters_[dstG] + dfly_->a();
        grp_rotaters_[dstG] = (grp_rotaters_[dstG] + 1) % covering_;
                vector < int > intermediate_groups;
                for(int i=0;i<dfly_->a();i++)
                {
                        int port_i=i+dfly_->a();
                        if(port_i!=port)
                        {
                                intermediate_groups.push_back(port_i);
                        }
                }
                //srand(time(NULL));
                int port_inter= rand()%intermediate_groups.size();
                hdr->stage_number = valiant_stage2;
                hdr->st=hdr->st+1;
                int e_port = intermediate_groups[port_inter];
                hdr->q_length=hdr->q_length+netsw_->queueLength(e_port, all_vcs);
                hdr->edge_port = intermediate_groups[port_inter];
                rter_debug("continuing non-minimal path from %d to %d on port %d not port %d",my_g_,dstG,intermediate_groups[port_inter],port);
          }
          else if (hdr->stage_number == valiant_stage2) {
        int grpOffset = my_g_ < dstG ? dstG - 1 : dstG;
        int port = grpOffset*covering_ + grp_rotaters_[dstG] + dfly_->a();
        grp_rotaters_[dstG] = (grp_rotaters_[dstG] + 1) % covering_;
        hdr->st=hdr->st+1;
        hdr->q_length=hdr->q_length+netsw_->queueLength(port, all_vcs);
        hdr->edge_port = port;
        hdr->deadlock_vc = 1; //yep - here now
        rter_debug("continuing non-minimal path from %d to %d on port %d ",my_g_,dstG,port);
      } else {
        int minimalPort = grp_rotaters_[dstG] + dfly_->a()
            + ((my_g_ < dstG ? (dstG - 1) : dstG)*covering_);
        hdr->st=hdr->st+1;
        hdr->q_length=hdr->q_length+netsw_->queueLength(minimalPort, all_vcs);
        hdr->edge_port = minimalPort;
        hdr->deadlock_vc = 0;
     }
    }
  }

  int numVC() const override {
    return 2;
  }

 private:
  DragonflyPlus* dfly_;
  int my_row_;
  int my_g_;
  int up_rotater_;
  int num_leaf_switches_;
  std::vector<int> grp_rotaters_;
  int covering_;

};

}
}
