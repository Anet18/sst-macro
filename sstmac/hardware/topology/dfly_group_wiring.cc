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

#include <sstmac/hardware/topology/dfly_group_wiring.h>
#include <sstmac/hardware/router/router.h>
#include <math.h>
#include <random>
#include <algorithm>
#include <sstream>
#include <sprockit/stl_string.h>
#include <sprockit/output.h>
#include <sprockit/util.h>
#include <sprockit/keyword_registration.h>
#include <sprockit/sim_parameters.h>

RegisterKeywords(
{ "h", "the number inter-group connections per router" },
{ "group_connections", "the number of inter-group connections per router"},
{ "inter_group", "the inter-group wiring scheme"},
);

namespace sstmac {
namespace hw {

InterGroupWiring::InterGroupWiring(SST::Params& params, int a, int g, int h) :
  a_(a), g_(g), h_(h)
{
}

SwitchId
InterGroupWiring::randomIntermediate(Router* rtr, SwitchId current_sw, SwitchId dest_sw, uint32_t seed)
{
  int srcA = current_sw % g_;
  int srcG = current_sw / g_;
  int dstA = dest_sw % g_;
  int dstG = dest_sw / g_;
  SwitchId sid = current_sw;
  uint32_t attempt = 0;
  if (srcG == dstG){
    int interA = srcA;
    while (interA == srcA || interA == dstA){
      interA = rtr->randomNumber(a_, attempt, seed);
      ++attempt;
    }
    return srcG*a_ + interA;
  } else {
    int interA = rtr->randomNumber(a_, attempt, seed);
    int interG = srcG;
    while (interG == srcG || interG == dstG){
      interG = rtr->randomNumber(g_, attempt, seed);
      ++attempt;
    }
    return interG*a_ + interA;
  }
}

static inline int mod(int a, int b){
  int rem = a % b;
  return rem < 0 ? rem + b : rem;
}

class SingleLinkGroupWiring : public InterGroupWiring
{
 public:
  SPKT_REGISTER_DERIVED(
    InterGroupWiring,
    SingleLinkGroupWiring,
    "macro",
    "single",
    "wiring with only one group link on each switch")

  SingleLinkGroupWiring(SST::Params& params, int a, int g, int h) :
    InterGroupWiring(params, a, g, h)
  {
    if (h_ != 1){
      spkt_abort_printf("h must be 1 for single link inter-group pattern");
    }
  }

  int inputGroupPort(int srcA, int srcG, int srcH, int dstA, int dstG) const override {
    if (srcH != 0){
      spkt_abort_printf("h must be 1 for single link inter-group pattern");
    }
    return 0;
  }

  /**
   * @brief connected_routers
   * @param a The src router index within the group
   * @param g The src router group
   * @param connected [in-out] The routers (switch id) for each inter-group interconnection
   * @return The number of routers in connected array
   */
  void connectedRouters(int srcA, int srcG, std::vector<int>& connected) const override {
    int plusMinus = 1 - 2*(srcA%2);
    int deltaG = (1 + srcA/2)*plusMinus;
    int deltaA = plusMinus;
    int aMax = a_ - 1;
    if ( (a_%2) && (srcA == aMax) ){
      deltaA = 0;
    }
    connected.resize(1);
    int dstG = (srcG+g_+deltaG)%g_;
    int dstA = (srcA+a_+deltaA)%a_;
    SwitchId dst = dstG*a_ + dstA;
    connected[0] = dst;
  }

  /**
   * @brief connected_to_group
   * @param srcG
   * @param dstG
   * @param connected [in-out] The list of all intra-group routers in range (0 ... a-1)
   *                  that have connections to a router in group dstG. The second entry in the pair
   *                  is the "port offset", a unique index for the group link
   * @return The number of routers in group srcG with connections to dstG
   */
  void connectedToGroup(int srcG, int dstG, std::vector<std::pair<int,int>>& connected) const override {
    connected.resize(1);
    for (int a=0; a < a_; ++a){
      int plusMinus = 1 - 2*(a%2);
      int deltaG = (1 + a/2)*plusMinus;
      int testG = (srcG + deltaG + g_) % g_;
      if (testG == dstG){
        connected[0] = std::make_pair(a,0);
      }
    }
  }

};

class CirculantGroupWiring : public InterGroupWiring
{
 public:
  SPKT_REGISTER_DERIVED(
    InterGroupWiring,
    CirculantGroupWiring,
    "macro",
    "circulant",
    "wiring with circulant pattern")

  CirculantGroupWiring(SST::Params& params, int a, int g, int h) :
    InterGroupWiring(params, a, g, h)
  {
    if (h_ % 2){
      spkt_abort_printf("group connections must be even for circulant inter-group pattern");
    }
  }

  int inputGroupPort(int srcA, int srcG, int srcH, int dstA, int dstG) const override {
    if (srcH % 2 == 0){
      return srcH + 1;
    } else {
      return srcH - 1;
    }
  }

  /**
   * @brief connected_routers
   * @param a The src router index within the group
   * @param g The src router group
   * @param connected [in-out] The routers (switch id) for each inter-group interconnection
   * @return The number of routers in connected array
   */
  void connectedRouters(int srcA, int srcG, std::vector<int>& connected) const override {
    connected.clear();
    int dstA = srcA;
    int half = h_  /2;
    for (int h=1; h <= half; ++h){
      int plusG = mod(srcG + srcA*half + h, g_);
      if (plusG != srcG){
        SwitchId dst = plusG*a_ + dstA;
        connected.push_back(dst); //full switch ID
      }
      int minusG = mod(srcG - srcA*half - h, g_);
      if (minusG != srcG){
        SwitchId dst = minusG*a_ + dstA;
        connected.push_back(dst); //full switch ID
      }
    }
  }

  /**
   * @brief connected_to_group
   * @param srcG
   * @param dstG
   * @param connected [in-out] The list of all intra-group routers in range (0 ... a-1)
   *                  that have connections to a router in group dstG. The second entry in the pair
   *                  is the "port offset", a unique index for the group link
   * @return The number of routers in group srcG with connections to dstG
   */
  void connectedToGroup(int srcG, int dstG, std::vector<std::pair<int,int>>& connected) const override {
    connected.clear();
    std::vector<int> tmp;
    for (int a=0; a < a_; ++a){
      connectedRouters(a, srcG, tmp);
      for (int c=0; c < tmp.size(); ++c){
        int g = tmp[c] / a_;
        if (dstG == g){
          connected.emplace_back(a,c);
          break;
        }
      }
    }
  }

};

class AllToAllGroupWiring : public InterGroupWiring
{
 public:
  SPKT_REGISTER_DERIVED(
    InterGroupWiring,
    AllToAllGroupWiring,
    "macro",
    "alltoall",
    "wiring with every switch having connection to every group")

  AllToAllGroupWiring(SST::Params& params, int a, int g, int h) :
    InterGroupWiring(params, a, g, h)
  {
    covering_ = h_ / (g_-1);
    if (covering_ == 0){
      spkt_abort_printf("Group connections h=%d is not able to provide all-to-all covering for g=%d",
                        h_, g_);
    }

    stride_ = a_ / covering_;

    top_debug("alltoall links cover groups %dx with stride=%d", covering_, stride_);

    if (h_ % (g_-1)) spkt_abort_printf("dragonfly #groups-1=%d must evenly divide #group_connections=%d", g_-1, h_);
    if (a_ % covering_) spkt_abort_printf("dragonfly covering=%d must evenly divide group_size=%d", covering_, a_);
  }

  int inputGroupPort(int srcA, int srcG, int srcH, int dstA, int dstG) const override {
    int deltaA = (srcA + a_ - dstA) % a_; //mod arithmetic in case srcA < dstA
    int offset = deltaA / stride_;
    if (srcG < dstG){
      return srcG*covering_ + offset;
    } else {
      return (srcG - 1)*covering_ + offset;
    }
  }

  /**
   * @brief connected_routers
   * @param a The src router index within the group
   * @param g The src router group
   * @param connected [in-out] The routers (switch id) for each inter-group interconnection
   * @return The number of routers in connected array
   */
  void connectedRouters(int srcA, int srcG, std::vector<int>& connected) const override {
    connected.clear();
    for (int g=0; g < g_; ++g){
      if (g == srcG) continue;
      for (int c=0; c < covering_; ++c){
        int dstA = (srcA + stride_*c) % a_;
        int dst = g*a_ + dstA;
        connected.push_back(dst);
      }
    }
  }

  SwitchId randomIntermediate(Router* rtr, SwitchId current_sw, SwitchId dest_sw, uint32_t seed) override
  {
    int srcG = current_sw / a_;
    int dstG = dest_sw / a_;
    uint32_t attempt = 0;
    if (srcG == dstG){
      return InterGroupWiring::randomIntermediate(rtr, current_sw, dest_sw, seed);
    } else {
      int srcA = current_sw % a_;
      int interG = rtr->randomNumber(g_, attempt, seed);
      while (interG == srcG || interG == dstG){
        interG = rtr->randomNumber(g_, attempt, seed);
      }
      //go to a router directly connected to srcA
      int srcH = rtr->randomNumber(h_, attempt, seed);
      int interA = (srcH * stride_ + srcA) % a_;
      return interG*a_ + interA;
    }
  }

  /**
   * @brief connected_to_group
   * @param srcG
   * @param dstG
   * @param connected [in-out] The list of all intra-group routers in range (0 ... a-1)
   *                  that have connections to a router in group dstG. The second entry in the pair
   *                  is the "port offset", a unique index for the group link
   * @return The number of routers in group srcG with connections to dstG
   */
  void connectedToGroup(int srcG, int dstG, std::vector<std::pair<int,int>>& connected) const override {
    connected.clear();
    for (int a=0; a < a_; ++a){
      int offset = dstG * covering_;
      for (int c=0; c < covering_; ++c){
        connected.emplace_back(a, offset+c);
      }
    }
  }

 private:
  int covering_;
  int stride_;
};


class RandomGroupWiring : public InterGroupWiring
{
 public:
  SPKT_REGISTER_DERIVED(
    InterGroupWiring,
    RandomGroupWiring,
    "macro",
    "random",
    "random global wiring - guarantees global diameter 1")
  
  RandomGroupWiring(SST::Params& params, int a, int g, int h) :
    InterGroupWiring(params, a, g, h)
  {

    if (a_ * h_ < g_ - 1){
      spkt_abort_printf("Group connections h=%d is not able to provide global dimater 1 for g=%d",
                        h_, g_);
    }

    if (a_ * h_ % (g_ - 1) != 0){
      spkt_abort_printf("Group connections h*a=%d must evenly divide #groups=%d",
                        h_*a_, g_);
    }


    /* For each group g, partition the set of groups other than g, into a subsets 
     * (sets of	groups) of cardinality h. Then assign one subset to each router of g.
     * for every pair of groups A, B, find in A the router assigned to group B and in B
     * the router assigned to group A. Then, add a global link between the routers found.
     */
    
    std::vector<int> subsets;
    std::vector< std::vector<int> > groups;
    //remember to check the seed for this one
    std::random_device rd;
    std::mt19937 rngu(1);

    //int di = (a_ * h_)/(g_ - 1);
    //int re = (a_ * h_)%(g_- 1);
    //int ah;
    //if(re ==0){
      int ah = a_*h_;
    //}
    //else{
      //ah = (di+1)*(g_-1);
    //}
     
    int j = 0;
    for (int i = 0; i < ah; ++i,++j) {
      subsets.push_back(j % (g_-1));
      top_debug("Global Connections for Router %d", j % (g_-1));
    }

    groups.resize(g_);
    for (int i = 0; i < g_; ++i) {
      groups[i].resize(ah);
      std::shuffle(subsets.begin(), subsets.end(), rngu);
      groups[i] = subsets;
      for (j = 0; j < ah; j++)
	top_debug("Global Connections for Group %d: %d", i, groups[i][j]);
    }
    
    mapping_.resize(a_ * g_);
    //for (int i = 0; i < a_ * g_; ++i) {
    //  mapping_[i].resize(h_);
    //}

    for (int i = 0; i < g_; i++) {      
      for (j = 0; j < ah; j++) {
	
	if (groups[i][j] < i)
	  continue;
	
	int dst_g = groups[i][j] + 1;
	int src_r = i * a_ + j / h_;
	
	std::vector<int>::iterator it;
	it = std::find(groups[dst_g].begin(), groups[dst_g].end(), i);

	if (it == groups[dst_g].end())
	  spkt_abort_printf("Error in random group wiring generation");

	int dst_g_port = it - groups[dst_g].begin();
	int dst_r = dst_g * a_ + dst_g_port / h_;

	mapping_[src_r].push_back(dst_r);
	mapping_[dst_r].push_back(src_r);

	//cheaper than removing from vector
	groups[i][j] = -1;
	groups[dst_g][dst_g_port] = -1;
      }
    }
    
    for (int i = 0; i < a_ * g_; i++) {
      top_debug("Global Connections for Router %d", i);
      for (j = 0; j < h_; j++) {
	top_debug("%d", mapping_[i][j]);
      }
    }
    
  }

  int inputGroupPort(int srcA, int srcG, int srcH, int dstA, int dstG) const override {
    //top_debug("inputgroupport");

    //there might be multiple connections between routers.
    //we should find the port connected to srcH in destination group.

    int count = 0;

    /*//adjust router ids if megafly
    if (dimensions_.size() == 3) {
      int row, aa, gg;
      getCoords();
      srcA = ;
      dstA = ;
      }*/
    for (int i=0; i < srcH; ++i) {
      if (mapping_[srcG * a_ + srcA][i] == dstA + dstG * a_) {
	count++;
      }
    }

    int j = 0;
    while (count > 0 && j < h_) {
      if (mapping_[dstG * a_ + dstA][j] == srcG * a_ + srcA)
	count--;
      j++;
    }
    
    if ( j == h_)
      spkt_abort_printf("No global connections from router %d in group %d to router %d in group %d.", srcA, srcG, dstA, dstG);


    for (int k=j; k < h_; ++k)
      if (mapping_[dstG * a_ + dstA][k] == srcG * a_ + srcA)
	return k;

  
    spkt_abort_printf("No global connections from router %d in group %d to router %d in group %d.", srcA, srcG, dstA, dstG);
    return -1;
  }

  /**
   * @brief connected_routers
   * @param a The src router index within the group
   * @param g The src router group
   * @param connected [in-out] The routers (switch id) for each inter-group interconnection
   * @return The number of routers in connected array
   */
  void connectedRouters(int srcA, int srcG, std::vector<int>& connected) const override {
    //top_debug("connectedRouters");
    connected.clear();
    int src_r = srcG * a_ + srcA;
    for (int i=0; i < h_; ++i){
      connected.push_back(mapping_[src_r][i]);
    }
  
  }
  /**
   * @brief connected_to_group
   * @param srcG
   * @param dstG
   * @param connected [in-out] The list of all intra-group routers in range (0 ... a-1)
   *                  that have connections to a router in group dstG. The second entry in the pair
   *                  is the "port offset", a unique index for the group link
   * @return The number of routers in group srcG with connections to dstG
   */
  void connectedToGroup(int srcG, int dstG, std::vector<std::pair<int,int>>& connected) const override {
    //top_debug("connectedToGroup: srcG:%d, dstG:%d", srcG, dstG);
    connected.clear();
    for (int a=0; a < a_; ++a){
      int src_r = srcG * a_ + a;
      for (int p=0; p < h_; ++p){
	if (mapping_[src_r][p] / a_ == dstG)
	  connected.emplace_back(a, a * h_ + p);
      }
    }
  }

 private:
  std::vector< std::vector<SwitchId> > mapping_;
};

}
} //end of namespace sstmac
