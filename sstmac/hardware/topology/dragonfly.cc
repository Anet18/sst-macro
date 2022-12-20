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

#include <sstmac/hardware/topology/dragonfly.h>
#include <sstmac/hardware/topology/dfly_group_wiring.h>
#include <sstmac/hardware/router/router.h>
#include <math.h>
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

static const double PI = 3.141592653589793238462;

Dragonfly::Dragonfly(SST::Params& params) :
  CartesianTopology(params)
{
  if (dimensions_.size() != 2){
    spkt_abort_printf("dragonfly topology geometry should have 2 entries: routers per group, num groups");
  }

  static const double TWO_PI = 6.283185307179586;
  vtk_edge_size_ = params.find<double>("vtk_edge_size", 0.25);

  a_ = dimensions_[0];
  g_ = dimensions_[1];

  //determine radius to make x-dim of switches 0.33
  //r*2pi = edge*n - edge here is 25% larger than requested edge size
  vtk_radius_ = (vtk_edge_size_*1.25*a_*g_) / TWO_PI;
  vtk_box_length_ = 0.2*vtk_radius_;

  vtk_group_radians_ = TWO_PI / g_;
  vtk_switch_radians_ = vtk_group_radians_ / a_ / 1.5;

  if (params.contains("group_connections")){
    h_ = params.find<int>("group_connections");
  } else {
    h_ = params.find<int>("h");
  }

  group_wiring_ = sprockit::create<InterGroupWiring>(
   "macro", params.find<std::string>("inter_group", "circulant"), params, a_, g_, h_);
}

void
Dragonfly::endpointsConnectedToInjectionSwitch(SwitchId swaddr,
                                   std::vector<InjectionPort>& nodes) const
{
  nodes.resize(concentration_);
  for (int i = 0; i < concentration_; i++) {
    InjectionPort& port = nodes[i];
    port.nid = swaddr*concentration_ + i;
    port.switch_port = a_ + h_ + i;
    port.ep_port = 0;
  }
}

int
Dragonfly::minimalDistance(SwitchId src, SwitchId dst) const
{
  int srcA, srcG; getCoords(src, srcA, srcG);
  int dstA, dstG; getCoords(dst, dstA, dstG);

  if (srcG == dstG) return 1;

  std::vector<int> connected;
  int directConn = -1;
  group_wiring_->connectedRouters(srcA, srcG, connected);
  for (int i=0; i < connected.size(); ++i){
    if (connected[i] == dst){
      return 1;
    } else if (computeG(connected[i]) == dstG){
      directConn = connected[i];
    }
  }

  if (directConn > 0){
    return 2;
  } else {
    return 3;
  }
}

void
Dragonfly::connectedOutports(SwitchId src, std::vector<Connection>& conns) const
{
  int max_num_conns = (a_ - 1) + h_;
  conns.resize(max_num_conns);

  int myA;
  int myG;
  getCoords(src, myA, myG);

  int cidx = 0;

  for (int a = 0; a < a_; a++){
    if (a != myA){
      SwitchId dst = getUid(a, myG);
      Connection& conn = conns[cidx++];
      conn.src = src;
      conn.dst = dst;
      conn.src_outport = a;
      conn.dst_inport = myA;
    }
  }

  std::vector<int> groupConnections;
  group_wiring_->connectedRouters(myA, myG, groupConnections);
  for (int h = 0; h < groupConnections.size(); ++h){
    SwitchId dst = groupConnections[h];
    int dstG = computeG(dst);
    if (dstG != myG){
      Connection& conn = conns[cidx++];
      conn.src = src;
      conn.dst = dst;
      conn.src_outport = h + a_;
      int dstA = computeA(dst);
      conn.dst_inport = group_wiring_->inputGroupPort(myA, myG, h, dstA, dstG) + a_;

      top_debug("dragonfly (%d,%d:%d)->(%d,%d:%d)",
         myA, myG, conn.src_outport, dstA, dstG, conn.dst_inport);

    }
  }
  conns.resize(cidx);
}

bool
Dragonfly::isCurvedVtkLink(SwitchId sid, int port) const
{
  if (port >= a_){
    return false; //global link - these are straight lines
  } else {
    return true; //local link - these need to be curved
  }
}

Topology::VTKSwitchGeometry
Dragonfly::getVtkGeometry(SwitchId sid) const
{
  /**
   * The switches are arranged in circle. The faces
   * pointing into the circle represent inter-group traffic
   * while the the faces pointing out of the circle are intra-group traffic.
   * The "reference" switch starts at [radius, 0, 0]
   * and has dimensions [1.0,0.25,0.25]
   * This reference switch is rotated by an angle theta determine by the group number
   * and the position of the switch within the group. The radius is chosen to be large
   * enough to fit all switches with an inner size of 2.5
  */

  int myA = computeA(sid);
  int myG = computeG(sid);

  //we need to figure out the radian offset of the group
  double inter_group_offset = myG*vtk_group_radians_;
  double intra_group_start = myA*vtk_switch_radians_;

  double theta = inter_group_offset + intra_group_start;

  /** With no rotation, these are the corners.
   * These will get rotated appropriately */
  double zCorner = 0.0;
  double yCorner = 0.0;
  double xCorner = vtk_radius_;

  double xSize = vtk_box_length_;
  double ySize = 0.25; //this is the face pointing "into" the circle
  double zSize = 0.25;

  std::vector<VTKSwitchGeometry::port_geometry> ports(a_ + h_ + concentration());
  double port_fraction_a = 1.0 / a_;
  double port_fraction_h = 1.0 / h_;
  double port_fraction_c = 1.0 / concentration();

  for (int a=0; a < a_; ++a){
    VTKSwitchGeometry::port_geometry& p = ports[a];
    p.x_offset = 1.0;
    p.x_size = -0.3;
    p.y_offset = a * port_fraction_a;
    p.y_size = port_fraction_a;
    p.z_offset = 0;
    p.z_size = 1.0;
  }

  for (int h=0; h < h_; ++h){
    VTKSwitchGeometry::port_geometry& p = ports[a_ + h];
    p.x_offset = 0;
    p.x_size = 0.3;
    p.y_offset = h * port_fraction_h;
    p.y_size = port_fraction_h;
    p.z_offset = 0;
    p.z_size = 1.0;
  }


  for (int c=0; c < concentration(); ++c){
    VTKSwitchGeometry::port_geometry& p = ports[a_ + h_ + c];
    p.x_offset = 0.35;
    p.x_size = 0.35;
    p.y_offset = c * port_fraction_c;
    p.y_size = port_fraction_c;
    p.z_offset = 0;
    p.z_size = 1.0;
  }

  VTKSwitchGeometry geom(xSize, ySize, zSize,
                           xCorner, yCorner, zCorner, theta,
                           std::move(ports));

  return geom;
}



}
} //end of namespace sstmac
