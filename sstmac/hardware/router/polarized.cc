// Follow SST/Macro spacing convention in Vim. Indent with 2 spaces.
// vim: set expandtab tabstop=2:
/**
Implementation of Polarized routing for SST/Macro - 12

Cristóbal Camarero in collaboration with PNNL.

**/



#include <sstmac/hardware/router/router.h>
#include <sstmac/hardware/switch/network_switch.h>
#include <sstmac/hardware/topology/topology.h>
#include <sprockit/util.h>
#include <sprockit/sim_parameters.h>
#include <sprockit/keyword_registration.h>
#include <sstmac/hardware/topology/file.h>
#include <sstmac/libraries/nlohmann/json.hpp>
#include <sstream>
#include <fstream>
#include <climits>

RegisterKeywords(
  { "penalty", "Employed by Polarized routing to priorize better routes. Array with penalization[index] being a penalization in units of queueLength for candidates in the index position. " },
  { "compute_distances", "Employed by Polarized routing to compute the distance from Topology::connectedOutports instead of using Topology::numHopsToNode." }
);

namespace sstmac {
namespace hw {

class PolarizedRouter : public Router {
  public:
  SST_ELI_REGISTER_DERIVED(
    Router,
    PolarizedRouter,
    "macro",
    "polarized",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "router implementing Polarized routing")

  struct header : public Packet::Header {
    // There is a memset to 0 on Packet constructor.
    uint8_t num_hops : 4;
  };
  private:
  struct Candidate {
    int port;
    int weight_change;
  };

  public:
    PolarizedRouter(SST::Params& params, Topology* top, NetworkSwitch* netsw)
    : Router(params,top,netsw), topology(top)
    {
      compute_distances = params.find<bool>("compute_distances",false);
      if(compute_distances)build_distance_matrix();
      // Compute the diameter.
      SwitchId n = topology->numSwitches();
      int max_base_distance = 0;
      for( SwitchId src=0;src<n;src++)
          for( SwitchId dst=0;dst<n;dst++)
          {
            int dist = distance(src,dst);
            if(dist>max_base_distance)max_base_distance=dist;
          }
      // Polarized routes are bound by 4D-3 when the diameter is at least 2.
      switch(max_base_distance)
      {
        case 0:
          spkt_abort_printf("diameter is 0??");
        case 1:
          longest_path = 2;
          break;
        default:
          longest_path = 4*max_base_distance-3;
      }
      penalty[0]=0;
      penalty[0]=64;
      penalty[0]=80;
      if (params.contains("penalty")) {
        std::vector<int> pp;
        params.find_array("penalty",pp);
        int end=3;
        if(pp.size()<3)end=pp.size();
        for(int i=0;i<end;i++)penalty[i]=pp[i];
      }
    }
  
  // The maximum value the Router promises to write into `packet.rtrHeader()->deadlock_vc`.
  int numVC() const override {
    return longest_path;
  }
  
  std::string toString() const override {
    return "Polarized router";
  }
  
  void route(Packet *pkt) override
  {
    auto* hdr = pkt->rtrHeader<header>();
    SwitchId dst = topology->endpointToSwitch(pkt->toaddr());
    if (dst == my_addr_){
      std::vector<Topology::InjectionPort> ports_to_node;
      topology->injectionPorts(pkt->toaddr(),ports_to_node);
      int selected_port = 0;//can a node have several ports?
      hdr->edge_port = ports_to_node[selected_port].switch_port;
      hdr->deadlock_vc = 0;
      return;
    }
    SwitchId src = topology->endpointToSwitch(pkt->fromaddr());
    std::vector<Topology::Connection> conns;
    int a = distance(src,my_addr_);
    int b = distance(my_addr_,dst);
    int weight = b - a;
    topology->connectedOutports(my_addr_, conns);
    std::vector<Candidate> candidates;
    for(auto con: conns)
    {
      SwitchId neighbour = con.dst;
      int newa = distance(src,neighbour);
      int newb = distance(neighbour,dst);
      int new_weight = newb - newa;
      bool condition = new_weight <= weight;
      if(new_weight==weight)
      {
        if(a<b) condition = a<newa;
        else condition = newb<b;
      }
      if(condition)
      {
        candidates.push_back(Candidate{con.src_outport,new_weight-weight});
      }
    }
    if(candidates.size()==0)
    {
        spkt_abort_printf("Got zero Polarized egress candidates for packet %p. hop count=%d. src=%d. current=%d. dst=%d.", pkt,hdr->num_hops,src,my_addr_,dst);
    }
    // Compute a shift that would put the best candidate at the 0 penalty index.
    int min_weight_change = INT_MAX;
    for(auto can: candidates)if(can.weight_change<min_weight_change)min_weight_change=can.weight_change;
    int best=-1, best_value=INT_MAX;
    for(int i=0;i<candidates.size();i++)
    {
      Candidate& can = candidates[i];
      int queue_length = netsw_->queueLength(can.port, all_vcs);
      int value = queue_length + penalty[ can.weight_change - min_weight_change ];
      if(value<best_value)
      {
        best=i;
        best_value=value;
      }
    }
    hdr->edge_port = candidates[best].port;
    hdr->deadlock_vc = hdr->num_hops;
    ++hdr->num_hops;
  }

  int distance(SwitchId src, SwitchId dst)
  {
    // We wrap the topology function, to make a potential change to another distance be easier.
    if(compute_distances){
      uint8_t dist = distance_matrix[src][dst];
      if(dist==UINT8_MAX)spkt_abort_printf("Switch %d has no path to %d.",src,dst);
      return dist;
    } else return topology->numHopsToNode(src,dst);
  }

  void build_distance_matrix()
  {
    distance_matrix.clear();
    SwitchId n = topology->numSwitches();
    for(SwitchId src=0;src<n;src++)
    {
      //For each source we perform BFS.
      std::vector<uint8_t> distance_vector(n, UINT8_MAX);
      distance_vector[src] = 0;
      SwitchId queue[n];
      queue[0]=src;
      int queue_read_index=0;
      int queue_write_index=1;
      while(queue_read_index<queue_write_index)
      {
        SwitchId current = queue[queue_write_index];
        queue_read_index++;
        uint8_t alternative_distance = distance_vector[current];
        if(alternative_distance==UINT8_MAX)continue;
        else alternative_distance++;
        std::vector<Topology::Connection> conns;
        topology->connectedOutports(current,conns);
        for(auto con: conns)
        {
          SwitchId neighbour = con.dst;
          if(alternative_distance < distance_vector[neighbour])
          {
            distance_vector[neighbour] = alternative_distance;
            queue[queue_write_index] = neighbour;
            queue_write_index++;
          }
        }
      }
      distance_matrix.push_back( distance_vector );
    }
  }
  
  private:
    Topology* topology;
    uint8_t longest_path;
    int penalty[3];
    bool compute_distances;
    /// When the configuration option compute_distance is set to true, distance_matrix[src][dst] will be the computed distance from SwitchId src to dst.
    /// UINT8_MAX is used for infinite.
    std::vector<std::vector<uint8_t>> distance_matrix;
};

}}

