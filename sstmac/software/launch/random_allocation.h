/*
 *  This file is part of SST/macroscale:
 *               The macroscale architecture simulator from the SST suite.
 *  Copyright (c) 2009 Sandia Corporation.
 *  This software is distributed under the BSD License.
 *  Under the terms of Contract DE-AC04-94AL85000 with Sandia Corporation,
 *  the U.S. Government retains certain rights in this software.
 *  For more information, see the LICENSE file in the top
 *  SST/macroscale directory.
 */

#ifndef random_alloCATION_H
#define random_alloCATION_H

#include <sstmac/software/launch/allocation_strategy.h>

namespace sstmac {
namespace sw {

class random_allocation : public allocation_strategy
{
 public:
  void
  init_factory_params(sprockit::sim_parameters *params);

  virtual
  ~random_allocation() throw ();

  void
  allocate(
    int nnode_requested,
    const node_set& available,
    node_set& allocation) const;

 protected:
  RNG::UniformInteger* rng_;

};


}
} // end of namespace sstmac


#endif // random_alloCATION_H

