/**
Copyright 2009-2017 National Technology and Engineering Solutions of Sandia, 
LLC (NTESS).  Under the terms of Contract DE-NA-0003525, the U.S.  Government 
retains certain rights in this software.

Sandia National Laboratories is a multimission laboratory managed and operated
by National Technology and Engineering Solutions of Sandia, LLC., a wholly 
owned subsidiary of Honeywell International, Inc., for the U.S. Department of 
Energy's National Nuclear Security Administration under contract DE-NA0003525.

Copyright (c) 2009-2017, NTESS

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

    * Neither the name of Sandia Corporation nor the names of its
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

#include <tests/unit_tests/util/util.h>
#include <sprockit/output.h>

void test_torus(UnitTest& unit);
void test_crossbar(UnitTest& unit);
void test_fattree2(UnitTest& unit);
void test_fattree4(UnitTest& unit);
void test_butterfly(UnitTest& unit);
void test_fbfly(UnitTest& unit);
void test_dragonfly_v1(UnitTest& unit);
void test_dragonfly_v2(UnitTest& unit);

using namespace sstmac;
using namespace sstmac::hw;

void
test_topology(sprockit::sim_parameters& params)
{
  topology* top = topology::factory::get_param("name", &params);
  topology::set_static_topology(top);
  interconnect::switch_map switches;
  init_switches(switches, params, top);
  sstmac::env::params = &params;
}

int main(int argc, char** argv)
{
  sprockit::output::init_out0(&std::cout);
  sprockit::output::init_err0(&std::cerr);
  sprockit::output::init_outn(&std::cout);
  sprockit::output::init_errn(&std::cerr);
  UnitTest unit;
  try{
      std::cout << "Testing torus...\n";
          test_torus(unit);
      std::cout << "Testing fat tree...\n";
          //test_fattree2(unit);
          test_fattree4(unit);
      std::cout << "Testing crossbar...\n";
          test_crossbar(unit);
      std::cout << "Testing butterfly...\n";
          test_butterfly(unit);
      std::cout << "Testing fbfly...\n";
          test_fbfly(unit);
      std::cout << "Testing dragonfly...\n";
          test_dragonfly_v1(unit);
          test_dragonfly_v2(unit);
      unit.validate();
  } catch (std::exception& e) {
      cerrn << e.what() << std::endl;
      return 1;
  }

  return 0;
}