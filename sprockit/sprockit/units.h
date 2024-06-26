/**
Copyright 2009-2022 National Technology and Engineering Solutions of Sandia,
LLC (NTESS).  Under the terms of Contract DE-NA-0003525, the U.S. Government
retains certain rights in this software.

Sandia National Laboratories is a multimission laboratory managed and operated
by National Technology and Engineering Solutions of Sandia, LLC., a wholly
owned subsidiary of Honeywell International, Inc., for the U.S. Department of
Energy's National Nuclear Security Administration under contract DE-NA0003525.

Copyright (c) 2009-2022, NTESS

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

#ifndef SPROCKIT_COMMON_UNITS_H_INCLUDED
#define SPROCKIT_COMMON_UNITS_H_INCLUDED

#include <map>
#include <stdint.h>

namespace sprockit {

/// Multiply two 64 bit integer values and check for overflow.
/// This would be two lines if we had 128-bit integers.
int64_t multiply64(int64_t a, int64_t b, bool &errorflag);

void populateBandwidthNames(std::map<std::string, int64_t>& abbrname,
                            std::map<std::string, int64_t>& fullname);

double getBandwidth(const char *value, bool &errorflag, bool print_errors = false);

double getBandwidth(const char *value);

double getFrequency(const char* value, bool& errorflag, bool print_errors = false);

long byteLength(const char* value, bool& errorflag, bool print_errors = false);

void populateTimeDeltaNames(std::map<std::string, int64_t> &value);

void populateFrequencyNames(std::map<std::string, int64_t> &value);

/// Get a timestamp possiblly suffixed with any of the identifiers
/// psec, nsec, usec, msec, sec, ps, ns, us, ms, s
double getTimeDelta(const char *value, bool &errorflag, bool print_errors = false);

}

#endif
