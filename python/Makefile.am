#
#   This file is part of SST/macroscale: 
#                The macroscale architecture simulator from the SST suite.
#   Copyright (c) 2009-2022, NTESS.
#   This software is distributed under the BSD License.
#   Under the terms of Contract DE-NA0003525 with NTESS,
#   the U.S. Government retains certain rights in this software.
#   For more information, see the LICENSE file in the top 
#   SST/macroscale directory.
#

include $(top_srcdir)/Makefile.common

python_includedir=$(includedir)/python
EXTRA_DIST= \
  arielShadowPuppet.py \
  jobScheduler.py \
  merlin.py \
  plotSwitches.py \
  emberMacro.py \
  emberLoadInfo.py \
  emberDefaultParams.py 

nobase_python_include_HEADERS = \
    default.py 

uninstall-local:
	test -f $(includedir)/sstmac/pysst/default.pyc ; if [[ $$? -eq 0 ]] ; then rm $(includedir)/sstmac/pysst/*.pyc; fi




