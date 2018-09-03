from sst.macro import *
import sys
import os
import sst

ic = setupDeprecated()
print dir(ic.switches[0])
print ic.num_switches

trafficStats = sst.StatisticGroup("traffic_intensity_stats")
trafficStats.addStatistic("traffic_intensity", {"resetOnRead": False})
#sst.StatisticOutput("sst.statOutputEXODUS", {"filepath" : "./trafExodus.out"})
trafficStats.setOutput(sst.StatisticOutput("macro.statOutputEXODUS", {"filepath": "/Users/perrinel/Dev/trafEXODUS.out", "count_x":"4", "count_y":"4"}))

for i in range(ic.num_switches):
  s, params = ic.switches[i]
  trafficStats.addComponent(s)
  s.enableStatistics(["traffic_intensity"], {"type":"macro.traffic_intensity","rate":"0ns"})

#for n in self.nodes:
#  s.enableStatistics([
#    "traffic_intensity"], {
#    "type":"sst.TrafficStatistic",
#      "rate":"0ns"
#      })

