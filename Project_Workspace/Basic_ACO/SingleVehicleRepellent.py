# Basic simulation of a single vehicle adhering to simple repellent ACO
# In normal simulation, target vehicle ("0") took 317 moments to reach dest
# This program attempts to use ACO to improve on this metric

# Note that this script was developed very quickly

""" Next:
- classes/imports for ACO and SUMO/TRACI boilerplate code
- 25% 50% 100% ACO adherence
- Fix how ph is stored and processed. Persistence layer? Parallel?
- Tweak how ph evaporates and how ph is formulated
"""

import os, sys, time
from random import randint

sys.path.append(os.path.join(os.environ["SUMO_HOME"], 'tools'))

sumoCmd = [ "/usr/bin/sumo-gui", "-c", "./ACOSim.sumocfg" ]

import traci, traci.constants
import sumolib

net = sumolib.net.readNet('HighConnectivity.net.xml')

# ACO Specific functions #

def reroute(veh, ph, vis, dest, net, edge):
  edge = traci.vehicle.getRoadID(veh)
  # use lane.getLinks() for connectivity
  outgoing = net.getEdge(edge).getOutgoing()
 
  target = None
  minPh = 1000000
  for candidate in outgoing:
    if candidate not in vis and ph[candidate] < minPh: # grave assume: all paths lead to dest
      target = candidate
      vis.append(target)

  # Set vehicle edge to lowest ph unvisited edge that connects to dest 
  traci.vehicle.setRoute(edge, target)

traci.start(sumoCmd)

ph = {}
for e in traci.edge.getIDList():
  ph[e] = 0

print "Executing single-vehicle adherence simulation..."

traveltime = 0
interestV = "0"
visitedEdges = []

destinationEdge = traci.vehicle.getRoute(interestV)[-1]# This should be translated into the connecting vertex
traci.simulationStep()
for i in range(400): # run sim
  for v in traci.vehicle.getIDList():
    ph[traci.vehicle.getRoadID(v)] += 1
  
  current = traci.vehicle.getRoadID(interestV)
  print current, destinationEdge
  if current != destinationEdge:
    reroute(interestV, ph, visitedEdges, destinationEdge, net, current)
 
  for e in ph:
    ph[e] -= 1

  traci.simulationStep()
  traveltime += 1

print "Closing..."
print "Interest vehicle reached dest in time: ", traveltime
traci.close(False)


