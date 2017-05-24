# Basic simulation of a single vehicle adhering to simple ACO
# In normal simulation, target vehicle ("0") took 495 moments to reach dest
# This program attempts to use ACO to improve on this metric

# A test run of this script brings the vehicle of interest to its destination in 435 moments - an improvement of 60 moments
# If anything, this is a successful one-small-step

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
  maxPh = 0
  minPh = 1000000

  outgoingIds = [ i.getID() for i in outgoing ]
  unvisitedCandidates = [item for item in outgoingIds if item not in vis]
  validCandidates = [item for item in unvisitedCandidates if pathExists(item, dest, vis, net) or item == dest ]
  for cand in validCandidates:
    if ph[cand] < minPh:
      target = cand
      minPh = ph[cand]

  # Set vehicle edge to lowest ph unvisited edge that connects to dest 
  print "edge, targ", edge, target
  vis.append(target)
  traci.vehicle.setRoute(veh, [edge, target])

def pathExists(src, dst, vis, net):
  # bfs for src to dst (no loop back)
  q = [(src, [src])]
  while q:
    (curr, path) = q.pop(0)
    outgoing = [ i.getID() for i in net.getEdge(curr).getOutgoing() ]
    for out in [item for item in outgoing if item not in path and item not in vis ]:
      if out == dst:
        return True
      else:
        q.append((out, path + [out]))

  return False
  
traci.start(sumoCmd)

ph = {}
for e in traci.edge.getIDList():
  ph[e] = 0

print "Executing single-vehicle adherence simulation..."

traveltime = 0
interestV = "0"
visitedEdges = []
lastEdge = ""

destinationEdge = traci.vehicle.getRoute(interestV)[-1]# This should be translated into the connecting vertex
traci.simulationStep()
for i in range(1000): # run sim
  for v in traci.vehicle.getIDList():
    ph[traci.vehicle.getRoadID(v)] += 1
  
  if interestV in traci.vehicle.getIDList():
    traveltime += 1 
    current = traci.vehicle.getRoadID(interestV)
    print current, destinationEdge
    if lastEdge != current and current != destinationEdge:
      reroute(interestV, ph, visitedEdges, destinationEdge, net, current)
      lastEdge = current
 
  for e in ph:
    ph[e] = max(ph[e]-1, 0)

  traci.simulationStep()

print "Closing..."
print "Interest vehicle reached dest in time: ", traveltime
traci.close(False)


