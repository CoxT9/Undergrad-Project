#(['103', '777', '137'###########################################################################################################################################
### A simple script which runs a SUMO simulation where 1 vehicle complies to a very basic repellent-ACO system
### Without R-ACO, the vehicle ("0") completes its route in 494 moments.
### With R-ACO, the vehicle completes its route in approximately 425 moments.
### This is obviously not a finding of any real value or significance - this is a working proof-of-concept for this style of ACO.
###########################################################################################################################################

###########################################################################################################################################
############################################# Setup. ######################################################################################
###########################################################################################################################################
import os, sys, time
from random import randint

sys.path.append(os.path.join(os.environ["SUMO_HOME"], 'tools'))
sumoCmd = ["/usr/bin/sumo-gui", "-c", "./ACOSim.sumocfg"]

import traci, traci.constants, sumolib
traci.start(sumoCmd)
network = sumolib.net.readNet('HighConnectivity.net.xml')

ph = {value:0 for value in traci.edge.getIDList()}

###########################################################################################################################################
############################################# Helper Functions. ###########################################################################
###########################################################################################################################################

def reroute(agent, currEdge, visitedEdges, destEdge, network, ph):
  outgoing = [i.getID() for i in network.getEdge(currEdge).getOutgoing()]

  unvisitedCandidates = [item for item in outgoing if item not in visitedEdges]
  validCandidates = [item for item in unvisitedCandidates if pathExists(item, destEdge, visitedEdges, network) or item == destEdge]
  minPh = 1000000000
  candidateKeypairs = {key: ph[key] for key in ph if key in validCandidates}
  target = min(candidateKeypairs, key=candidateKeypairs.get)

  visitedEdges.append(target)
  targetReverse = str(int(target)*-1) # forbid backtracing

  visitedEdges.append(targetReverse)
  traci.vehicle.setRoute(agent, [currEdge, target])

def pathExists(srcEdge, destEdge, visitedEdges, network):
  q = [(srcEdge, [srcEdge])]
  while q:
    (curr, path) = q.pop(0)
    outgoing = [i.getID() for i in network.getEdge(curr).getOutgoing()]
    for out in [item for item in outgoing if item not in path and item not in visitedEdges]:
      if out == destEdge:
        return True
      else:
        q.append((out, path + [out]))

  return False
  
###########################################################################################################################################
############################################# Actual Simulation Processing. ###############################################################
###########################################################################################################################################

traveltime = 0
interestV = "777"
visitedEdges = []
lastEdge = ""
while interestV not in traci.vehicle.getIDList():
  traci.simulationStep()

destinationEdge = traci.vehicle.getRoute(interestV)[-1]

print "Launching single-vehicle-compliance scenario..."
for _ in xrange(1000):
  traci.simulationStep()
  for v in traci.vehicle.getIDList():
    ph[traci.vehicle.getRoadID(v)] += 1
  
  if interestV in traci.vehicle.getIDList():
    traveltime += 1
    currentEdge = traci.vehicle.getRoadID(interestV)

    if lastEdge != currentEdge and currentEdge != destinationEdge:
      reroute(interestV, currentEdge, visitedEdges, destinationEdge, network, ph)
      lastEdge = currentEdge
 
  for e in ph:
    ph[e] = max(ph[e]-1, 0)

print "Vehicle '0' reached destination at time: ", traveltime
traci.close(False)
print "Done."
