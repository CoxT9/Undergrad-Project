# A basic approach to pheromones with SUMO: An in-memory key store of edge->pheromone value

import os, sys, time 
from random import randint

if 'SUMO_HOME' in os.environ:
  tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
  sys.path.append(tools)
else:
  sys.exit("Missing SUMO_HOME env var!")

sumoGui = "/usr/bin/sumo-gui"
sumoCmd = [sumoGui, "-c", "./Basic.sumocfg"]

import traci
# Traci constants ref: http://www.sumo.dlr.de/pydoc/traci.constants.html
import traci.constants as tc
traci.start(sumoCmd)

pheromoneStore = {} # Next step: Consider moving this to a persistence layer
for edge in traci.edge.getIDList():
  pheromoneStore[edge] = 0

print "Executing simulation..."



for i in range(200):
  maxPh = ("-1", -1)
  for veh in traci.vehicle.getIDList():
    currEdge = traci.vehicle.getRoadID(veh)
    pheromoneStore[currEdge] += 1
    if(pheromoneStore[currEdge] > maxPh[1]):
      maxPh = (currEdge, pheromoneStore[currEdge])

  for edge in pheromoneStore:
    pheromoneStore[edge] -= 1

  print "The max pheromone at time %d was %s with value of %d" % (i, maxPh[0], maxPh[1])
  traci.simulationStep()
  


print "Closing simulation..."
traci.close(False)

# The next step will be pheromone reading and reactions/rerouting
