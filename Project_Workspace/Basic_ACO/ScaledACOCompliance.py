# This is a python script which executes the simple repellent-ACO system against an input road network.
# This script attempts to improve the following metrics:
# - Average overall travel time
# - Average travel time of participating/compliant vehicles.
# Travel time means "moments to desination"

# When the compliance to ACO is scaled, it must be noted that the entrances of vehicles is staggered (for realism and performance purposes)
# This program takes one argument: The percentage of vehicles which will comply to the simple repellent-ACO system

# Note that there are 385 vehicles in the current simple dataset

import os, sys, time, xml.etree.ElementTree, subprocess
from random import randint
from fractions import Fraction

sys.path.append(os.path.join(os.environ["SUMO_HOME"], 'tools'))
sumoGui = ["/usr/bin/sumo-gui", "-c", "./ACOSim.sumocfg"]
sumoCmd = ["/usr/bin/sumo", "-c", "./ACOSim.sumocfg"]

import traci, traci.constants, sumolib

def reroute(agent, currEdge, visitedEdges, destEdge, network, ph):
  outgoing = [i.getID() for i in network.getEdge(currEdge).getOutgoing()]
  
  unvisitedCandidates = [item for item in outgoing if item not in visitedEdges]
  validCandidates = [item for item in unvisitedCandidates if pathExists(item, destEdge, visitedEdges, network) or item == destEdge]
  minPh = 10000000000
  candidateKeypairs = {key: ph[key] for key in ph if key in validCandidates}
  if len(candidateKeypairs.keys()) < 1:
    print agent, currEdge, destEdge, visitedEdges
  target = min(candidateKeypairs, key=candidateKeypairs.get)

  visitedEdges.append(target)
  visitedEdges.append(str(int(target)*-1))
  traci.vehicle.setRoute(agent, [currEdge, target])

def pathExists(srcEdge, destEdge, visitedEdges, network):
  q = [(srcEdge, [srcEdge])]
  while q:
    (curr, path) = q.pop(0)
    print srcEdge, destEdge, visitedEdges
    outgoing = [i.getID() for i in network.getEdge(curr).getOutgoing()]
    for out in [item for item in outgoing if item not in path and item not in visitedEdges]:
      if out == destEdge:
        return True
      else:
        q.append((out, path + [out]))

  return False

network = sumolib.net.readNet("HighConnectivity.net.xml")

# Collect all compliant vehicle ids:
e = xml.etree.ElementTree.parse('LongRoutes.rou.xml').getroot()
i = 1
compliantAgents = set()

agents = e.findall('vehicle')
complianceTotal = int(len(agents) * (float(sys.argv[1]) / 100) )
print "Total number of compliant agents:", complianceTotal

while len(compliantAgents) < complianceTotal:
  index = randint(0, len(agents)-1)
  compliantAgents.add(agents[index].get('id'))

# We now have a collection of x % of all agents which will be compliant to the ACO system.
# Run the simulation twice:
# First run: Gather arrival data on "compliant" members and whole dataset
# Second run: Actually execute ACO on compliant members and gather same metrics
subprocess.call(sumoCmd, stderr = open(os.devnull, 'w'))
time.sleep(1)
print ""
print "Completed execution of non-ACO simulation"

overallAverageTime = 0
averageCompliantAgentTime = 0
for vehicle in sumolib.output.parse('Logs.out.xml', 'vehicle'):
  travelTime = float(vehicle.arrival) - float(vehicle.depart)
  if vehicle.id in compliantAgents:
    averageCompliantAgentTime += travelTime
  
  overallAverageTime += travelTime

overallAverageTime /= len(agents)
averageCompliantAgentTime /= len(compliantAgents)

print "Average time for all vehicles without any ACO compliance is", overallAverageTime
print "Average time for to-be-compliant subset of vehicles (without compliance) is", averageCompliantAgentTime

# So far this script gathers a percentage of the agent population as "ACO-compliant" agents. 
# The script then evaluates the average travel-time performance of the whole population and the compliant population,
# without any usage of ACO so far.

# Next, the simulation will execute with the target population complying to ACO.
#traci.start(sumoGui)

print format("Launching multi-vehicle-compliance scenario with %s%% population compliance..." % sys.argv[1])

traci.start(sumoGui)
ph = { value:0 for value in traci.edge.getIDList()}
visitedEdgesStore = {agent:[] for agent in compliantAgents}
lastEdgeStore = {agent:"" for agent in compliantAgents}
destinationEdgeStore = {}

for trip in sumolib.output.parse('ActualTripData.trip.xml', 'trip'):
  if trip.id in compliantAgents:
    destinationEdgeStore[trip.id] = trip.to
    

# Will need keystores for all previously scalar data

for _ in xrange(1000):
  presentAgents = traci.vehicle.getIDList()
  for v in presentAgents:
    ph[traci.vehicle.getRoadID(v)] += 1
  for compliantV in set(presentAgents).intersection(compliantAgents):
    #print compliantV
    traci.vehicle.setColor(compliantV, (255, 0, 0, 0))
    # Run ACO system for compliant vehicle
    currentEdge = traci.vehicle.getRoadID(compliantV)
    
    if lastEdgeStore[compliantV] != currentEdge and currentEdge != destinationEdgeStore[compliantV]:
      reroute(compliantV, currentEdge, visitedEdgesStore[compliantV], destinationEdgeStore[compliantV], network, ph)
      
      lastEdgeStore[compliantV] = currentEdge
    
  for e in ph:
    ph[e] = max(ph[e]-1, 0)

  traci.simulationStep()
  #print _, "is value"

traci.close(False)
print "Done." 
