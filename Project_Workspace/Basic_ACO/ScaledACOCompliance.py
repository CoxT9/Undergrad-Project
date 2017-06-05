"""
A script to demo the performance of a very basic repellent-ACO system scaled to multiple vehicles.
The script tracks the following metrics, doing so with and without ACO in effect:
- Moments to destination for all vehicles
- Moments to destination for compliant vehicles

Some sample populations which show a performance improvement in compliant agents and total agents (when using ActualTripData.trip.xml)
- 164, 199, 127
- 347, 320, 329, 207, 150

Sample that shows overall increase but compliant-population decrease (in terms of performance):
- 27, 59, 181, 89, 82, 359

Noted behavior of current script and datasets: 
Larger ACO-compliant population tends to grow overall performance marginally, but causes performance decrease for compliant members.

Currently, the script seems to underperform because of the following reasons:
A) Pheromone evaporation optimization missing
B) Pheromone deposit optimization missing
C) Multicriteria optimization between route length and pheromone missing

In terms of python performance: There is a lot of room for parallelism in the independent routing phase.

Arguments:
  - Percentage, an integer 1 < n < 100 identifying the percentage of agents to comply with ACO

"""
##################################################################################################################################
################################################## CONSTANTS & IMPORTS ###########################################################
##################################################################################################################################

CONFIG = "./ACOSim.sumocfg"
NETWORK = "HighConnectivity.net.xml"
ROUTES = "LongRoutes.rou.xml" # "RoutesDataset2.rou.xml"
LOGS = "Logs.out.xml"
TRIPS = "ActualTripData.trip.xml" # "TripDataset2.trip.xml"

BIG_NUM = 10000000000

import os, sys, time, xml.etree.ElementTree, subprocess, math
from random import randint
from fractions import Fraction

sumoGui = ["/usr/bin/sumo-gui", "-c", CONFIG]
sumoCmd = ["/usr/bin/sumo", "-c", CONFIG]

import traci, traci.constants, sumolib

##################################################################################################################################
####################################################### FUNCTIONS ################################################################
##################################################################################################################################

""" Route @agent from @currEdge to the lowest-@ph unvisited edge in 
@network which contains a path to @destEdge not covering @visitedEdges """
def reroute(agent, currEdge, visitedEdges, destEdge, network, ph):
  outgoing = [i.getID() for i in network.getEdge(currEdge).getOutgoing()]
  unvisitedCandidates = [item for item in outgoing if item not in visitedEdges and unsigned(item) not in visitedEdges]
  if unsigned(destEdge) in [ unsigned(item) for item in unvisitedCandidates]:
    validCandidates = [ unvisitedCandidates[ [unsigned(item) for item in unvisitedCandidates ].index( unsigned(destEdge) ) ]]
  else:
    validCandidates = [item for item in unvisitedCandidates if pathExists(item, destEdge, visitedEdges+[flipsign(item)], network)] 
  minPh = BIG_NUM

  candidateKeypairs = {key: ph[key] for key in ph if key in validCandidates}
  target = min(candidateKeypairs, key=candidateKeypairs.get)

  visitedEdges.append(target)
  visitedEdges.append(flipsign(target))
  traci.vehicle.setRoute(agent, [currEdge, target])

""" Check if a path exists in @network between @srcEdge and @destEdge that does not include @visitedEdges """
def pathExists(srcEdge, destEdge, visitedEdges, network):
  q = [(srcEdge, [srcEdge])]
  while q:
    (curr, path) = q.pop(0)
    outgoing = [i.getID() for i in network.getEdge(curr).getOutgoing()]
    for out in [item for item in outgoing if unsigned(item) not in set(path) and unsigned(item) not in visitedEdges]:
      if unsigned(out) == unsigned(destEdge):
        return True
      else:
        q.append((out, path + [ unsigned(out) ]))

  return False

""" Find the average moments to destination for @universalAgents and @compliantAgents """
def averages(universalAgents, compliantAgents):
  totalAgents = len(universalAgents)
  totalCompl = len(compliantAgents)
  overallAverageTime = 0
  averageCompliantAgentTime = 0

  for vehicle in universalAgents:
    arrival = float(BIG_NUM)
    if vehicle.arrival is None:
      print vehicle.id, " Failed to reach destination"
    else:
      arrival = float(vehicle.arrival)
    travelTime = arrival - float(vehicle.depart)
    if vehicle.id in compliantAgents:
      averageCompliantAgentTime += travelTime

    overallAverageTime += travelTime

  return overallAverageTime/totalAgents, averageCompliantAgentTime/totalCompl

def unsigned(identifier):
  return str(abs(int(identifier)))

def flipsign(identifier):
  return str(int(identifier)*-1)

##################################################################################################################################
######################################################## SETUP ###################################################################
##################################################################################################################################
starttime = time.time()
complianceFactor = float(sys.argv[1])

network = sumolib.net.readNet(NETWORK)
agents = xml.etree.ElementTree.parse(ROUTES).getroot().findall('vehicle')

complianceTotal = int(len(agents) * (complianceFactor / 100) )
print "Total number of compliant agents:", complianceTotal
print "Total agents in the dataset:", len(agents)

compliantAgents = set()
while len(compliantAgents) < complianceTotal:
  index = randint(0, len(agents)-1)
  compliantAgents.add(agents[index].get('id'))

subprocess.call(sumoCmd, stderr = open(os.devnull, 'w'))
print "\nCompleted execution of non-ACO simulation"

overallAverageTime, averageCompliantAgentTime = averages(list(sumolib.output.parse(LOGS, 'vehicle')), compliantAgents)
print "IDs of agents complying to ACO: ", ', '.join(compliantAgents)
print "PRE-ACO: Overall average time: ", overallAverageTime
print "PRE-ACO: Average time of compliant agents: ", averageCompliantAgentTime
print "Launching simple ACO simulation..."

##################################################################################################################################
###################################################### EXECUTION #################################################################
##################################################################################################################################

traci.start(sumoGui)

ph = {value:0 for value in traci.edge.getIDList()}
visitedEdgesStore = {agent:[] for agent in compliantAgents}
lastEdgeStore = {agent:"" for agent in compliantAgents}
destinationEdgeStore = {trip.id:unsigned(trip.to) for trip in sumolib.output.parse(TRIPS, 'trip') if trip.id in compliantAgents}

for _ in xrange(1000):
  presentAgents = traci.vehicle.getIDList()
  for v in presentAgents:
    ph[traci.vehicle.getRoadID(v)] += 1

  for compliantV in set(presentAgents).intersection(compliantAgents):
    traci.vehicle.setColor(compliantV, (255, 0, 0, 0))

    currentEdge = traci.vehicle.getRoadID(compliantV)
    if lastEdgeStore[compliantV] != currentEdge and unsigned(currentEdge) != destinationEdgeStore[compliantV]:
      reroute(compliantV, currentEdge, visitedEdgesStore[compliantV], destinationEdgeStore[compliantV], network, ph)
      lastEdgeStore[compliantV] = currentEdge

  for e in ph:
    ph[e] = max(ph[e]-1, 0)

  traci.simulationStep()

traci.close(False)
print "Done."

print "Dumping metric changes from ACO..."
overallAverageTime, averageCompliantAgentTime = averages(list(sumolib.output.parse(LOGS, 'vehicle')), compliantAgents)

print "WITH-ACO: Overall average time: ", overallAverageTime
print "WITH-ACO Average time of compliant agents: ", averageCompliantAgentTime
print format("Execution completed in %f seconds." % (time.time() - starttime))
