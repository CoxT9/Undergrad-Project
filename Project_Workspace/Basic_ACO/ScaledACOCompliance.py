"""
A WIP script to demo the performance of a very basic repellent-ACO system scaled to multiple vehicles.
The script tracks the following metrics, doing so with and without ACO in effect:
- Moments to destination for all vehicles
- Moments to destination for compliant vehicles

Arguments:
  - Percentage, an integer 1 < n < 100 identifying the percentage of agents to comply with ACO
  - Config File, the path to a .sumocfg file which containts necessary information about the simulation
  - A 0 or 1, indicating whether the simulation will run with the GUI (1 for GUI)

  Next steps for the threshold-reroute model:
  - Investigate pheromone deposit and evaporation mechanisms
  - Tweak criteria for dijkstra based on what SUMO/TraCI provides (travltime? effort?)
  - Consider how to incorporate velocity into density model
  - Tweak threshold for acceptable congestion rates

  Next steps for project overall:
  - Tackle odd junction deadlock issue

  Next model:
  - Look at Multipath routing. Backup paths? User-choice?
  - Complete readings on this matter and look into implementation for week of June 12
"""
##################################################################################################################################
####################################################### CLASSES ##################################################################
##################################################################################################################################
class Trip(object):
  def __init__(self, agent, src, dest):
    self.agent = agent
    self.src = src
    self.dest = dest

class Agent(object):
  def __init__(self, agentid, visitedEdges=None, lastEdge=None, destinationEdge=None, destination=None, source=None):
    self.agentid = agentid
    self.visitedEdges = visitedEdges if visitedEdges else []
    self.lastEdge = lastEdge if lastEdge else ""
    self.destinationEdge = destinationEdge if destinationEdge else ""
    self.destination = destination if destination else ""
    self.source = source if source else ""


class ACOMetrics(object):
  def __init__(self, averageCompliantAgentTime, overallAverageTime):
    self.averageCompliantAgentTime = averageCompliantAgentTime
    self.overallAverageTime = overallAverageTime

  def dumpMetrics(self):
    print "Overall average time: ", self.overallAverageTime
    print "Average time of compliant agents: ", self.averageCompliantAgentTime

class SumoConfigWrapper(object):
  def __init__(self, configfile):
    self.configfile = configfile
    self.networkfile, self.routefile, self.logfile, self.endtime = self.parsecfg(configfile)

  def parsecfg(self, config):
    network = sumogenerator(config, entity="net-file").next().value
    routes = sumogenerator(config, entity="route-files").next().value
    logfile = sumogenerator(config, entity="vehroute-output").next().value
    endtime = sumogenerator(config, entity="end").next().value
    return network, routes, logfile, endtime

##################################################################################################################################
###################################################### SETUP #####################################################################
##################################################################################################################################
from fractions import Fraction
from random import randint
import copy
import math
import os
import subprocess
import sys
import time
import xml.etree.ElementTree

sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

import sumolib
import traci

BIG_NUM = 1000
PH_THRESHOLD = 100
EVAP_RATE = .08
DEP_RATE = 0.5
RED = (255, 0, 0, 0)
##################################################################################################################################
####################################################### FUNCTIONS ################################################################
##################################################################################################################################
""" Route @agent from @currEdge to the lowest-@ph unvisited edge in 
@network which contains a path to @destEdge not covering @visitedEdges """
def reroute(agent, currEdge, network, ph):
  visitedEdges = agent.visitedEdges
  destEdge = agent.destinationEdge

  outgoing = [i.getID() for i in network.getEdge(currEdge).getOutgoing()]
  unvisitedCandidates = [item for item in outgoing if item not in visitedEdges and flipsign(item) not in visitedEdges]

  validCandidates = []
  i = 0
  while destEdge not in validCandidates and flipsign(destEdge) not in validCandidates and i < len(unvisitedCandidates):
    if pathExists(unvisitedCandidates[i], destEdge, visitedEdges+[flipsign(unvisitedCandidates[i])], network):
      validCandidates.append(unvisitedCandidates[i])
    i += 1

  unsignedCandidates = [unsigned(item) for item in validCandidates]
  if unsigned(destEdge) in unsignedCandidates:
    target = validCandidates[unsignedCandidates.index(unsigned(destEdge))]
  else:
    candidateKeypairs = {key: ph[key] for key in ph if key in validCandidates}
    target = min(candidateKeypairs, key=candidateKeypairs.get)

  visitedEdges.append(target)
  visitedEdges.append(flipsign(target))
  traci.vehicle.setRoute(agent.agentid, [currEdge, target])

""" Some old fashioned Dijkstra. Find the shortest path from @srcEdge to @destEdge on @network using @weights """
def shortestPath(src, dest, network, weights):
  if src == dest:
    return [dest]

  unvisited = {src}
  visited = set()

  totalCost = {src: 0}
  candidates = {}

  while unvisited:
    # Pick smallest weight
    current = min( [ (totalCost[node], node) for node in unvisited] )[1]
    if current == dest:
      break

    unvisited.discard(current)
    visited.add(current)

    connectedNodesToEdges = {}
    for edge in network.getNode(current).getOutgoing():
      connectedNodesToEdges[network.getEdge(edge.getID()).getToNode().getID()] = edge.getID()

    unvisitedNeighbours = set(connectedNodesToEdges.keys()).difference(visited)
    for neighbour in unvisitedNeighbours:
      nei_dist = totalCost[current] + weights[connectedNodesToEdges[neighbour]]
      if nei_dist < totalCost.get(neighbour, float('inf')):
        totalCost[neighbour] = nei_dist
        candidates[neighbour] = current
        unvisited.add(neighbour)

  return unpackPath(candidates, dest)

""" Dijkstra utility function """
def unpackPath(candidates, dest):
  if dest not in candidates:
    return None
  goal = dest
  path = []
  while goal:
    path.append(goal)
    goal = candidates.get(goal)
  return list(reversed(path))

""" Convert @junctionList to the corresponding edgeList. This function is needed because traci's reroute takes a list of edges,
but dijkstra returns a list of vertices. """
def edgeListConvert(vertexList, network):
  incomingEdges = []
  outgoingEdges = []
  edgeList = []

  i = 0
  while i < len(vertexList)-1: # get outgoing up to last one
    outgoingEdges.append([j.getID() for j in network.getNode(vertexList[i]).getOutgoing()])
    i += 1

  i = 1
  while i < len(vertexList): # get incoming starting from first
    incomingEdges.append([j.getID() for j in network.getNode(vertexList[i]).getIncoming()])
    i += 1

  assert len(incomingEdges) == len(outgoingEdges)
  i = 0
  while i < len(incomingEdges):
    currIncoming = incomingEdges[i]
    currOutgoing = outgoingEdges[i]
    edgeList.append(str(list( set(currIncoming).intersection(currOutgoing) )[0]) )
    i += 1

  return edgeList

""" Get all non-internal edges in the system """
def getExplicitEdges():
  return [ edge for edge in traci.edge.getIDList() if edge[0] != ":"]

""" Check if any edges in @edgeList exceed a threshold density """
def congestionExcessive(edgeList, ph, network):
  excessive = False
  i = 0
  while not excessive and i < len(edgeList):
    currPh = ph[edgeList[i]]
   # print currPh
    if currPh > PH_THRESHOLD:
      excessive = True
    i += 1

  return excessive

""" Check if a path exists in @network between @srcEdge and @destEdge that does not include @visitedEdges """
def pathExists(srcEdge, destEdge, visitedEdges, network):
  if unsigned(srcEdge) == unsigned(destEdge):
    return True

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
      print vehicle.id, "Failed to reach destination"
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

def sumolist(filename, entity='vehicle'):
  return list(sumogenerator(filename, entity))

def sumogenerator(filename, entity='vehicle'):
  return sumolib.output.parse(filename, entity)

""" Reduce the simulation's routes to trips (src, dest pairs) and return the trips """
def getTrips(routes):
  trips = []
  for routeEntry in sumogenerator(routes):
    edges = routeEntry.route[0].edges.split()
    trips.append(Trip(routeEntry.id, edges[0], edges[-1]))
  return trips

def parseArgs():
  if len(sys.argv) != 4:
    print "Usage: %s <ComplianceFactor> <ConfigFile> <0/1> " % sys.argv[0]
    exit(0)

  complianceFactor = float(sys.argv[1])
  config = sys.argv[2]
  gui = bool(int(sys.argv[3]))
  return complianceFactor, config, gui

""" Return a subset of all agents for an ACO sample population """
def getParticipatingAgents(complianceFactor, routefile):
  agents = sumolist(routefile)
  complianceTotal = int(len(agents) * (complianceFactor / 100) )
  print "Total number of compliant agents:", complianceTotal
  print "Total agents in the dataset:", len(agents)

  compliantAgents = {}
  while len(compliantAgents) < complianceTotal:
    index = randint(0, len(agents)-1)
    compliantAgents[agents[index].id] = Agent(agents[index].id)

  print "IDs of participating agents:", compliantAgents.keys()
  return compliantAgents

""" Launch the simulation in the simplest way as a proof-of-concept """
def executeSimple(compliantAgents, config, startCommand):
  network = sumolib.net.readNet(config.networkfile)
  traci.start(startCommand)
  ph = {value:0 for value in traci.edge.getIDList()}

  for trip in getTrips(config.routefile):
    if trip.agent in compliantAgents:
      compliantAgents[trip.agent].destination = unsigned(trip.dest)

  print "Launching ACO simulation..."
  for _ in xrange(int(config.endtime)):
    presentAgents = traci.vehicle.getIDList()
    for v in presentAgents:
      ph[traci.vehicle.getRoadID(v)] += 1

    for compliantV in set(presentAgents).intersection(compliantAgents):
      traci.vehicle.setColor(compliantV, RED)

      currentEdge = traci.vehicle.getRoadID(compliantV)
      if compliantAgents[compliantV].lastEdge != currentEdge and unsigned(currentEdge) != compliantAgents[compliantV].destination:
        reroute(
          compliantAgents[compliantV], 
          currentEdge, 
          network, 
          ph
        )
        compliantAgents[compliantV].lastEdge = currentEdge

    for e in ph:
      ph[e] = max(ph[e]-1, 0)

    traci.simulationStep()

  traci.close(False)
  time.sleep(1)
  print "\nDone."
  return ACOMetrics(*averages(sumolist(config.logfile), compliantAgents))

""" impl the traffic delta formula from DTPOS """
def trafficDelta(edge, network):
  return sum([ traci.vehicle.getSpeed(v)/(network.getEdge(edge).getLength()/1000) for v in traci.edge.getLastStepVehicleIDs(edge)])

""" Launch the simulation with the following optimizations:
 - Actually use edges and vertices instead of this weird unsigned stuff
 - Pheromone deposit and evaporation optimized
 - Reroute triggers a full reroute to destination, not a single edge
 - Reroute finds the shortest path considering a balance of distance and density
 - Reroute only triggers if density of route exceeds a threshold
"""
def executeOptimized(compliantAgents, config, startCommand):
  network = sumolib.net.readNet(config.networkfile)
  traci.start(startCommand)
  costStore = {value:0 for value in getExplicitEdges()}
  traffic = {value:1/(network.getEdge(value).getLength()/1000) for value in getExplicitEdges()}

  for trip in getTrips(config.routefile):
    if trip.agent in compliantAgents:
      compliantAgents[trip.agent].source = network.getEdge(trip.src).getFromNode().getID()
      compliantAgents[trip.agent].destination = network.getEdge(trip.dest).getToNode().getID()

  print "Launching Optimized ACO simulation..."
  for _ in xrange(int(config.endtime)):
    presentAgents = traci.vehicle.getIDList()

    # One big goal right now is to identify the model for which the cost of edges will be defined.
    # IACO and DTPOS have provided some insight here, they are compared to the SUMO-native shortest-time, and by simple evaluating the number of vehicles alone
    # DTPOS appears to outperform IACO, though DTPOS is only being used here for its representation of traffic on edges.
    # Need to find the right way to model the pseudodynamic nature of traffic networks for use in dijkstra
    for e in [edge for edge in costStore if edge[0] != ":"]: # Skip internal edges
      # IACO model
      #costStore[e] = network.getEdge(e).getLength() + ( traci.edge.getLastStepVehicleNumber(e) * DEP_RATE) - ( (network.getEdge(e).getLength()/network.getEdge(e).getSpeed()) * DEP_RATE )
      # DTPOS model
      # the cost is distance*traffic
      # Setup cost then update traffic for next time interval
      # This uses the cost evaluation for edges from DTPOS, but does not use the entire model (ie: use dijkstra instead of scoring summation)
      costStore[e] = traci.edge.getLastStepVehicleNumber(e) * (0.5 * (network.getEdge(e).getLength()/1000)) * (0.5 * traffic[e])
      traffic[e] = ((1 - EVAP_RATE) * traffic[e]) + trafficDelta(e, network)
      # Pure-density model
      #costStore[e] = traci.edge.getLastStepVehicleNumber(e)
    #print costStore["-313"]

    for compliantV in set(presentAgents).intersection(compliantAgents):
      traci.vehicle.setColor(compliantV, RED)
      if congestionExcessive(traci.vehicle.getRoute(compliantV), costStore, network) and traci.vehicle.getRoadID(compliantV)[0] != ":":
        edgeList = edgeListConvert( 
          shortestPath(
            network.getEdge(traci.vehicle.getRoadID(compliantV)).getToNode().getID(), 
            compliantAgents[compliantV].destination, 
            network, 
            costStore), 
          network)
        traci.vehicle.setRoute(compliantV, [traci.vehicle.getRoadID(compliantV)]+edgeList)

    traci.simulationStep()
    
  traci.close(False)
  time.sleep(1)
  print "\nDone."
  return ACOMetrics(*averages(sumolist(config.logfile), compliantAgents))

""" Setup and benchmarking """
def main():
  complianceFactor, config, gui = parseArgs()

  configPaths = SumoConfigWrapper(config)
  compliantAgents = getParticipatingAgents(complianceFactor, configPaths.routefile)

  sumoGui = ["/usr/bin/sumo-gui", "-c", config]
  sumoCmd = ["/usr/bin/sumo", "-c", config]

  subprocess.call(sumoCmd, stderr = open(os.devnull, 'w'))
  print "\n"
  benchmarkResults = ACOMetrics(*averages(sumolist(configPaths.logfile), compliantAgents))
  benchmarkResults.dumpMetrics()

  #simulationResults = executeSimple(copy.deepcopy(compliantAgents), configPaths, sumoGui if gui else sumoCmd)
  #print "Dumping metrics changes from simple ACO..."
  #simulationResults.dumpMetrics()

  simulationResults = executeOptimized(copy.deepcopy(compliantAgents), configPaths, sumoGui if gui else sumoCmd)
  print "Dumping metrics changes from optimized (ph) ACO..."
  simulationResults.dumpMetrics()

##################################################################################################################################
####################################################### LAUNCH ###################################################################
##################################################################################################################################

if __name__ == "__main__":
  starttime = time.time()
  main()
  print "Execution completed in %f seconds." % (time.time() - starttime)
else:
  print __name__
  exit(0)