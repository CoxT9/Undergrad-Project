"""
A WIP script to demo the performance of a repellent-ACO system scaled to multiple vehicles.
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
  def __init__(self, overallAverageTime, averageCompliantAgentTime):
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
from multiprocessing import Process
from random import randint
from threading import Thread
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
PH_THRESHOLD = 250
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

def shortestPath(src, dest, edgesStore, network, weights):
  return dijkstra(src, dest, network, weights)
  #return bfm(src, dest, edgesStore, network, weights)

""" Some old fashioned Dijkstra. Find the shortest path from @srcEdge to @destEdge on @network using @weights """
def dijkstra(src, dest, network, weights):
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

  result = unpackPath(candidates, dest)
  return result

""" get a sumo network node neighbours """
def getNeighbors(network, node):
  return [ network.getEdge(e.getID()).getToNode().getID() for e in network.getNode(node).getOutgoing() ]

""" Setup BFM params """
def initBfm(src, network):
  dests = {}
  prevs = {}

  for node in network.getNodes():
    dests[node.getID()] = float('Inf')
    prevs[node.getID()] = None
  dests[src] = 0
  return dests, prevs

""" Relax utility method for bfm """
def relax(node, neighbour, network, dests, prevs, weights, edgesStore):
  # Find better path between node and neighbour
  edge = edgesStore[(node, neighbour)]
  if dests[neighbour] > dests[node] + weights[edge]:
    dests[neighbour] = dests[node] + weights[edge]
    prevs[neighbour] = node

""" A different pathing approach: Bellman-Ford-Moore. Same params as Dijkstra but different algorithm """
def bfm(src, dest, edgesStore, network, weights):
  dests, prevs = initBfm(src, network)
  nodes = network.getNodes()
  for i in range(len(nodes)-1):
    for node in nodes:
      for neighbour in getNeighbors(network, node.getID()):
        relax(node.getID(), neighbour, network, dests, prevs, weights, edgesStore)

  for node in network.getNodes():
    for neighbour in getNeighbors(network, node.getID()):
      edge = edgesStore[(node.getID(), neighbour)]
      assert dests[neighbour] <= dests[node.getID()] + weights[edge]

  # return actual route
  currNode = dest
  path = []
  while currNode != None:
    path.insert(0, currNode)
    currNode = prevs[currNode]

  return path

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

""" Convert @vertexList to the corresponding edgeList. This function is needed because traci's reroute takes a list of edges,
but dijkstra returns a list of vertices. """
def edgeListConvert(vertexList, edgesStore):
  edgeList = []
  i = 0
  while i < len(vertexList)-1:
    edgeList.append( edgesStore[ (vertexList[i], vertexList[i+1]) ] )
    i += 1

  return edgeList

""" Get all non-internal edges in the system """
def getExplicitEdges():
  return [ edge for edge in traci.edge.getIDList() if edge[0] != ":"]

""" Check if any edges in @edgeList exceed a threshold density """
def congestionExcessive(edgeList, ph):
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
def trafficDelta(edge, network, agents, edgelen):
  return sum([ traci.vehicle.getSpeed(v)/(edgelen) for v in agents])

""" geyt @edge average speed """
def getAverageSpeed(edge):
  currAgents = traci.edge.getLastStepVehicleIDs(edge)
  total = len(currAgents)
  sumSpeed = sum(traci.vehicle.getSpeed(v) for v in currAgents)
  return sumSpeed / total

""" build a table of (node, node) => edge """
def buildEdgesStore(network, edges):
  store = {}
  for e in edges:
    edge = network.getEdge(e)
    toNode = edge.getToNode().getID()
    fromNode = edge.getFromNode().getID()
    store[(fromNode, toNode)] = e

  return store

""" Function for shortest path reroute """
def routeVehicle(vehicleId, costStore, network, edges, edgesStore, compliantAgents, currentRoad, currentRoute, newRoutes):
  if congestionExcessive(currentRoute, costStore) and currentRoad in edges:
    edgeList = edgeListConvert(
      shortestPath(
        network.getEdge(currentRoad).getToNode().getID(),
        compliantAgents[vehicleId].destination,
        edgesStore,
        network,
        costStore),
      edgesStore)
    newRoutes[vehicleId] = [currentRoad]+edgeList

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
  edges = getExplicitEdges()
  costStore = {value:0 for value in edges}
  traffic = {value:1/(network.getEdge(value).getLength()/1000) for value in edges}
  edgesStore = buildEdgesStore(network, edges)
  for trip in getTrips(config.routefile):
    if trip.agent in compliantAgents:
      compliantAgents[trip.agent].source = network.getEdge(trip.src).getFromNode().getID()
      compliantAgents[trip.agent].destination = network.getEdge(trip.dest).getToNode().getID()

  print "Launching Optimized ACO simulation..."
  for _ in xrange(int(config.endtime)):
    presentAgents = traci.vehicle.getIDList()

    # Traffic density model
    for e in edges:
      # IACO model
      # costStore[e] = network.getEdge(e).getLength() + ( traci.edge.getLastStepVehicleNumber(e) * DEP_RATE) - ( (network.getEdge(e).getLength()/network.getEdge(e).getSpeed()) * DEP_RATE )
      # DTPOS model
      agents = traci.edge.getLastStepVehicleIDs(e)
      edgelen = network.getEdge(e).getLength()/1000

      costStore[e] = len(agents) * (0.5 * (edgelen)) * (0.5 * traffic[e])
      traffic[e] = ((1 - EVAP_RATE) * traffic[e]) + trafficDelta(e, network, agents, edgelen)

    # Parallelize
    threads = []
    newRoutes = {}
    for compliantV in set(presentAgents).intersection(compliantAgents):
      traci.vehicle.setColor(compliantV, RED)
      currentRoad = traci.vehicle.getRoadID(compliantV)
      currentRoute = traci.vehicle.getRoute(compliantV)
      threads.append(Thread( # Can interchange between thread or process
        target=routeVehicle, 
        args=(
          compliantV, 
          costStore, 
          network, 
          edges,
          edgesStore, 
          compliantAgents, 
          currentRoad, 
          currentRoute, 
          newRoutes, 
          )
        )
      )
    
    for t in threads:
      t.start()

    for t in threads:
      t.join()

    for key in newRoutes:
      traci.vehicle.setRoute(key, newRoutes[key])

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
