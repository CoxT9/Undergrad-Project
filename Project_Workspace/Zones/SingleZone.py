# Working with partitioning a SUMO network into zones.

# First, assume a zone has a radius of two. pick a node and use it to form a zone.
# Some code repition from the first ACO-SUMO project but that's alright
""" Next:
- Criteria for zone center
- Color vehicles inside a zone
- Setup intra zone routing tables
- Path traversal problem
"""

import os
import random
import sys
import time

sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

import sumolib
import traci

NEIGHBOUR_LIM = 2
MEMBERSHIP_LIM = 5 # Consider minimum zone coverage

# TraCI colors

# next:
# zone_contains method
# all unique pairs in a table,
# ie: (n1, n2) => cost
# The dict is (tuple) = [edge]

class Zone(object):
  def __init__(self, center, network, memberNodes=None, borderNodes=None):
    self.color = (
      random.sample( xrange(0, 255), 1)[0], 
      random.sample( xrange(0, 255), 1)[0], 
      random.sample( xrange(0, 255), 1)[0], 
    0)
    self.network = network
    self.center = center
    if memberNodes == None or borderNodes == None:
      self.memberNodes, self.borderNodes = self.buildZone(center)
    else:
      self.memberNodes = list(memberNodes)
      self.borderNodes = list(borderNodes)

    self.optimalRoutes = self.setupRoutePairs(self.memberNodes)

  def buildZone(self, center):
    i = 0
    centerNode = self.network.getNode(center)
    nodes = set()
    borderNodes = set()
    nodes.add(centerNode)
    collectNodes(nodes, borderNodes, centerNode, i)
    return list(nodes), list(borderNodes)

  def updateRoutePairs(self, weights):
    # Update pairs table on traffic change
    newOptimalRoutes = {}

    for src, dest in self.optimalRoutes:
      newOptimalRoutes[(src, dest)] = dijkstra(src, dest, self.network, weights)

    self.optimalRoutes = newOptimalRoutes

  def setupRoutePairs(self, nodes):
    # route pairs init with dijkstra
    routes = {}
    for index, node1 in enumerate(nodes):
      for node2 in nodes[:index]+nodes[index+1:]:
        routes[(node1, node2)] = dijkstra(node1, node2, self.network)

    return routes

  def __contains__(self, nodeId):
    return nodeId in self.memberNodes

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

def sumolist(filename, entity='vehicle'):
  return list(sumogenerator(filename, entity))

def sumogenerator(filename, entity='vehicle'):
  return sumolib.output.parse(filename, entity)

def parseArgs():
  if len(sys.argv) != 5:
    print "Usage: %s <ComplianceFactor> <ConfigFile> <0/1> <CsvFile>" % sys.argv[0]
    exit(0)

  complianceFactor = float(sys.argv[1])
  config = sys.argv[2]
  gui = bool(int(sys.argv[3]))
  csvFile = sys.argv[4]
  return complianceFactor, config, gui, csvFile

# for now, zones are effectively circular
""" Given @centerNode, return a list of nodes in the zone, and nodes of the zone's border """
def collectNodes(collection, borderNodes, centerNode, stackdepth=0):
  if stackdepth < NEIGHBOUR_LIM:
    neighbours = [edge.getToNode() for edge in centerNode.getOutgoing()]
    for n in neighbours:
      collectNodes(collection, borderNodes, n, stackdepth+1)

  collection.add(centerNode.getID())

  if stackdepth == NEIGHBOUR_LIM:
    borderNodes.add(centerNode.getID())

""" Some old fashioned Dijkstra. Find the shortest path from @srcEdge to @destEdge on @network using @weights """
def dijkstra(src, dest, network, weights=None):
  if weights == None:
    weights = {}
    for e in network.getEdges():
      weights[e.getID()] = e.getLength()

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

def isValidZoneCenter(node):
  return len(node.getOutgoing()) > 1

# What is the fastest/best way to ensure all nodes belong to at least 1 zone?
# vertex cover?
def assignNodesToZones(network, nodes):
  # Zone-network coverage here
  zoneStore = []
  nodeToZones = {value:[] for value in nodes}
  nodeZoneMembership = {value:0 for value in nodes} # id to 0

  done = False

  while not done:
    # look for a node not yet in at least 1 zone
    # the issue here is that zones also can't be in more than 2 zones.
    print nodeZoneMembership.values()
   # time.sleep(3)
    # Take a node. Is it a valid center? Is it in under 2 zones? 
    # If yes, take the node's neighbor candidates. Are they all in under 2 ones?
    # If yes, make the zone and update the node-zone appearance table
    # else, next node
    for node in nodeZoneMembership.keys():
      if isValidZoneCenter(network.getNode(node)) and nodeZoneMembership[node] < MEMBERSHIP_LIM:
        candidateNodes = set()
        candidateBorders = set()
        collectNodes(candidateNodes, candidateBorders, network.getNode(node))

        if MEMBERSHIP_LIM not in [nodeZoneMembership[cand] for cand in candidateNodes]: # this zone will not break the limit
          newZone = Zone(node, network, candidateNodes, candidateBorders)
          zoneStore.append(newZone)
          for member in newZone.memberNodes:
            nodeZoneMembership[member] += 1
            # Each node belongs to a list of zones
            nodeToZones[member].append(newZone)


    done = 0 not in nodeZoneMembership.values()

  return zoneStore, nodeToZones

def getExplicitEdges():
  return [ edge for edge in traci.edge.getIDList() if edge[0] != ":"]

def launchSim(zoneStore, nodeToZoneDict, network, sim, config):
  traci.start(sim)
  edges = getExplicitEdges()
  costStore = {value:0 for value in edges}
  traffic = {value:0 for value in edges}

  for _ in xrange(int(config.endtime)):
    # Color the vehicles on each edge based on their source node
    for edge in edges:
      edgeOccupants = traci.edge.getLastStepVehicleIDs(edge)
      fromNode = network.getEdge(edge).getFromNode().getID()
      # get Zone of fromNode, and use that color
      currZone = nodeToZoneDict[fromNode][0] # The first zone this node belongs to
      color = currZone.color
      for agent in edgeOccupants:
        traci.vehicle.setColor(agent, color)

      # Update edge ph at each step
      edgelen = network.getEdge(edge).getLength()
      costStore[edge] = edgelen + traffic[edge]
      traffic[edge] += 0.2 * (sum([1/(traci.vehicle.getSpeed(v)+0.01) for v in edgeOccupants]))

      traffic[edge] = max(0, traffic[edge]/(0.2 * (edgelen/network.getEdge(edge).getSpeed())) )

    # With up-to-date traffic levels, update zone proactive route tables
    for z in zoneStore:
      z.updateRoutePairs(costStore)

    traci.simulationStep()
    if len(traci.vehicle.getIDList()) < 1:
      break

  traci.close(False)
  print "Completed Execution."

""" Setup and benchmarking """
def main():
  complianceFactor, config, gui, csvFile = parseArgs()
  configPaths = SumoConfigWrapper(config)

  sumoGui = ["/usr/local/bin/sumo-gui", "-c", config]
  sumoCmd = ["/usr/local/bin/sumo", "-c", config]

  # Pick a node
  network = sumolib.net.readNet(configPaths.networkfile)
  nodes = [node.getID() for node in network.getNodes()]

  zoneStore, nodeToZones = assignNodesToZones(network, nodes) # Collection of zones which cover the graph
  exc = sumoGui if gui else sumoCmd
  launchSim(zoneStore, nodeToZones, network, exc, configPaths)
  # First, run TraCI and color vehicles differently based on the zone
  print len(zoneStore)

if __name__ == "__main__":
  starttime = time.time()
  main()
  print "Execution completed in %f seconds." % (time.time() - starttime)