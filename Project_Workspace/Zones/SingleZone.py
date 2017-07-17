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

# next:
# zone_contains method
# all unique pairs in a table,
# ie: (n1, n2) => cost
# The dict is (tuple) = [edge]

class Zone(object):
  def __init__(self, center, network):
    self.network = network
    self.center = center
    self.memberNodes, self.borderNodes = self.buildZone(center)
    self.optimalRoutes = self.setupRoutePairs(self.memberNodes)

  def buildZone(self, center):
    i = 0
    centerNode = self.network.getNode(center)
    nodes = set()
    borderNodes = set()
    nodes.add(centerNode)
    self.collectNodes(nodes, borderNodes, centerNode, i)
    return [node.getID() for node in nodes], [node.getID() for node in borderNodes ]

  # for now, zones are effectively circular
  def collectNodes(self, collection, borderNodes, centerNode, stackdepth):
    if stackdepth < NEIGHBOUR_LIM:
      neighbours = [edge.getToNode() for edge in centerNode.getOutgoing()]
      for n in neighbours:
        self.collectNodes(collection, borderNodes, n, stackdepth+1)

    collection.add(centerNode)

    if stackdepth == NEIGHBOUR_LIM:
      borderNodes.add(centerNode)

  def setupRoutePairs(self, nodes):
    # route pairs init with dijkstra
    routes = {}
    for index, node1 in enumerate(nodes):
      for node2 in nodes[:index]+nodes[index+1:]:
        routes[(node1, node2)] = dijkstra(node1, node2, self.network)

    return routes

  def updateRoutePairs(self):
    # Change table on traffic change
    pass

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
  return len( node.getOutgoing() > 1)

def assignNodesToZones(network, nodes):
  # Zone-network coverage here
  pass

""" Setup and benchmarking """
def main():
  complianceFactor, config, gui, csvFile = parseArgs()
  configPaths = SumoConfigWrapper(config)

  sumoGui = ["/usr/local/bin/sumo-gui", "-c", config]
  sumoCmd = ["/usr/local/bin/sumo", "-c", config]

  # Pick a node
  network = sumolib.net.readNet(configPaths.networkfile)
  nodes = [ i.getID() for i in network.getNodes() ]

  # Best way to generate zones for whole network:
  # Keep a structure of zones
  # until every node is in >0 zones, continue. If a node is already in two nodes, skip
  # this will probably result in tight overlap
  zoneStore = assignNodesToZones(network, nodes)
  zoneCenter = random.choice(nodes)
  myZone = Zone(zoneCenter, network)
  print zoneCenter

if __name__ == "__main__":
  starttime = time.time()
  main()
  print "Execution completed in %f seconds." % (time.time() - starttime)