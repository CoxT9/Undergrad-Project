# Working with partitioning a SUMO network into zones.

# First, assume a zone has a radius of two. pick a node and use it to form a zone.
# Some code repition from the first ACO-SUMO project but that's alright
""" Next:
- Reduce zone overlap
- Stricter partitioning (zone routes should not leave zone)
- Zone traversal
- Need to parallelize 
"""

import os
import random
import sys
import time

from threading import Thread

sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

import sumolib
import traci

NEIGHBOUR_LIM = 2 # A zone is constructed with r hops from center
MEMBERSHIP_LIM = 5 # A node may only be a member of up to n nodes

class Zone(object):
  def __init__(self, id, center, network, memberNodes=None, borderNodes=None):
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

    self.connectedZones = {} # Which zones _self_ connects to, and which nodes connect _self_ with other zones

    self.optimalRoutes = self.setupRoutePairs(self.memberNodes)
    self.id = "z"+str(id)

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

  def __str__(self):
    return self.id

  def __repr__(self):
    return self.id


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

""" Convert list of vertices to list of edges. This crucial for interopability between graph traversal algorithms and SUMO APIs """
def edgeListConvert(vertexList, edgesStore):
  edgeList = []
  i = 0
  while i < len(vertexList)-1:
    edgeList.append( edgesStore[ (vertexList[i], vertexList[i+1]) ] )
    i += 1

  return edgeList

""" build a table of (node, node) => edge """
def buildEdgesStore(network, edges):
  store = {}
  for e in edges:
    edge = network.getEdge(e)
    toNode = edge.getToNode().getID()
    fromNode = edge.getFromNode().getID()
    store[(fromNode, toNode)] = e

  return store

def isValidZoneCenter(node):
  return len(node.getOutgoing()) > 1

def assignNodesToZones(network, nodes):
  zoneStore = []
  nodeToZones = {value:[] for value in nodes}
  nodeZoneMembership = {value:0 for value in nodes} # id to 0

  done = False
  i = 0

  while not done:
    for node in nodeZoneMembership.keys():
      if isValidZoneCenter(network.getNode(node)) and nodeZoneMembership[node] < MEMBERSHIP_LIM:
        candidateNodes = set()
        candidateBorders = set()
        collectNodes(candidateNodes, candidateBorders, network.getNode(node))

        if MEMBERSHIP_LIM not in [nodeZoneMembership[cand] for cand in candidateNodes]: # this zone will not break the limit
          newZone = Zone(i, node, network, candidateNodes, candidateBorders)
          i += 1
          zoneStore.append(newZone)
          for member in newZone.memberNodes:
            nodeZoneMembership[member] += 1
            # Each node belongs to a list of zones
            nodeToZones[member].append(newZone)

    done = 0 not in nodeZoneMembership.values()

  for zone in zoneStore:
    for node in zone.memberNodes:
      connectedZones = [ z for z in nodeToZones[node] if z != zone ]
      for z in connectedZones:
        if z in zone.connectedZones:
          zone.connectedZones[z].append(node)
        else:
          zone.connectedZones[z] = [node]

  return zoneStore, nodeToZones

def getExplicitEdges():
  return [ edge for edge in traci.edge.getIDList() if edge[0] != ":"]

def updateRoutePairsConcurrentCaller(zone, weights):
  zone.updateRoutePairs(weights)

def runThreads(threads):
  for t in threads:
    t.start()

  for t in threads:
    t.join()

def launchSim(zoneStore, nodeToZoneDict, network, sim, config):
  traci.start(sim)
  edges = getExplicitEdges()
  costStore = {value:0 for value in edges}
  traffic = {value:0 for value in edges}

  edgesStore = buildEdgesStore(network, edges)

  for _ in xrange(int(config.endtime)):

    for edge in edges:
      edgeOccupants = traci.edge.getLastStepVehicleIDs(edge)
      fromNode = network.getEdge(edge).getFromNode().getID()

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
    threads = []
    for z in zoneStore:
      threads.append(Thread(
        target=updateRoutePairsConcurrentCaller,
        args=(
          z,
          costStore,
          )
        )
      )
    runThreads(threads)

    for v in traci.vehicle.getIDList():
      currEdge = traci.vehicle.getRoadID(v)
      destNode = network.getEdge(traci.vehicle.getRoute(v)[-1]).getToNode()

      # Optimize at the last mile
      # Next, string zones together to find shortest paths between them
      # After that, smooth out the edges and start running experiments
      if currEdge in edges and network.getEdge(currEdge).getToNode != destNode: # No point in optimizing the path if v is about to hit destination
        nextNode = network.getEdge(currEdge).getToNode().getID()
        
        currZones = nodeToZoneDict[nextNode] 
        i = 0
        found = False
        while not found and i < len(currZones):
          selection = currZones[i]
          if destNode in selection.memberNodes:
            found = True
          i += 1

        if found:
          nodeList = selection.optimalRoutes[(nextNode, destNode)]
          traci.vehicle.setRoute(v, edgeListConvert(nodeList, edgesStore))

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

  network = sumolib.net.readNet(configPaths.networkfile)
  nodes = [node.getID() for node in network.getNodes()]

  zoneStore, nodeToZones = assignNodesToZones(network, nodes)
  exc = sumoGui if gui else sumoCmd
  launchSim(zoneStore, nodeToZones, network, exc, configPaths)

if __name__ == "__main__":
  starttime = time.time()
  main()
  print "Execution completed in %f seconds." % (time.time() - starttime)