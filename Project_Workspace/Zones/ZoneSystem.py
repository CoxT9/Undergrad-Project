# Working with partitioning a SUMO network into zones.

# First, assume a zone has a radius of two. pick a node and use it to form a zone.
# Some code repition from the first ACO-SUMO project but that's alright
""" Next:
- Reduce zone overlap
- Stricter partitioning (zone routes should not leave zone)
- Zone traversal
- Need to parallelize 
"""

import copy
import os
import random
import sys
import time

from threading import Thread

sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

import sumolib
import traci

NEIGHBOUR_LIM = 5 # A zone is constructed with r hops from center
MEMBERSHIP_LIM = 4 # A node may only be a member of up to n nodes

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
    # This is not overlapping nodes, this is overlapping or adjacent nodes. need to store the cost from z1 to z2 (ie: the cost of the edge that connects the two nodes)
    # In fact this probably isn't even necessary. Could think about which zones each member node connects to.

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

def removeAll(lst, itemsToRemove):
  newList = []
  for l in lst:
    if l not in itemsToRemove:
      newList.append(l)

  return newList

def nonePresent(subList, largerList):
  return len(list(set(largerList).difference(subList))) == len(largerList)

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
  collection.add(centerNode.getID())

  if stackdepth < NEIGHBOUR_LIM:
    neighbours = [edge.getToNode() for edge in centerNode.getOutgoing() if edge.getToNode().getID() not in collection]
    for n in neighbours:
      if n not in collection and stackdepth == NEIGHBOUR_LIM-1:
        borderNodes.add(n.getID())
      collection.add(n.getID())
    
    for n in neighbours:
      collectNodes(collection, borderNodes, n, stackdepth+1)



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

""" Dijkstra's scaled up to zones """
def shortestInterZonePathOLD(srcZones, srcNode, destNode, nodeToZoneMap, weights, edgesStore):
  # Literally run dijkstra's and link together paths from zones
  # Make sure to swap the vertex list back into an edge list for traci api
  #path = [srcNode]
  srcZone = srcZones[0]

  if srcNode == destNode:
    return [destNode]

  unvisited = {srcZone}
  visited = set()

  totalCost = {srcZone: 0}
  candidates = {}

  while unvisited:
    # Pick smallest weight
    current = min( [ (totalCost[zone], zone) for zone in unvisited] )[1]
    if current in nodeToZoneMap[destNode]:
      break

    unvisited.discard(current)
    visited.add(current)

    connectedZonesToEdges = {}
    #for edge in network.getNode(current).getOutgoing():
    for neighbour in current.connectedZones:
      connectorEdges = current.connectedZones[neighbour]
      # for now, take min cost. If a path exists between 2 zones, take the cheapest edge?
      # This should not use dijkstra. We actually should work with the HOPNET approach
      # build up paths until at dest node, then decide which path is shorter from reverse
      connectorEdge = min(connectorEdges, key=lambda k: weights[k])
      connectedZonesToEdges[neighbour] = connectorEdge

    unvisitedNeighbours = set(connectedZonesToEdges.keys()).difference(visited)
    for neighbour in unvisitedNeighbours:
      nei_dist = totalCost[current] + weights[connectedZonesToEdges[neighbour]]
      if nei_dist < totalCost.get(neighbour, float('inf')):
        totalCost[neighbour] = nei_dist
        candidates[neighbour] = current
        unvisited.add(neighbour)

  # Need to string together path of nodes somehow
  result = gatherZonePaths(candidates, destNode, nodeToZoneMap)

  print srcZone
  print nodeToZoneMap[destNode]
  print candidates
  for z in nodeToZoneMap[destNode]:
    print z, candidates[z]

  print result
  # result gives us a list of zone sequences. We have to find the cheapest one
  # Find the cheapest node by working backwards
  # here's the trick: this thing just gave us multiple paths
  # look at all the paths, take only the minimal overall.
  paths = []
  for zoneSeq in result:
    # first entry has the source
    for index, zone in enumerate(zoneSeq):
      print zoneSeq

  print srcZone
  print "------------"

  print srcZones
  print nodeToZoneMap[destNode]
  print srcNode
  print destNode
  for z in srcZones:
    print z, sorted([int(x) for x in z.memberNodes])

  for zp in nodeToZoneMap[destNode]:
    print zp, sorted([int(y) for y in zp.memberNodes])

  print "-/-/-/-/-/-/-/--//--/-/-/-/-/-/-"
  paths = [ [] ]
  for z in nodeToZoneDict[srcNode]:
    getInterZonePaths(srcNode, destNode, z, nodeToZoneDict, paths)
  print "ggggg"
  return "foo"
  #return result


def interZonePaths(srcNode, destNode, srcZone, nodeToZoneMap):
  # srcZone is the (a) zone srcNode belongs in
  if destNode == srcNode:
    return [destNode]
  elif destNode in srcZone.memberNodes:
    return srcZones.optimalRoutes[(srcNode, destNode)]
  else:
    for neighbour in srcZone.connectedZones:
      borderNodes = srcZone.connectedZones[neighbour]

""" Zone-Traversal utility function """
def gatherZonePaths(candidates, destNode, nodeToZoneMap):
  paths = []
  for dest in nodeToZoneMap[destNode]:
    if dest in candidates:
      goal = dest
      path = []
      while goal:
        path.append(goal)
        goal = candidates.get(goal)
    paths.append(list(reversed(path)))
  return paths

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
          removals = []
          for existingZone in zoneStore:
            # If our new zone is a superset of some zone z, we remove z.
            if set(newZone.memberNodes).issuperset(existingZone.memberNodes):
              removals.append(existingZone)

            removeAll(zoneStore, existingZone)

          zoneStore.append(newZone)
          for member in newZone.memberNodes:
            nodeZoneMembership[member] += 1
            # Each node belongs to a list of zones
            nodeToZones[member].append(newZone)

    done = 0 not in nodeZoneMembership.values()

  # for zone in zoneStore:
  #   for node in zone.memberNodes:
  #     # Take all the outgoing edges of this node. See what zones that edge links to. These are the zone's neighbours
  #     outgoingEdges = [edge.getID() for edge in network.getNode(node).getOutgoing()]
  #     for edge in outgoingEdges:
  #       toNode = network.getEdge(edge).getToNode().getID()
  #       endpointZones = nodeToZones[toNode]

  #       for z in endpointZones:
  #         if z != zone: # Avoid 1 zone cycles
  #           if z in zone.connectedZones:
  #             zone.connectedZones[z].append(edge)  # this should be a node. 1 zone cycle?
  #           else:
  #             zone.connectedZones[z] = [edge]

  # for z in zoneStore:
  #   for bord in z.borderNodes:
  #     for zprime in nodeToZones[bord]:
  #       if zprime != z:
  #         if zprime in z.connectedZones:
  #           z.connectedZones[zprime].append(mem)
  #         else:
  #           z.connectedZones[zprime] = [mem]

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
    print _

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

      # String zones together for shortest path
      if currEdge in edges and network.getEdge(currEdge).getToNode != destNode: # No point in optimizing the path if v is about to hit destination
        startNode = network.getEdge(currEdge).getToNode().getID()
        srcNode = startNode
        resultPath = []
        currPath = []

        scoreTable = {"bestPath":[], "bestCost":sys.maxint}
       # print srcNode
       # print destNode.getID()
        shortestZonePath([], srcNode, destNode.getID(), nodeToZoneDict, scoreTable, currPath, edgesStore, costStore, network)
        resultPath = scoreTable["bestPath"]

        # it runs way too slow and finds way too many paths

        # prune to min cost path
       # exit(0)
        newRoute = edgeListConvert(resultPath, edgesStore)

        traci.vehicle.setRoute(v, [currEdge]+newRoute)
        #raise Exception("foo")
        #traci.vehicle.setRoute(v, shortestInterZonePathOLD(nodeToZoneDict[startNode], startNode, destNode.getID(), nodeToZoneDict, costStore, edgesStore) )

    traci.simulationStep()
    if len(traci.vehicle.getIDList()) < 1:
      break

  traci.close(False)
  print "Completed Execution."

def getPathCost(vertexList, edgesStore, weights):
  return sum(weights[e] for e in edgeListConvert(vertexList, edgesStore))

# This is hacky. Unnecessary pointer voodoo
def shortestZonePath(visitedZones, srcNode, destNode, nodeToZoneDict, scoreTable, currPath, edgesStore, weights, network):
  pathToDest = []
  if srcNode == destNode:
    pathToDest = [destNode]
  else:
    srcZones = nodeToZoneDict[srcNode]
    for z in srcZones:
      if destNode in z.memberNodes:
        pathToDest = z.optimalRoutes[(srcNode, destNode)]

  if len(pathToDest) > 0 and nonePresent(pathToDest[1:], currPath): # append and return
    bestCost = scoreTable["bestCost"]
    bestPath = scoreTable["bestPath"]
    candidatePath = currPath[:-1] + pathToDest
    candidateCost = getPathCost(candidatePath, edgesStore, weights)
    if candidateCost < bestCost:
      scoreTable["bestPath"] = candidatePath
      scoreTable["bestCost"] = candidateCost
    return # we found a route. no need to expand to more zones

  # effecetively an else case

  visitedZones.extend(nodeToZoneDict[srcNode])
  # now expand to multi zone. If the dest node was not in our zone, send the request to each neighbour
  for srcZone in nodeToZoneDict[srcNode]:

    for border in srcZone.borderNodes:
      if border == srcNode:
        pathToBorder = [border]
      else:
        pathToBorder = srcZone.optimalRoutes[(srcNode, border)]

      if len(network.getNode(border).getOutgoing()) > 1 and nonePresent(pathToBorder[1:], currPath): # We haven't looked at this path yet
        newPath = currPath[:-1] + pathToBorder

        # mark each neighbour as visited and try them
        zonesToVisit = list(set(nodeToZoneDict[border]).difference(visitedZones))
        if len(zonesToVisit) > 0:
          shortestZonePath (
            visitedZones + zonesToVisit,
            border,
            destNode,
            nodeToZoneDict,
            scoreTable,
            newPath,
            edgesStore,
            weights,
            network
          )


def getInterZonePaths_old(visitedZones, srcNode, destNode, srcZone, nodeToZoneDict, paths, currPath):
  # paths is an array of arrays
 # for p in paths:
 #   if destNode in p:
 #     return

  if srcNode == destNode:
    paths.append(currPath[:-1] + [destNode])
  elif destNode in srcZone.memberNodes:
    paths.append(currPath[:-1] + srcZone.optimalRoutes[(srcNode, destNode)] )
  else:
    for borderNode in srcZone.borderNodes:
     # print nodeToZoneDict[borderNode]
     # print visitedZones
      for neighbour in nodeToZoneDict[borderNode]:
        if borderNode not in currPath and borderNode != srcNode and neighbour not in visitedZones and neighbour != srcZone:
          getInterZonePaths (
            visitedZones+[neighbour], 
            borderNode, 
            destNode, 
            neighbour, 
            nodeToZoneDict, 
            paths, 
            currPath[:-1]+srcZone.optimalRoutes[(srcNode,borderNode)], 
          )
  # need to shorten run time by cutting down on recursion depth. Something is far too unbounded here


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