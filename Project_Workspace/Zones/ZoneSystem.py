"""
Zone routing SUMO and TraCI.
The network is partitioned into zones,
the path between every pair of nodes within one zone is cached proactively (updated on changes),
the path between multiple zones is evaluated on-demand (reactively).

Ongoing challenges:
- Robust node-zone assignment algorithm
"""
# Still to do:
# Run experiments, bigger datasets

import copy
import datetime
import os
import random
import sys
import time

from threading import Thread

sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

import sumolib
import traci

NEIGHBOUR_LIM = 3 # A zone is constructed with n hops from center node 
MEMBERSHIP_LIM = 4 # A node may only be a member of up to m zones

class Zone(object):
  def __init__(self, zid, center, network, memberNodes=None, borderNodes=None):
    self.color = (
      random.sample( xrange(0, 255), 1)[0], 
      random.sample( xrange(0, 255), 1)[0], 
      random.sample( xrange(0, 255), 1)[0], 
      0
    )
    self.network = network
    self.center = center
    if memberNodes == None or borderNodes == None:
      self.memberNodes, self.borderNodes = self.buildZone(center)
    else:
      self.memberNodes = list(memberNodes)
      self.borderNodes = list(borderNodes)

    self.connectedZones = {} # Which zones _self_ connects to, and which nodes connect _self_ with other zones

    self.optimalRoutes = self.setupRoutePairs(self.memberNodes)
    self.zid = "z"+str(zid)

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
    return self.zid

  def __repr__(self):
    return self.zid


class SumoConfigWrapper(object):
  def __init__(self, configfile, guiOn):
    self.configfile = configfile
    self.networkfile, self.routefile, self.logfile, self.endtime = self.parsecfg(configfile)
    self.gui = guiOn

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
  if len(sys.argv) != 6:
    print "Usage: %s <ComplianceFactor> <ConfigFile> <0/1> <CsvFile> <0/1>" % sys.argv[0]
    exit(0)

  complianceFactor = float(sys.argv[1])
  config = sys.argv[2]
  gui = bool(int(sys.argv[3]))
  csvFile = sys.argv[4]
  verbose = bool(int(sys.argv[5]))
  return complianceFactor, config, gui, csvFile, verbose

def log(entry, verbose=True):
  if verbose:
    print "(%s): %s" % (datetime.datetime.utcnow(), entry)

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
def assignNodesToZones(network, nodes, verbose):
  log("Setting up Node-Zone assignment...", verbose)
  zoneStore = []
  nodeToZones = {value:[] for value in nodes}
  nodeZoneMembership = {value:0 for value in nodes}

  done = False
  i = 0

  while not done:
    for node in nodeZoneMembership.keys():
      if nodeZoneMembership[node] == 0:
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

          removeAll(zoneStore, removals)

          zoneStore.append(newZone)
          for member in newZone.memberNodes:
            nodeZoneMembership[member] += 1
            # Each node belongs to a list of zones
            nodeToZones[member].append(newZone)

    done = 0 not in nodeZoneMembership.values()

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

def launchSim(zoneStore, nodeToZoneDict, network, sim, config, verbose):
  print "Setting up execution..."
  traci.start(sim)
  edges = getExplicitEdges()
  costStore = {value:0 for value in edges}
  traffic = {value:0 for value in edges}

  edgesStore = buildEdgesStore(network, edges)

  for step in xrange(int(config.endtime)):
    log("step: %s" % step, verbose)
    for edge in edges:
      edgeOccupants = traci.edge.getLastStepVehicleIDs(edge)
      if config.gui:
        fromNode = network.getEdge(edge).getFromNode().getID()
        currZone = nodeToZoneDict[fromNode][0]
        color = currZone.color
        for agent in edgeOccupants:
          traci.vehicle.setColor(agent, color)

      # Update edge ph at each step
      edgelen = network.getEdge(edge).getLength()
      costStore[edge] = edgelen + traffic[edge]
      traffic[edge] += 0.2 * (sum([1/(traci.vehicle.getSpeed(v)+0.01) for v in edgeOccupants]))

      traffic[edge] = max(0, traffic[edge]/(0.2 * (edgelen/network.getEdge(edge).getSpeed())) )

    # With up-to-date traffic levels, update intrazone tables
    threads = []
    log("Setting intrazone routes with updated edge costs...", verbose)
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

    # Reroute vehicles given updated zones
    threads = []
    newRoutes = {}
    log("Assigning routes to vehicles...", verbose)
    for v in traci.vehicle.getIDList():

      currEdge = traci.vehicle.getRoadID(v)
      destNode = network.getEdge(traci.vehicle.getRoute(v)[-1]).getToNode().getID()

      threads.append(Thread(
        target=routeVehicle,
        args=(
          v,
          costStore,
          network,
          edgesStore,
          currEdge,
          destNode,
          newRoutes,
          nodeToZoneDict
          )
        )
      )
    runThreads(threads)

    for key in newRoutes:
      traci.vehicle.setRoute(key, newRoutes[key])

    traci.simulationStep()
    if len(traci.vehicle.getIDList()) < 1:
      log("Vehicles left simulation at step %s" % step, verbose)
      break

  traci.close(False)
  print "Completed Execution."

def routeVehicle(vehId, edgeCostStore, network, edgeStore, currEdge, destNode, newRoutesDict, nodeToZoneDict):
  if currEdge in edgeStore and network.getEdge(currEdge).getToNode.getID() != destNode:
    scoreTable = {"bestPath":[], "bestCost":sys.maxint}
    currPath = []
    srcNode = network.getEdge(currEdge).getToNode().getID()

    shortestZonePath([], srcNode, destNode, nodeToZoneDict, scoreTable, currPath, edgeStore, edgeCostStore, network)
    newRoute = edgeListConvert(scoreTable["bestPath"])
    newRoutesDict[vehId] = [currEdge]+newRoute

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

""" Setup and benchmarking """
def main():
  complianceFactor, config, gui, csvFile, verbose = parseArgs()
  configPaths = SumoConfigWrapper(config, gui)

  sumoGui = ["/usr/local/bin/sumo-gui", "-c", config]
  sumoCmd = ["/usr/local/bin/sumo", "-c", config]

  network = sumolib.net.readNet(configPaths.networkfile)
  nodes = [node.getID() for node in network.getNodes()]

  zoneStore, nodeToZones = assignNodesToZones(network, nodes, verbose)
  exc = sumoGui if gui else sumoCmd
  launchSim(zoneStore, nodeToZones, network, exc, configPaths, verbose)

if __name__ == "__main__":
  starttime = time.time()
  main()
  time.sleep(1)
  print "\n\n\n" # Buffer lines because SUMO also dumps to stdout
  print "Execution completed in %f seconds." % (time.time() - starttime)