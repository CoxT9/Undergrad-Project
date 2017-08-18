"""
Zone routing SUMO and TraCI.
The network is partitioned into zones,
the path between every pair of nodes within one zone is cached proactively (updated on changes),
the path between multiple zones is evaluated on-demand (reactively).

Ongoing challenges:
- Slow performance in large scale data
- No complete experimental output yet produced
"""

import copy
import datetime
import itertools
import os
import random
import sys
import time

from threading import Thread
from threading import Lock

sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

import sumolib
import traci

# need to keep this value down
NEIGHBOUR_LIM = 2 # A zone is constructed with n hops from center node 

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
    self.optimalRoutes = {}
    self.zid = "z"+str(zid)

  def buildZone(self, center):
    i = 0
    centerNode = self.network.getNode(center)
    nodes = set()
    borderNodes = set()
    nodes.add(centerNode)
    collectNodes(nodes, borderNodes, centerNode, i)
    return list(nodes), list(borderNodes)

  """ Route Maintenance Step """
  def updateRoutePairs(self, weights, globalEdges):
    nodes = self.memberNodes
    edges = {}
    for e in globalEdges:
      if e[0] in nodes and e[1] in nodes:
        edges[e] = globalEdges[e]

    paths = self.zoneFloydWarshall(nodes, edges, weights)

    for pair in itertools.permutations(self.memberNodes, 2):
      path = self.unpackFWPath(pair, paths)
      self.optimalRoutes[(pair[0], pair[1])] = path

  """ Single Zone Floyd Warshall """
  def zoneFloydWarshall(self, nodes, edges, weights):
    dists = {}
    nexts = {}
    for n in nodes:
      for n2 in nodes:
        dists[(n, n2)] = sys.maxint 
        nexts[(n, n2)] = None
      dists[(n, n)] = 0

    for edge in edges:
      dists[(edge[0], edge[1])] = weights[edges[edge]]
      nexts[(edge[0], edge[1])] = edge[1]

    for i in range(len(nodes)):
      for j in range(len(nodes)):
        for k in range(len(nodes)):

          if dists[(nodes[j], nodes[k])] > dists[(nodes[j], nodes[i])] + dists[(nodes[i], nodes[k])]:
            dists[(nodes[j], nodes[k])] = dists[(nodes[j], nodes[i])] + dists[(nodes[i], nodes[k])]
            nexts[(nodes[j], nodes[k])] = nexts[(nodes[j], nodes[i])]

    return nexts

  """ Floyd Warshasll Utility. Take tuple @pair and array @nexts and turn into list of vertices """
  def unpackFWPath(self, pair, nexts):
    if nexts[(pair[0], pair[1])] == None:
      return []

    curr = pair[0]
    path = [curr]
    while curr != pair[1]:
      curr = nexts[(curr, pair[1])]
      path.append(curr)
    return path

  def setPair(self, src, dest, routes, weights):
    routes[(src, dest)] = dijkstra(src, dest, self.network, weights)

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
    log("Usage: %s <ComplianceFactor> <ConfigFile> <0/1> <CsvFile> <0/1>" % sys.argv[0])
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

""" Dijkstra utility function """
def unpackPath(candidates, dest):
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
  nodeAssigned = {value:False for value in nodes}
  nodeZoneMembership = {value:0 for value in nodes}
  i = 0
  for node in nodes:
    if not nodeAssigned[node]:

      candidateNodes = set()
      candidateBorders = set()

      collectNodes(candidateNodes, candidateBorders, network.getNode(node))
      newZone = Zone(i, node, network, candidateNodes, candidateBorders)

      i += 1
      zoneStore.append(newZone)

      for member in newZone.memberNodes:
        nodeAssigned[member] = True
        nodeToZones[member].append(newZone)
        nodeZoneMembership[member] += 1

  return zoneStore, nodeToZones

def getExplicitEdges():
  return [ edge for edge in traci.edge.getIDList() if edge[0] != ":"]

def runThreads(threads):
  for t in threads:
    t.start()

  for t in threads:
    t.join()

def setVehicleColors(network, nodeToZoneDict, edge, edgeOccupants):
  fromNode = network.getEdge(edge).getFromNode().getID()
  currZone = min(nodeToZoneDict[fromNode])
  color = currZone.color
  for agent in edgeOccupants:
    traci.vehicle.setColor(agent, color)

def launchSim(zoneStore, nodeToZoneDict, network, sim, config, verbose):
  log("Setting up execution...", verbose)
  traci.start(sim)
  edges = getExplicitEdges()

  costStore = {value:0 for value in edges}
  traffic = {value:0 for value in edges}
  edgesStore = buildEdgesStore(network, edges)

  for step in xrange(int(config.endtime)):
    log("step: %s" % step, verbose)
    zonesNeedingUpdate = set()

    # This is our edge-cost system. Inspired by DTPOS and IACO. Used as an example for the zone algorithm
    for edge in edges:
      originalCost = costStore[edge]
      edgeOccupants = traci.edge.getLastStepVehicleIDs(edge)
      speedData = [traci.vehicle.getSpeed(v) for v in edgeOccupants]
      if config.gui:
        setVehicleColors(network, nodeToZoneDict, edge, edgeOccupants)

      # Update edge ph at each step
      edgelen = network.getEdge(edge).getLength()
      costStore[edge] = edgelen + traffic[edge]
      traffic[edge] += 0.2 * (sum([1/(entry+0.01) for entry in speedData]))
      traffic[edge] = max(0, traffic[edge]/(0.2 * (edgelen/network.getEdge(edge).getSpeed())) )

      if originalCost != costStore[edge]:
        zonesNeedingUpdate |= set(nodeToZoneDict[network.getEdge(edge).getToNode().getID()])
        zonesNeedingUpdate |= set(nodeToZoneDict[network.getEdge(edge).getFromNode().getID()])

    threads = []
    log("Setting intrazone routes with updated edge costs...", verbose)
    for z in list(zonesNeedingUpdate):
      threads.append(Thread(
        target=z.updateRoutePairs,
        args=(costStore,edgesStore)
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
      # is there a faster way to do this? likely lots of repitition
      threads.append(Thread(
        target=routeVehicle,
        args=(
          v,
          costStore,
          network,
          edgesStore,
          edges,
          currEdge,
          destNode,
          newRoutes,
          sum(costStore[e] for e in traci.vehicle.getRoute(v)[1:]),
          nodeToZoneDict
          )
        )
      )
    runThreads(threads)

    for key in newRoutes:
      traci.vehicle.setRoute(key, newRoutes[key])

    traci.simulationStep()
    print len(traci.vehicle.getIDList())
    if len(traci.vehicle.getIDList()) < 1:
      log("Vehicles left simulation at step %s" % step, verbose)
      break

  traci.close(False)
  log("Completed Execution.", verbose)

def routeVehicle(vehId, edgeCostStore, network, edgeStore, edges, currEdge, destNode, newRoutesDict, currCost, nodeToZoneDict):
  if currEdge in edges:
    srcNode = network.getEdge(currEdge).getToNode().getID()
    if srcNode != destNode:
      scoreTable = {"bestPath":[], "bestCost":currCost} # this needs to be vehicles existing route. Otherwise we end up with all kinds of insanity
      currPath = []
      srcNode = network.getEdge(currEdge).getToNode().getID()

      shortestZonePath(set(), srcNode, destNode, nodeToZoneDict, scoreTable, currPath, edgeStore, edgeCostStore, network)
      finalPath = scoreTable["bestPath"]
      if len(finalPath) > 0:
        newRoute = edgeListConvert(finalPath, edgeStore)
        newRoutesDict[vehId] = [currEdge]+newRoute

def getPathCost(vertexList, edgesStore, weights):
  return sum(weights[e] for e in edgeListConvert(vertexList, edgesStore))

# Some pointer voodoo. Hold on tight!
# This is insane fast now. Something else broken?
# Vehicles made it out. Make sure it worked though!!
def shortestZonePath(visitedZones, srcNode, destNode, nodeToZoneDict, scoreTable, currPath, edgesStore, weights, network):
  srcZones = nodeToZoneDict[srcNode]
  possibleZones = filter(lambda z: destNode in z, srcZones)

  if(len(possibleZones)) > 0: # Destination found in src's zone.
    possibleRoutes = map(lambda z: z.optimalRoutes[(srcNode, destNode)], possibleZones)
    currPathCost = getPathCost(currPath, edgesStore, weights)
    pathToDest = currPath[:-1] + min(possibleRoutes, key=lambda k: getPathCost(k, edgesStore, weights) )
    currentCost = getPathCost(pathToDest, edgesStore, weights)

    storedCost = scoreTable["bestCost"]
    if currentCost < storedCost:
      scoreTable["bestPath"] = pathToDest
      scoreTable["bestCost"] = currentCost

  else: # Destination not found in src's zones.
    # So all the src zones don't have the destination. Mark them visited.
    visitedZones |= set(srcZones)
    threads = []

    # Let's see if any of our source zones' unvisited neighbours will get us a path:
    for srcZone in srcZones:
      # The borders will show us who the neighbours are
      for border in srcZone.borderNodes:
        if len(network.getNode(border).getOutgoing()) > 1 and set(nodeToZoneDict[border]).difference(visitedZones) != set():
          pathToBorder = [border] if border == srcNode else srcZone.optimalRoutes[(srcNode, border)]

          if nonePresent(pathToBorder, currPath):
            # We haven't looked at this path yet
            newPath = currPath + pathToBorder

            shortestZonePath(
                visitedZones,
                border,
                destNode,
                nodeToZoneDict,
                scoreTable,
                newPath,
                edgesStore,
                weights,
                network,
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

  # also need csv output
  # route timings and total exec timings, size of graph and population too

if __name__ == "__main__":
  starttime = time.time()
  main()
  time.sleep(1)
  log("\n\n\nExecution completed in %f seconds." % (time.time() - starttime))