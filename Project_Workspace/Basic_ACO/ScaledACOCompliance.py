"""
A WIP script to demo the performance of a very basic repellent-ACO system scaled to multiple vehicles.
The script tracks the following metrics, doing so with and without ACO in effect:
- Moments to destination for all vehicles
- Moments to destination for compliant vehicles

Arguments:
  - Percentage, an integer 1 < n < 100 identifying the percentage of agents to comply with ACO
  - Config File, the path to a .sumocfg file which containts necessary information about the simulation
  - A 0 or 1, indicating whether the simulation will run with the GUI (1 for GUI)
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
  def __init__(self, agentid, visitedEdges=None, lastEdge=None, destinationEdge=None):
    self.agentid = agentid
    self.visitedEdges = visitedEdges if visitedEdges else []
    self.lastEdge = lastEdge if lastEdge else ""
    self.destinationEdge = destinationEdge if destinationEdge else ""

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
import sumolib
import sys
import time
import traci
import xml.etree.ElementTree

BIG_NUM = 10000000000
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
  traci.vehicle.setRoute(agent.agentid, [currEdge, target])

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
      compliantAgents[trip.agent].destinationEdge = unsigned(trip.dest)

  print "Launching ACO simulation..."
  for _ in xrange(int(config.endtime)):
    presentAgents = traci.vehicle.getIDList()
    for v in presentAgents:
      ph[traci.vehicle.getRoadID(v)] += 1

    for compliantV in set(presentAgents).intersection(compliantAgents):
      traci.vehicle.setColor(compliantV, RED)

      currentEdge = traci.vehicle.getRoadID(compliantV)
      if compliantAgents[compliantV].lastEdge != currentEdge and unsigned(currentEdge) != compliantAgents[compliantV].destinationEdge:
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

""" Launch the simulation with the following optimizations:
 - Pheromone deposit and evaporation optimized
 - Reroute triggers a full reroute to destination, not a single edge
 - Reroute finds the shortest path considering a balance of distance and density
 - Reroute only triggers if density of route exceeds a threshold
"""
def executeOptimized(compliantAgents, config, startCommand):
  network = sumolib.net.readNet(config.networkfile)
  traci.start(startCommand)
  ph = {value:0 for value in traci.edge.getIDList()}

  for trip in getTrips(config.routefile):
    if trip.agent in compliantAgents:
      compliantAgents[trip.agent].destinationEdge = unsigned(trip.dest)

  print "Launching ACO simulation..."
  for _ in xrange(int(config.endtime)):
    presentAgents = traci.vehicle.getIDList()
    
    for e in ph:
      ph[e] = int(traci.edge.getLastStepVehicleNumber(e))

    for compliantV in set(presentAgents).intersection(compliantAgents):
      traci.vehicle.setColor(compliantV, RED)

      currentEdge = traci.vehicle.getRoadID(compliantV)
      if compliantAgents[compliantV].lastEdge != currentEdge and unsigned(currentEdge) != compliantAgents[compliantV].destinationEdge:
        reroute(
          compliantAgents[compliantV], 
          currentEdge, 
          network, 
          ph
        )
        compliantAgents[compliantV].lastEdge = currentEdge

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

  simulationResults = executeSimple(copy.deepcopy(compliantAgents), configPaths, sumoGui if gui else sumoCmd)
  print "Dumping metrics changes from simple ACO..."
  simulationResults.dumpMetrics()

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