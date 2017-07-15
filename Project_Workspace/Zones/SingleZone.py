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

class Zone(object):
  def __init__(self, center, network):
    self.network = network
    self.center = center
    self.memberNodes = self.buildZone(center)

  def buildZone(self, center):
    i = 0
    centerNode = self.network.getNode(center)
    nodes = set()
    nodes.add(centerNode)
    self.collectNodes(nodes, centerNode, i)
    print [node.getID() for node in nodes]

  # for now, zones are effectively circular
  def collectNodes(self, collection, centerNode, stackdepth):
    if stackdepth < NEIGHBOUR_LIM:
      neighbours = [edge.getToNode() for edge in centerNode.getOutgoing()]
      for n in neighbours:
        self.collectNodes(collection, n, stackdepth+1)

    collection.add(centerNode)

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

""" Setup and benchmarking """
def main():
  complianceFactor, config, gui, csvFile = parseArgs()
  configPaths = SumoConfigWrapper(config)

  sumoGui = ["/usr/local/bin/sumo-gui", "-c", config]
  sumoCmd = ["/usr/local/bin/sumo", "-c", config]

  # Pick a node
  network = sumolib.net.readNet(configPaths.networkfile)
  nodes = [ i.getID() for i in network.getNodes() ]

  zoneCenter = random.choice(nodes)
  myZone = Zone(zoneCenter, network)
  print zoneCenter

if __name__ == "__main__":
  starttime = time.time()
  main()
  print "Execution completed in %f seconds." % (time.time() - starttime)