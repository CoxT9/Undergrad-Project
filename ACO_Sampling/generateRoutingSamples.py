# quick python script to build 10 route files for the input network files.
import os
import sys
sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

import sumolib

from subprocess import call, check_call

def getNumExplicitEdges(networkFile):
  network = sumolib.net.readNet(networkFile)
  return len( [edge for edge in network.getEdges() if edge.getID()[0] != ":"] )

networkFiles = [x.strip() for x in sys.argv[1:]]
for network in networkFiles:
  INCR = 0.3
  i = 0
  filePrefix = network[0:4] + ("A" if "A" in network else "B")

  # .4 .8 1.2 1.6 2.0 2.4 2.8 3.2 3.6 4.0
  factor = INCR
  total = getNumExplicitEdges(network)
  while i < 10:
    factorStr = str(factor*100)

    portionPrefix = factorStr[:factorStr.index(".")]+"P"
    fileIdentifier = filePrefix + "-" + portionPrefix

    tripOutput = fileIdentifier + ".trip.xml"
    routeOutput = "routes/" + fileIdentifier + ".rou.xml"
    cfgOutput = fileIdentifier + ".sumocfg"
    logOutput = "logs/" + fileIdentifier + ".logs.xml"

    print "Setting up trips file", tripOutput
    print total*factor
    call([
      "randomTrips", 
      "-n", network, 
      "-o", tripOutput, 
      "-p", "0.01", 
      "-e", str( (total*factor)/100 ), 
      "--validate"
    ])

    print "Setting up routes file", routeOutput
    call([
      "duarouter", 
      "--net-file", network, 
      "--output-file", routeOutput, 
      "--repair", "true", 
      "--ignore-errors", "true", 
      "--trip-files", tripOutput, 
      "--departlane", "best",
      "--remove-loops", "true",
      "--departpos", "last"
    ])

    print "Setting up cfg file", cfgOutput 
    call([
      "sumo",
      "-n", network,
      "-r", routeOutput,
      "--vehroute-output", logOutput,
      "--vehroute-output.exit-times", "1",
      "-b", "0",
      "-e", "10800", # Three hours
      "--time-to-teleport", "-1",
      "--max-depart-delay", "-1",
      "--waiting-time-memory", "10800",
      "--save-configuration", cfgOutput
    ])

    print "Cleaning up..."
    check_call("rm -f *trip* *routes/alt*", shell=True)

    factor += INCR
    i += 1
"""
netgenerate -r --rand.iterations 100 --random t --output-file 100IterationsA.net.xml --tls.guess true
netgenerate -r --rand.iterations 100 --random t --output-file 100IterationsB.net.xml --tls.guess true
netgenerate -r --rand.iterations 250 --random t --output-file 250IterationsA.net.xml --tls.guess true
netgenerate -r --rand.iterations 250 --random t --output-file 250IterationsB.net.xml --tls.guess true
netgenerate -r --rand.iterations 500 --random t --output-file 500IterationsA.net.xml --tls.guess true
netgenerate -r --rand.iterations 500 --random t --output-file 500IterationsB.net.xml --tls.guess true
"""