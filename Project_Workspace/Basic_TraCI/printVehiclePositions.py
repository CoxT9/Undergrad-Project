# A First take on TraCI: Grabbing Vehicle ID and Coordinates every 10 simulation steps

import os, sys, time 
from random import randint

if 'SUMO_HOME' in os.environ:
  tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
  sys.path.append(tools)
else:
  sys.exit("Missing SUMO_HOME env var!")

sumoGui = "/usr/bin/sumo-gui"
sumoCmd = [sumoGui, "-c", "./Basic.sumocfg"]

import traci
# Traci constants ref: http://www.sumo.dlr.de/pydoc/traci.constants.html
import traci.constants as tc

def watchRoutes(start, end):
  for i in range(start, end):
    if i % 5 == 0:
      for j in traci.vehicle.getIDList():
        print traci.vehicle.getRoute(j)
        print traci.vehicle.getRoadID(j)

    print "-----"
    traci.simulationStep()
    time.sleep(0.5)

traci.start(sumoCmd)


print "Starting simulation..."
watchRoutes(0, 25)
# at the 50th step, change the effort of all edges and trigger a re-route on effort for all vehicles
for edgeid in traci.edge.getIDList():
  traci.edge.setEffort(edgeid, randint(0, 999), 0, 100)

for j in traci.vehicle.getIDList():
  traci.vehicle.rerouteEffort(j)

watchRoutes(25, 50)

traci.close(False)
