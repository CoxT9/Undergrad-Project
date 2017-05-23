# A targeted re route in TraCI: Select a specific vehicle and re-route based on travel-time, see if there is a difference in route.

import os, sys, time 
from random import randint

if 'SUMO_HOME' in os.environ:
  tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
  sys.path.append(tools)
else:
  sys.exit("Missing SUMO_HOME env var!")

sumoGui = "/usr/bin/sumo-gui"
sumoCmd = [sumoGui, "-c", "./Basic.sumocfg"]

vehid = "601"

import traci
# Traci constants ref: http://www.sumo.dlr.de/pydoc/traci.constants.html
import traci.constants as tc

traci.start(sumoCmd)
print "Starting simulation..."

# run 100 steps
for i in range(100):
  traci.simulationStep()

# Dump route
print traci.vehicle.getRoute(vehid)
# Reroute
traci.vehicle.rerouteTraveltime(vehid)
# Dump new route
print traci.vehicle.getRoute(vehid)
# Run last 100 steps
for i in range(100):
  traci.simulationStep()

traci.close(False)
