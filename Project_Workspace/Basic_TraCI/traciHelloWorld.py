# A First take on TraCI: Grabbing Vehicle ID and Coordinates every 10 simulation steps

import os, sys, time

if 'SUMO_HOME' in os.environ:
  tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
  sys.path.append(tools)
else:
  sys.exit("Missing SUMO_HOME env var!")

sumoGui = "/usr/bin/sumo-gui"
sumoCmd = [sumoGui, "-c", "./Basic.sumocfg"]

import traci
import traci.constants as tc

traci.start(sumoCmd)

for i in range(100):
  traci.simulationStep()
  time.sleep(3)
  print "Step!"

traci.close(False)
