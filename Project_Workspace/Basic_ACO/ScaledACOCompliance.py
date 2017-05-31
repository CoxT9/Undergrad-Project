# This is a python script which executes the simple repellent-ACO system against an input road network.
# This script attempts to improve the following metrics:
# - Average overall travel time
# - Average travel time of participating/compliant vehicles.
# Travel time means "moments to desination"

# When the compliance to ACO is scaled, it must be noted that the entrances of vehicles is staggered (for realism and performance purposes)
# This program takes one argument: The percentage of vehicles which will comply to the simple repellent-ACO system

# Note that there are 385 vehicles in the current simple dataset

import os, sys, time, xml.etree.ElementTree, subprocess
from random import randint

sys.path.append(os.path.join(os.environ["SUMO_HOME"], 'tools'))
sumoGui = ["/usr/bin/sumo-gui", "-c", "./ACOSim.sumocfg"]
sumoCmd = ["/usr/bin/sumo", "-c", "./ACOSim.sumocfg"]

import traci, traci.constants, sumolib
network = sumolib.net.readNet("HighConnectivity.net.xml")

complianceFactor = (float(sys.argv[1])/100).as_integer_ratio()[1] # 1 in complianceFactor agents will be selected
print complianceFactor

# Collect all compliant vehicle ids:
e = xml.etree.ElementTree.parse('LongRoutes.rou.xml').getroot()
i = 1
compliantAgents = set()

agents = e.findall('vehicle')
while len(compliantAgents) < len(agents)/complianceFactor:
  index = randint(0, len(agents)-1)
  compliantAgents.add(agents[index].get('id'))

# We now have a collection of x % of all agents which will be compliant to the ACO system.
# Run the simulation twice:
# First run: Gather arrival data on "compliant" members and whole dataset
# Second run: Actually execute ACO on compliant members and gather same metrics
subprocess.call(sumoCmd, stderr = open(os.devnull, 'w'))
time.sleep(1)
print ""
print "Completed execution of non-ACO simulation"

overallAverageTime = 0
averageCompliantAgentTime = 0
for vehicle in sumolib.output.parse('Logs.out.xml', 'vehicle'):
  travelTime = float(vehicle.arrival) - float(vehicle.depart)
  if vehicle.id in compliantAgents:
    averageCompliantAgentTime += travelTime
  
  overallAverageTime += travelTime

overallAverageTime /= len(agents)
averageCompliantAgentTime /= len(compliantAgents)

print "Average time for all vehicles without any ACO compliance is", overallAverageTime
print "Average time for to-be-compliant subset of vehicles (without compliance) is", averageCompliantAgentTime

# So far this script gathers a percentage of the agent population as "ACO-compliant" agents. 
# The script then evaluates the average travel-time performance of the whole population and the compliant population,
# without any usage of ACO so far.

# Next, the simulation will execute with the target population complying to ACO.
#traci.start(sumoGui)

print format("Launching multi-vehicle-compliance scenario with %s%% population compliance..." % sys.argv[1])

traci.start(sumoGui)
ph = { value:0 for value in traci.edge.getIDList()}

# Will need keystores for all previously scalar data

for _ in xrange(1000):
  presentAgents = traci.vehicle.getIDList()
  for v in presentAgents:
    ph[traci.vehicle.getRoadID(v)] += 1
   
  for compliantV in set(presentAgents).intersection(compliantAgents):
    # Run ACO system for compliant vehicle
    currentEdge = traci.vehicle.getRoadID(compliantV)
    
  for e in ph:
    ph[e] = max(ph[e]-1, 0)

  traci.simulationStep()

traci.close(False)
print "Done." 
