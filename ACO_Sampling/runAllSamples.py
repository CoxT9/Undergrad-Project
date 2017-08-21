# Execute sumo for each cfg file in directory
#from /mnt/c/Users/Taylor/Documents/School/COMP4520/Project_Workspace/Basic_ACO/ScaledACOCompliance import ScaledACOCompliance
import sys
import os
import subprocess
import time
import ACOModel as model
inittime = time.time()
configFiles = sys.argv[1:-1]
csvFile = sys.argv[-1]
print configFiles
#sys.exit(0)

csvPath = os.path.join(os.getcwd(), csvFile)

# Need to run for all adherence percentages
# Need to run this in a blocking manner (for serial file writes)

for cfg in configFiles:
  # 0, 25, 50, 100
  for portion in [100]:
    print cfg
    model.main([str(portion), cfg, "0", csvPath])

print "Metrics gathering complete!"
print format("Time elapsed: %s seconds" % str(time.time() - inittime))
