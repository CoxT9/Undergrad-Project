#!/bin/bash
# Execute the simulations being used:
# NonZones and Nones for 5 populations, so 10 execs in total

(./NonZones 100 500IA-100P.sumocfg 0 ZoneMetrics.csv) &&
(./NonZones 100 500IA-200P.sumocfg 0 ZoneMetrics.csv) &&
(./NonZones 100 500IA-300P.sumocfg 0 ZoneMetrics.csv) &&
(./NonZones 100 500IA-400P.sumocfg 0 ZoneMetrics.csv) &&
(./NonZones 100 500IA-500P.sumocfg 0 ZoneMetrics.csv) &&
(./ZoneSystem 500IA-100P.sumocfg 0 ZoneMetrics.csv 0) &&
(./ZoneSystem 500IA-200P.sumocfg 0 ZoneMetrics.csv 0) &&
(./ZoneSystem 500IA-300P.sumocfg 0 ZoneMetrics.csv 0) &&
(./ZoneSystem 500IA-400P.sumocfg 0 ZoneMetrics.csv 0) &&
(./ZoneSystem 500IA-500P.sumocfg 0 ZoneMetrics.csv 0)