#!/bin/bash

RED='\033[1;31m'
GREEN='\033[1;32m'
NC='\033[0m' # No Color

# Generate a CSV file and the swarm_environment.tex
if [ -f swarm.csv ]; then
  echo 'Generating CSV and configuration file.....'${GREEN}'ALREADY EXISTING'${NC}
else
  echo 'Generating CSV and configuration file.....'
  ~/workspace/swarm/tools/swarmlog_to_csv_comms.py swarm.log > swarm.csv
  if [ $? -eq 0 ]; then
    echo ${GREEN}OK${NC}
  else
    echo ${RED}FAIL${NC}
  fi
fi

# Generate images
echo 'Generating images'

for script in comms_msgs_sent.gplot comms_drops.gplot comms_datarate.gplot comms_neighbors.gplot
do
  echo -n '\t Executing gnuplot script ['${script}'].....'
  ./${script}
  if [ $? -eq 0 ]; then
    echo ${GREEN}OK${NC}
  else
    echo ${RED}FAIL${NC}
  fi
done

# Generate the PDF report.
echo -n 'Generating PDF.....'
pdflatex report > /dev/null
if [ $? -eq 0 ]; then
  echo ${GREEN}OK${NC}
else
  echo ${RED}FAIL${NC}
fi
