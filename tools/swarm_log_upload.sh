#!/bin/bash

# Check that a team name is specified on the command line
if [ -z "$1" ]; then
  echo "Error: No team name specified"
  echo
  echo "Usage:"
  echo "    swarm_log_upload.sh <team_name>"
  echo
  echo "where <team_name> is one of: byu,gatech,nps,upenn"
  echo
  echo "Example:"
  echo "    swarm_log_upload.sh gatech"
  exit 0
fi

# Get the most recent swarm log directory
full_swarm_log=`ls -dt ~/.swarm/log/* 2>/dev/null | head -1`
if [ -z "$full_swarm_log" ]; then
  echo "No swarm logs found in ~/.swarm/log"
  exit 0
fi

# Get the most recent gazebo log directory
full_gazebo_log=`ls -dt ~/.gazebo/log/* 2>/dev/null | head -1`
if [ -z "$full_gazebo_log" ]; then
  echo "No gazebo logs found in ~/.gazebo/log"
  exit 0
fi

# Get the swarm and gazebo timestamps
swarm_timestamp=`basename $full_swarm_log`
gazebo_timestamp=`basename $full_gazebo_log`

# Get the timestamp down to the second resolution
swarm_timestamp_seconds=`echo $swarm_timestamp | awk -F'.' '{print $1}'`
gazebo_timestamp_seconds=`echo $gazebo_timestamp | awk -F'.' '{print $1}'`

# Print out some information
echo "Team: $1"
echo "Swarm log timestamp: $swarm_timestamp"
echo "Gazebo log timestamp: $gazebo_timestamp"

# Check that both the gazebo and swarm logs have similar times
if [ "$swarm_timestamp_seconds" != "$gazebo_timestamp_seconds" ]; then
  echo
  echo "WARNING"
  echo
  echo "The swarm and gazebo timestamps do not match to the second."
  echo "This may mean gazebo didn't record a log file. Make sure"
  echo "to use the -r command-line variable with 'gzserver' or "
  echo "'gazebo'."
  echo
  echo -n "Do you want to continue (y/N)? "
  read ans
  if [ "$ans" = "n" ] || [ "$ans" = "N" ] || [ -z "$ans" ]; then
    exit 0
  else
    echo
  fi
fi

# Create a working directory
mkdir /tmp/$swarm_timestamp 2>/dev/null

# Copy in the log files
cp $full_swarm_log/* /tmp/$swarm_timestamp
cp $full_gazebo_log/gzserver/* /tmp/$swarm_timestamp

# Output the directory contents
echo "Review contents of upload directory: /tmp/$swarm_timestamp"
echo "-----------"
ls /tmp/$swarm_timestamp
echo "-----------"

# Ask if they want to upload
echo -n "Upload /tmp/$swarm_timestamp to s3://osrf-swarm/$1/ (Y/n)? "
read ans

# Upload if yes
if [ "$ans" = "y" ] || [ "$ans" = "Y" ] || [ -z "$ans" ]; then
  s3cmd sync /tmp/$swarm_timestamp s3://osrf-swarm/$1/
fi
