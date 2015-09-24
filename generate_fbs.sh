#!/bin/bash

# generate_fbs.sh: A shell script to generate the cpp code from a directory
# with flatbuffers.

# define usage function
usage()
{
  echo "Usage: $0 <fbs_dir> <output_dir>"
  exit 1
}

# Call usage() function if command not supplied.
[[ $# -ne 2 ]] && usage

for FB in $1/*.fbs
do
  echo "Generating cpp code for file $FB"
  flatc -c --gen-mutable -o $2 $FB
done
