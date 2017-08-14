#!/bin/bash

inputFile="$1"
outputPath="$2"

if [ "$inputFile" == "" ]
then
  echo "Usage:"
  echo " $0 /path/to/input.txt /path/to/output/dir/"
  echo "   Arguement 2, output path, is optional and if omitted output will save"
  echo "   to current directory"
  exit 1
fi

for testPrefix in `grep -o "Test_.*_" $inputFile | sort | uniq`
do
  if [ ! -e $outputPath$testPrefix"results.csv" ]
  then
    grep "$testPrefix" $inputFile | sort > $outputPath$testPrefix"results.csv"
  else
    echo "Error: output file exists"
  fi
done
