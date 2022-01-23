#!/bin/bash

make

for file in cvrp/vrp-A/*.vrp; do
    output=$( (./grasp_cvrp "$file";) 2>&1 )
    echo -e "Running $file...\n"
    echo -e "$output" | grep "Time"
    echo -e "$output" | grep "Cost"
    optimal=$( (cat ${file%.*}.sol | grep "Cost";) 2>&1 )
    echo -e "Optimal $optimal"
    echo -e ""
done