#!/bin/bash

make > /dev/null

echo "problem,time,cost,optimal"

alpha="$1"
iter="$2"

for file in cvrp/vrp-A/*.vrp; do
    for i in 1 2 3 4 5 6 7 8; do
        output=$( (./grasp_cvrp "$file" "$alpha" "$iter";) 2>&1 )
        time=$( (echo "$output" | grep 'Time';) 2>&1)
        time=${time:5:7}
        cost=$( (echo "$output" | grep 'Cost';) 2>&1)
        cost=${cost#"Cost "}
        optimal=$( (cat ${file%.*}.sol | grep "Cost";) 2>&1 )
        optimal=${optimal#"Cost "}
        problem=${file##*/}
        problem=${problem%.*}
        echo -e "$problem,$time,$cost,$optimal"
    done
done