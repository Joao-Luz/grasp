#!/bin/bash

make > /dev/null

for alpha in "0.2" "0.4" "0.5" "0.6" "0.8"; do
    echo -e "Alpha = $alpha"
    echo "problem,time,cost,optimal" > "test/data/tests_$alpha.csv"
    for file in cvrp/vrp-*/*.vrp; do
        echo -e "Running $file..."
        for i in 1 2 3 4 5 6 7 8; do
            output=$( (./grasp_cvrp "$file" --alpha "$alpha" --iter 200;) 2>&1 )
            time=$( (echo "$output" | grep 'Time';) 2>&1)
            time=${time:5:7}
            cost=$( (echo "$output" | grep 'Cost';) 2>&1)
            cost=${cost#"Cost "}
            optimal=$( (cat ${file%.*}.sol | grep "Cost";) 2>&1 )
            optimal=${optimal#"Cost "}
            problem=${file##*/}
            problem=${problem%.*}
            echo -e "$problem,$time,$cost,$optimal" >> "test/data/tests_$alpha.csv"
        done
    done
done