#!/bin/bash

make

for file in cvrp/vrp-A/*.vrp; do
    echo -e "------------------------------------\n"
    echo -e "Running $file\n"
    ./grasp_cvrp "$file"
    echo -e "\n"
    cat ${file%.*}.sol
done