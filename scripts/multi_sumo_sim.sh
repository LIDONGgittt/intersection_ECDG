#!/bin/bash
## change directory and load modules ##
ROOT_PATH=/home/dong/workspace/intersection_CDG
cd $ROOT_PATH/release
cmake -DCMAKE_BUILD_TYPE=RELEASE ..
make sumo_sim_batch

geometryIDList=(1 2)
arrivalIntervalList=(1 2)
N=(5 10 100)
sumoStepLengthList=(0.01 0.01 0.05)

cd $ROOT_PATH/log
mkdir sumosim
cd sumosim

for numVehicle in ${N[@]}; do
    if [[ ! -d N${numVehicle} ]]; then
        mkdir N${numVehicle}
    fi
done

cd $ROOT_PATH/release

for (( idx = 0 ; idx < ${#N[@]}; idx++ )); do
    numVehicle=${N[idx]}
    sumo_step_length=${sumoStepLengthList[idx]}
    for geometryID in ${geometryIDList[@]}; do
        for arrival_interval_avg in ${arrivalIntervalList[@]}; do
            echo "./sumo_sim/sumo_sim_batch --num_nodes $numVehicle --geometryID ${geometryID} --arrival_interval_avg ${arrival_interval_avg} --sumo_step_length ${sumo_step_length} --test_count 30 &"  # --test_count 1
            ./sumo_sim/sumo_sim_batch --num_nodes $numVehicle --geometryID ${geometryID} --arrival_interval_avg ${arrival_interval_avg} --sumo_step_length ${sumo_step_length} --test_count 30 &  # --test_count 1
            sleep 2
        done
    done
    wait
done

## Wait for all of the background tasks to finish
wait
exit 0
