#!/bin/bash
#SBATCH -J SUMO_sim
#SBATCH --account=ev_charging
#SBATCH --partition=normal_q
#SBATCH --nodes=1 --ntasks-per-node=24 --cpus-per-task=1
#SBATCH --time=0-10:00:00 # day-hour:minute:second

#SBATCH --mail-user=dongli@vt.edu
#SBATCH --mail-type=BEGIN  # send email when job begins
#SBATCH --mail-type=END    # send email when job ends
#SBATCH --mail-type=FAIL   # send email when job aborts

echo "Current job id is: $SLURM_JOB_ID"

## change directory and load modules ##
ROOT_PATH=/home/dongli/workspace/intersection_CDG
# ROOT_PATH=/home/dong/workspace/intersection_CDG
cd $ROOT_PATH/release
module reset
module load yaml-cpp
module load SUMO/1.14.1-foss-2021b

## No more needed after success compilation ##
module load CMake/3.21.1-GCCcore-11.2.0
cmake -DCMAKE_BUILD_TYPE=RELEASE ..
make sumo_sim_batch

geometryIDList=(1 2)
arrivalIntervalList=(1 2 3)
N=(5 10 25 50 100)
sumoStepLengthList=(0.01 0.01 0.05 0.05 0.05)

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
            echo "./sumo_sim/sumo_sim_batch --num_nodes $numVehicle --geometryID ${geometryID} --arrival_interval_avg ${arrival_interval_avg} --sumo_step_length ${sumo_step_length} &"  # --test_count 1
            ./sumo_sim/sumo_sim_batch --num_nodes $numVehicle --geometryID ${geometryID} --arrival_interval_avg ${arrival_interval_avg} --sumo_step_length ${sumo_step_length} &  # --test_count 1
            sleep 2
        done
    done
done

## Wait for all of the background tasks to finish
wait
exit 0
