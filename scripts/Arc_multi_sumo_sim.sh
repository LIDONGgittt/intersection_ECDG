#!/bin/bash
#SBATCH -J SUMO_SIM
#SBATCH --account=ev_charging
#SBATCH --partition=normal_q
#SBATCH --nodes=1 --ntasks-per-node=40 --cpus-per-task=1
#SBATCH --time=0-08:00:00 # 6 hours

#SBATCH --mail-user=dongli@vt.edu
#SBATCH --mail-type=BEGIN  # send email when job begins
#SBATCH --mail-type=END    # send email when job ends
#SBATCH --mail-type=FAIL   # send email when job aborts

echo "Current job id is: $SLURM_JOB_ID"

## change directory and load modules ##
ROOT_PATH=/home/dong/workspace/intersection_CDG
cd $ROOT_PATH/release
module reset
module load module load SUMO/1.14.1-foss-2021b

## No more needed after success compilation ##
cmake -DCMAKE_BUILD_TYPE=RELEASE ..
make sumo_sim_batch

N=(5 10 20 50 100)
geometryIDList=(1 2)
arrivalIntervalList=(1 2 3 4)

cd $ROOT_PATH/log/sumosim

for numVehicle in ${N[@]}; do
    if [[ ! -d N${numVehicle} ]]; then
        mkdir N${numVehicle}
    fi
done

cd $ROOT_PATH/release

for numVehicle in ${N[@]}; do
    for geometryID in ${geometryIDList[@]}; do
        for arrival_interval_avg in ${arrivalIntervalList[@]}; do
            echo "./sumo_sim/sumo_sim_batch --num_nodes $numVehicle --geometryID ${geometryID} --arrival_interval_avg ${arrival_interval_avg} &"
            ./sumo_sim/sumo_sim_batch --num_nodes $numVehicle --geometryID ${geometryID} --arrival_interval_avg ${arrival_interval_avg} &
            sleep 2
        done
    done
done

## Wait for all of the background tasks to finish
wait
exit 0