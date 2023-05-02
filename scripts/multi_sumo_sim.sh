echo "Current job id is: $SLURM_JOB_ID"

## change directory and load modules ##
ROOT_PATH=/home/dong/workspace/intersection_CDG
cd $ROOT_PATH/release
# module reset
# module load yaml-cpp/0.6.3-GCCcore-8.3.0

## No more needed after success compilation ##
cmake -DCMAKE_BUILD_TYPE=RELEASE ..
cmake -DCMAKE_BUILD_TYPE=RELEASE ..
make sumo_sim_batch

N=(5 10 15)

cd $ROOT_PATH/log/sumosim

for numVehicle in ${N[@]}; do
    if [[ ! -d N${numVehicle} ]]; then
        mkdir N${numVehicle}
    fi
done

cd $ROOT_PATH/release

for numVehicle in ${N[@]}; do
    echo "./sumo_sim/sumo_sim_batch --num_nodes $numVehicle --arrival_interval_avg 2 --geometryID 1"
done

## Wait for all of the background tasks to finish
wait
exit 0