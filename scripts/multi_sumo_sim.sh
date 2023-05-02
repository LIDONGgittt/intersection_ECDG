echo "Current job id is: $SLURM_JOB_ID"

## change directory and load modules ##
ROOT_PATH=/home/dong/workspace/intersection_CDG
cd $ROOT_PATH/release
module reset
module load module load SUMO/1.14.1-foss-2021b

## No more needed after success compilation ##
cmake -DCMAKE_BUILD_TYPE=RELEASE ..
make sumo_sim_batch

N=(5 8)

cd $ROOT_PATH/log/sumosim

for numVehicle in ${N[@]}; do
    if [[ ! -d N${numVehicle} ]]; then
        mkdir N${numVehicle}
    fi
done

cd $ROOT_PATH/release

for numVehicle in ${N[@]}; do
    echo "./sumo_sim/sumo_sim_batch --num_nodes $numVehicle --arrival_interval_avg 2 --geometryID 1 --test_count 2 &"
    ./sumo_sim/sumo_sim_batch --num_nodes $numVehicle --arrival_interval_avg 2 --geometryID 1 --test_count 2 &
    sleep 1

    echo "./sumo_sim/sumo_sim_batch --num_nodes $numVehicle --arrival_interval_avg 1 --geometryID 1 --test_count 2 &"
    ./sumo_sim/sumo_sim_batch --num_nodes $numVehicle --arrival_interval_avg 1 --geometryID 1 --test_count 2 &
done

## Wait for all of the background tasks to finish
wait
exit 0