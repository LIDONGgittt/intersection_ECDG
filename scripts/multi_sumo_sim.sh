## change directory and load modules ##
ROOT_PATH=/home/dong/workspace/intersection_CDG
cd $ROOT_PATH/release
cmake -DCMAKE_BUILD_TYPE=RELEASE ..
make sumo_sim_batch

N=(5 10 100)
geometryIDList=(1 2)
arrivalIntervalList=(1 2 3)

cd $ROOT_PATH/log
mkdir sumosim
cd sumosim

for numVehicle in ${N[@]}; do
    if [[ ! -d N${numVehicle} ]]; then
        mkdir N${numVehicle}
    fi
done

cd $ROOT_PATH/release

for numVehicle in ${N[@]}; do
    for geometryID in ${geometryIDList[@]}; do
        for arrival_interval_avg in ${arrivalIntervalList[@]}; do
            echo "./sumo_sim/sumo_sim_batch --num_nodes $numVehicle --geometryID ${geometryID} --arrival_interval_avg ${arrival_interval_avg} --test_count 50 &"  # --test_count 1
            ./sumo_sim/sumo_sim_batch --num_nodes $numVehicle --geometryID ${geometryID} --arrival_interval_avg ${arrival_interval_avg} --test_count 50 &  # --test_count 1
            sleep 2
        done
    done
    wait
done

## Wait for all of the background tasks to finish
wait
exit 0