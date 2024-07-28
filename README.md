## Safe and Efficient Unsignalized Intersection Management with Breadth-First Spanning Tree

### Requirements

- C++ 17
- CMake >= 3.14
- GoogleTest
- yaml-cpp

### Plot the results
- Change the `ROOT_PATH` in `Experiments/plot_result.py`
- Run `python Experiments/plot_result.py`

### Test locally
```
mkdir build
cd build 
cmake -DCMAKE_BUILD_TYPE=Release ..
make
./batch_test/batch_test_scheduler_v200
```