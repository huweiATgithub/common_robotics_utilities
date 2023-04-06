# common_robotics_utilities
Common utility functions and algorithms for robotics work used by ARC &amp; ARM labs and TRI.

## Bindings
Binding codes copied from [mpetersen94/drake:gcs2](https://github.com/mpetersen94/drake/tree/gcs2) with:
- adaption to latest commit of the cpp library
- write tests in a more pythonic way
- more tests

### Installation
Requirements: 
- cmake, Eigen3
- (Pip) scikit-build, calver, numpy
```shell
pip install .
```

### Usage
The C++ library supports parallelism using OpenMP in finding nearest k neighborhoods, connecting roadmaps, etc.
However, cares must be taken when calling these codes.
In general, if python codes is being executed in the parallel region with parallelism activated: single thread performance or deadlock will be observed.
One should either:
- (best performance) write most planning problem related codes in C++, for an example, see [GraphPuzzle](bindings/common_robotics_utilities/common_robotics_utilities_extra_py.cpp).
- (avoid deadlock) turn-off parallelism by passing `use_parallel=False` in those function.


## C++ library
### Building

```shell
mkdir build && cd build
cmake .. 
cmake --build .
cmake --install .
```


### Testing

```shell
mkdir build && cd build
cmake .. 
ctest
```

```shell
pytest bindings/test
```
