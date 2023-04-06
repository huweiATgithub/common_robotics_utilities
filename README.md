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

## Building

```shell
mkdir build && cd build
cmake .. 
cmake --build .
cmake --install .
```


## Testing

```shell
mkdir build && cd build
cmake .. 
ctest
```

```shell
pytest bindings/test
```
