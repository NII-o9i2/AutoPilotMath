# CiLQR Solver
A non-linear trajectory optimization library developed by sensetime, Inc. This library implements a C++ version of the original open-source Cilqr solver.

## For details on the algorithm, see the related papers:
https://bjack205.github.io/papers/AL_iLQR_Tutorial.pdf

## Dependencies
senseauto-buildtools
senseauto-3rdparty
    protobuf
    eigen

## Build
```
make tgz
```

## Unittest Running
```
make test