# CNMatrix [![.github/workflows/cmake.yml](https://github.com/cntools/cnmatrix/actions/workflows/cmake.yml/badge.svg)](https://github.com/cntools/cnmatrix/actions/workflows/cmake.yml)

This library provides a consistent C interface to a few matrix backends.

The interface itself is a little more sane than raw lapack / blas calls, and is meant to be reasonably performant for medium to large matrices. It should also be cross platform and work reasonably well on embedded low latency systems; as it consistently tries to avoid heap allocations.

As a caveat though; this library makes sense for C code bases, for C++ codebases it likely makes more sense to just use eigen directly.


