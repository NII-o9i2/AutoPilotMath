## common_math
This project mainly conducts some common module tests of planning in automatic driving systems.

### Ready:
**1.prepare bench dependency**

```shell
# Check out the library.
$ git submodule update --init --recursive
# Go to the library root directory
$ cd benchmark
# Make a build directory to place the build output.
$ cmake -E make_directory "build"
# Generate build system files with cmake, and download any dependencies.
$ cmake -E chdir "build" cmake -DBENCHMARK_DOWNLOAD_DEPENDENCIES=on -DCMAKE_BUILD_TYPE=Release ../
# or, starting with CMake 3.13, use a simpler form:
# cmake -DCMAKE_BUILD_TYPE=Release -S . -B "build"
# Build the library & install.
$ cmake --build "build" --config Release --target install
```
在/benchmark/CMakeLists.txt中修改一下:

    option(BENCHMARK_DOWNLOAD_DEPENDENCIES "Allow the downloading and in-tree building of unmet dependencies" ON)
    #option(BENCHMARK_ENABLE_GTEST_TESTS "Enable building the unit tests which depend on gtest" ON)
    #option(BENCHMARK_USE_BUNDLED_GTEST "Use bundled GoogleTest. If disabled, the find_package(GTest) will be used." ON)

Refs: https://github.com/google/benchmark

**2.prepare eigen dependency**
    在eigen文件夹中安装

```shell
    mkdir build
    cd build 
    cmake ..
    make install
```

**3.prepare osqp dependency**

Create build directory and change directory
```shell
cd osqp
mkdir build
cd build
#Create Makefiles

#In Linux and Mac OS run

cmake -G "Unix Makefiles" ..
#In Windows run

cmake -G "MinGW Makefiles" ..
#Compile OSQP

cmake --build .

# Install 

cmake --build . --target install
```

refs: https://osqp.org/docs/get_started/sources.html#build-the-binaries


**4.prepare pybind dependency**

```shell
pip install "pybind11[global]"
```
macOS, you can use
```
brew install pybind11
```

### PolyFit module
We use Eigen & OSQP to achieve polyfit func, and use pybind11 & jupyter and benchmark to explore the performance.
### njson Module
refs:https://github.com/nlohmann/json