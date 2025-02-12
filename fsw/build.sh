# $1 = path to the CMakeLists.txt directory
# $2 = name of the target to build

rm -rf $1/build
cmake -S $1 -B $1/build \
    -DCMAKE_C_COMPILER_LAUNCHER=ccache \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
    --toolchain $(pwd)/$(dirname $0)/miosix-kernel/miosix/cmake/Toolchains/gcc.cmake
cmake --build $1/build -j 16 --target $2
