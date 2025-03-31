cmake_minimum_required(VERSION 3.16)
project(CraterMiosix C CXX)
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_C_STANDARD 17)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

include(MiosixTarget)
include(MiosixBoardConfig)
include(Warnings)
include(LinkSystem)

add_compile_definitions($<$<COMPILE_LANGUAGE:C,CXX>:MIOSIX_SKIP_SETTINGS_EDIT>)

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/miosix-kernel/miosix" EXCLUDE_FROM_ALL)

target_compile_definitions(miosix PUBLIC -D_GLIBCXX_USE_WCHAR_T)

add_library(rust_lib STATIC IMPORTED)
set_target_properties(rust_lib PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/../target/thumbv7em-none-eabihf/debug/libflight.a"
)

add_executable(rust_main entrypoints/rust_main.cpp)
target_link_libraries(rust_main PRIVATE rust_lib)
miosix_target(rust_main)
set_target_properties(rust_main PROPERTIES EXPORT_COMPILE_COMMANDS ON)
target_compile_options(rust_main PUBLIC -lc)
