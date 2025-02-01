cmake_minimum_required(VERSION 3.16)
project(Example C CXX)

add_library(crater-drivers
    test/test.cpp
)
target_include_directories(crater-drivers PUBLIC .)

# Link Miosix as dependency
add_subdirectory(../miosix-kernel/miosix miosix EXCLUDE_FROM_ALL)
include(LinkTarget)
miosix_link_target(crater-drivers)
