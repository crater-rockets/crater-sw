cmake_minimum_required(VERSION 3.16)
project(Example C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Board selection
add_compile_definitions($<$<COMPILE_LANGUAGE:C,CXX>:MIOSIX_SKIP_SETTINGS_EDIT>)
set(MIOSIX_OPT_BOARD stm32f429zi_stm32f4discovery CACHE STRING "Target board")

# Include dependencies
add_subdirectory(../../../../drivers crater-drivers EXCLUDE_FROM_ALL)

add_executable(main main.cpp)
target_link_libraries(main PUBLIC crater-drivers)
miosix_add_program_target(main)
