cmake_minimum_required(VERSION 3.16)
project(CraterMiosix C CXX)
set (CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

include(MiosixTarget)
include(MiosixBoardConfig)

add_compile_definitions($<$<COMPILE_LANGUAGE:C,CXX>:MIOSIX_SKIP_SETTINGS_EDIT>)

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/miosix-kernel/miosix" EXCLUDE_FROM_ALL)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../shared" crater_shared EXCLUDE_FROM_ALL)

# This variable is defined inside the miosix makefile, but it's not avaialble in the scope.
# I'm too lazy to implement a proper solution so I just redefined it here
# set(MIOSIX_CPU_FLAGS -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16)
# target_compile_options(crater_shared PUBLIC ${MIOSIX_CPU_FLAGS})

set(EMBEDDED_SOURCES
)
target_compile_definitions(miosix PUBLIC -D_GLIBCXX_USE_WCHAR_T)
target_compile_definitions(fmt-header-only INTERFACE -D_GLIBCXX_USE_WCHAR_T)

add_executable(hi_mom 
                entrypoints/hi_mom/main.cpp
                ${EMBEDDED_SOURCES}
            )
target_include_directories(hi_mom PRIVATE inc/)
target_link_libraries(hi_mom PRIVATE crater_shared)
miosix_target(hi_mom)
