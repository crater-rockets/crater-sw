cmake_minimum_required(VERSION 3.16)
project(CraterMiosix C CXX)
set (CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../shared/cmake")

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


### UNIT TESTS ####
set(TEST_SOURCES )

add_executable(embedded_tests tests/tests_main.cpp ${TEST_SOURCES} ${CRATER_SHARED_TEST_SOURCES})
target_include_directories(embedded_tests PUBLIC ${INCLUDE_DIRS})
target_link_libraries(embedded_tests PUBLIC ${LIBRARIES})
target_link_libraries(embedded_tests PRIVATE lest)
set_target_properties(embedded_tests PROPERTIES EXPORT_COMPILE_COMMANDS ON)
target_link_libraries(embedded_tests PRIVATE crater_shared)
miosix_target(embedded_tests)

# Workaround to let mavlink-C++11 compile properly
target_include_directories(crater_mavlink PUBLIC ${CMAKE_CURRENT_LIST_DIR}/bsps/include)
