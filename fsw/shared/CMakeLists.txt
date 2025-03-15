cmake_minimum_required(VERSION 3.23)
project(Crater_sharedShared C CXX)
set (CMAKE_CXX_STANDARD 17)

add_subdirectory(libs/expected-lite EXCLUDE_FROM_ALL)
add_subdirectory(libs/span-lite EXCLUDE_FROM_ALL)
add_subdirectory(libs/fmt EXCLUDE_FROM_ALL)
add_subdirectory(../proto EXCLUDE_FROM_ALL proto)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/Warnings.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/LinkSystem.cmake)
target_compile_definitions(fmt-header-only INTERFACE -D_GLIBCXX_USE_WCHAR_T -DFMT_UNICODE=0 -DFMT_USE_LOCALE=false)

option(CRT_LIBRARY_ONLY "Build only the static library" OFF)

set(LIBRARIES
    nonstd::expected-lite
    nonstd::span-lite
    fmt::fmt-header-only
    crater_mavlink
)


set(SOURCES
    src/crater/core/errors/Error.cpp
    src/crater/core/io/ChannelToOutputStream.cpp
)

set(INCLUDE_DIRS
    inc/
)

#### Main Library ####
add_library(crater_shared STATIC ${SOURCES})
target_include_directories(crater_shared PUBLIC ${INCLUDE_DIRS})
target_link_libraries_system(crater_shared PUBLIC ${LIBRARIES})
target_compile_options(crater_shared PUBLIC "${CRATER_CXX_FLAGS}")


#### Interface Library ####
add_library(crater_shared_intf INTERFACE )
target_sources(crater_shared_intf INTERFACE ${SOURCES})
target_include_directories(crater_shared_intf INTERFACE ${INCLUDE_DIRS})
target_link_libraries_system(crater_shared_intf INTERFACE ${LIBRARIES})
target_compile_options(crater_shared_intf INTERFACE "${CRATER_CXX_FLAGS}")

#### UNIT TESTS ####
set(TEST_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/tests/test_circular_buffer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/tests/test_errors.cpp
    ${CMAKE_CURRENT_LIST_DIR}/tests/test_mutex.cpp
    ${CMAKE_CURRENT_LIST_DIR}/tests/test_channel.cpp
    ${CMAKE_CURRENT_LIST_DIR}/tests/test_mavlink.cpp
)

add_library(lest INTERFACE)
target_include_directories(lest INTERFACE libs/lest/include)
target_compile_definitions(lest INTERFACE -Dlest_FEATURE_AUTO_REGISTER=1)

if(NOT CRT_LIBRARY_ONLY)
    add_executable(crater_shared_tests tests/main.cpp ${TEST_SOURCES})

    target_include_directories(crater_shared_tests PUBLIC ${INCLUDE_DIRS})

    target_link_libraries(crater_shared_tests PUBLIC crater_shared_intf)
    target_link_libraries_system(crater_shared_tests PUBLIC ${LIBRARIES})
    target_link_libraries_system(crater_shared_tests PRIVATE lest)

    set_target_properties(crater_shared_tests PROPERTIES EXPORT_COMPILE_COMMANDS ON)

    add_test (NAME crater_shared_tests COMMAND crater_shared_tests)
else()
    set(CRATER_SHARED_TEST_SOURCES ${TEST_SOURCES} PARENT_SCOPE)
endif()

# set_target_properties(crater_shared PROPERTIES EXPORT_COMPILE_COMMANDS ON)