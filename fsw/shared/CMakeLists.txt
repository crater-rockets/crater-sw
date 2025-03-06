cmake_minimum_required(VERSION 3.16)
project(Crater_sharedShared C CXX)
set (CMAKE_CXX_STANDARD 17)

add_subdirectory(libs/expected-lite EXCLUDE_FROM_ALL)
add_subdirectory(libs/fmt EXCLUDE_FROM_ALL)

set(SOURCES
    src/a.cpp
)

set(LIBRARIES
    nonstd::expected-lite
    fmt::fmt-header-only
)

set(INCLUDE_DIRS
    inc/
)

# FMT compile size optimizations
target_compile_definitions(fmt-header-only INTERFACE -D_GLIBCXX_USE_WCHAR_T -DFMT_UNICODE=0 -DFMT_STATIC_THOUSANDS_SEPARATOR='.')

add_library(crater_shared STATIC ${SOURCES} )
target_include_directories(crater_shared PUBLIC ${INCLUDE_DIRS})
target_link_libraries(crater_shared PUBLIC ${LIBRARIES})


set(TEST_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/tests/test_circular_buffer.cpp
    )

# Export test sources for builds targeting the embedded device

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    add_subdirectory(libs/Catch2 EXCLUDE_FROM_ALL)


    add_executable(crater_shared_tests ${TEST_SOURCES})
    target_include_directories(crater_shared_tests PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(crater_shared_tests PUBLIC ${LIBRARIES})
    target_link_libraries(crater_shared_tests PRIVATE Catch2::Catch2WithMain)
    include(CTest)
    include(libs/Catch2/extras/ParseAndAddCatchTests.cmake)
    ParseAndAddCatchTests(crater_shared_tests)

    set_target_properties(crater_shared_tests PROPERTIES EXPORT_COMPILE_COMMANDS ON)
else()
    set(CRATER_SHARED_TEST_SOURCES ${TEST_SOURCES} PARENT_SCOPE)
endif()

set_target_properties(crater_shared PROPERTIES EXPORT_COMPILE_COMMANDS ON)
