cmake_minimum_required(VERSION 3.16)
project(Crater_sharedShared C CXX)

set(FMT_MASTER_PROJECT OFF)
add_subdirectory(libs/expected-lite)
# add_subdirectory(libs/Catch2)
add_subdirectory(libs/fmt)

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


# add_executable(tests tests/test_circular_buffer.cpp)
# target_include_directories(tests PUBLIC ${INCLUDE_DIRS})
# target_link_libraries(tests PUBLIC ${LIBRARIES})

# target_link_libraries(tests PRIVATE Catch2::Catch2WithMain)

# set_target_properties(crater_shared PROPERTIES EXPORT_COMPILE_COMMANDS ON)
# set_target_properties(tests PROPERTIES EXPORT_COMPILE_COMMANDS ON)
