cmake_minimum_required(VERSION 3.23)
project(Crater_Mavlink C CXX)
set (CMAKE_CXX_STANDARD 17)


### MAVLINK FILE GENERATION ###
set(MAVLINK_DIALECT_FILE ${CMAKE_CURRENT_LIST_DIR}/crater.xml)
set(MAVLINK_GEN_DIR ${CMAKE_CURRENT_LIST_DIR}/generated/mavlink)
set(MAVLINK_GEN_FILES
    ${MAVLINK_GEN_DIR}/mavlink_get_info.h
    ${MAVLINK_GEN_DIR}/message.hpp
    ${MAVLINK_GEN_DIR}/mavlink_types.h
    ${MAVLINK_GEN_DIR}/mavlink_helpers.h
    ${MAVLINK_GEN_DIR}/mavlink_sha256.h
    ${MAVLINK_GEN_DIR}/mavlink_conversions.h
    ${MAVLINK_GEN_DIR}/protocol.h
    ${MAVLINK_GEN_DIR}/msgmap.hpp
    ${MAVLINK_GEN_DIR}/checksum.h
    ${MAVLINK_GEN_DIR}/crater/mavlink_msg_testmessage.h
    ${MAVLINK_GEN_DIR}/crater/gtestsuite.hpp
    ${MAVLINK_GEN_DIR}/crater/crater.hpp
    ${MAVLINK_GEN_DIR}/crater/testsuite.h
    ${MAVLINK_GEN_DIR}/crater/crater.h
    ${MAVLINK_GEN_DIR}/crater/mavlink_msg_testmessage.hpp
    ${MAVLINK_GEN_DIR}/crater/mavlink.h
    ${MAVLINK_GEN_DIR}/crater/version.h
    )

add_custom_command(OUTPUT ${MAVLINK_GEN_FILES}
                    COMMAND mavgen.py --lang C++11 --wire-protocol 2.0 ${MAVLINK_DIALECT_FILE} -o ${MAVLINK_GEN_DIR}
                    DEPENDS ${MAVLINK_DIALECT_FILE}
                    )
add_custom_target(mavlink_generated DEPENDS ${MAVLINK_GEN_FILES})

### CUSTOM CODEGENERATION ###
set(CRTEXT_TEMPLATE_DIR ${CMAKE_CURRENT_LIST_DIR}/templates)
set(CRTEXT_GEN_DIR ${CMAKE_CURRENT_LIST_DIR}/generated/crater_ext)

set(CRTEXT_GEN_FILES
        ${CRTEXT_GEN_DIR}/ErrorCode.hpp
        ${CRTEXT_GEN_DIR}/ErrorCode.cpp
)

set(CRTEXT_TEMPLATE_FILES
        ${CRTEXT_TEMPLATE_DIR}/ErrorCode.hpp.jinja
        ${CRTEXT_TEMPLATE_DIR}/ErrorCode.cpp.jinja
)

add_custom_command(OUTPUT ${CRTEXT_GEN_FILES}
                    COMMAND python3 ${CMAKE_CURRENT_LIST_DIR}/cratergen.py ${MAVLINK_DIALECT_FILE} -t ${CRTEXT_TEMPLATE_DIR} -o ${CRTEXT_GEN_DIR}
                    DEPENDS ${MAVLINK_DIALECT_FILE} ${CRTEXT_TEMPLATE_FILES}
                    )
add_custom_target(crtext_generated DEPENDS ${CRTEXT_GEN_FILES})

add_library(crater_mavlink STATIC generated/crater_ext/ErrorCode.cpp)

target_include_directories(crater_mavlink PUBLIC generated)
target_compile_definitions(crater_mavlink PUBLIC MAVLINK_NO_CONVERSION_HELPERS)


# Tell cmake that our library depends on codegenerated files
add_dependencies(crater_mavlink mavlink_generated)
add_dependencies(crater_mavlink crtext_generated)