function (miosix_target TARGET)
    target_link_libraries_system(${TARGET} PUBLIC miosix)
    target_link_libraries(${TARGET} PUBLIC stdc++ c m gcc atomic)
    # Create .map file
    target_link_options(${TARGET} PRIVATE -Wl,-Map,$<TARGET_FILE_DIR:${TARGET}>/$<TARGET_FILE_BASE_NAME:${TARGET}>.map)

    # Create .bin file
    add_custom_command(
            TARGET ${TARGET} POST_BUILD
            COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET}> ${TARGET}.bin
            COMMENT "Creating ${TARGET}.bin"
            VERBATIM
        )
endfunction()