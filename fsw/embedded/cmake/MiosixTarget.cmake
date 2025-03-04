function (miosix_target TARGET)
    target_link_libraries(${TARGET} PUBLIC
        -Wl,--start-group miosix stdc++ c m gcc atomic -Wl,--end-group
    )

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