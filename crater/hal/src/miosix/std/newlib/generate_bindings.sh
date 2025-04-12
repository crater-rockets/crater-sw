#!/bin/sh
bindgen miosix_newlib.h -o miosix_newlib.rs --use-core  \
        -- --target=thumbv7em-none-eabihf \
        -I /opt/arm-miosix-eabi/arm-miosix-eabi/include -D_POSIX_MONOTONIC_CLOCK
