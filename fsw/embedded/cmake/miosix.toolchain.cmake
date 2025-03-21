# Copyright (C) 2024 by Skyward
#
# This program is free software; you can redistribute it and/or
# it under the terms of the GNU General Public License as published
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# As a special exception, if other files instantiate templates or use
# macros or inline functions from this file, or you compile this file
# and link it with other works to produce a work based on this file,
# this file does not by itself cause the resulting work to be covered
# by the GNU General Public License. However the source code for this
# file must still be made available in accordance with the GNU
# Public License. This exception does not invalidate any other
# why a work based on this file might be covered by the GNU General
# Public License.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")

# Tell CMake that we are building for an embedded ARM system
set(CMAKE_SYSTEM_NAME Miosix)

# Select compiler
set(MIOSIX_PREFIX      arm-miosix-eabi)

# From compiler prefix form the name of the compiler and other tools
set(CMAKE_ASM_COMPILER ${MIOSIX_PREFIX}-as)
set(CMAKE_C_COMPILER   ${MIOSIX_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${MIOSIX_PREFIX}-g++)
set(CMAKE_AR           ${MIOSIX_PREFIX}-ar)
set(CMAKE_OBJCOPY      ${MIOSIX_PREFIX}-objcopy)
set(CMAKE_OBJDUMP      ${MIOSIX_PREFIX}-objdump)
set(CMAKE_SIZE         ${MIOSIX_PREFIX}-size)
set(MIOSIX_READELF     ${MIOSIX_PREFIX}-readelf)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)


