cmake_minimum_required (VERSION 3.15)

project (qemu-nrf52 LANGUAGES C CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

MACRO(HEADER_DIRECTORIES return_list)
    FILE(GLOB_RECURSE new_list "${CMAKE_CURRENT_SOURCE_DIR}/hw/*.h")
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        SET(dir_list ${dir_list} ${dir_path})
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
    SET(${return_list} ${dir_list})
ENDMACRO()

HEADER_DIRECTORIES(header_dir_list)
list(LENGTH header_dir_list header_dir_list_count)
message(STATUS "[INFO] Found ${header_dir_list_count} header directories.")

#file(GLOB_RECURSE SRCS "${CMAKE_CURRENT_SOURCE_DIR}/arm/*.c")
#list(LENGTH SRCS SRCS_count)
#message(STATUS "[INFO] Found ${SRCS_count} source files.")

#add_definitions(-Wno-attributes -Wsign-compare -Wenum-compare -Werror)
add_definitions(-Werror=incompatible-pointer-types -Werror=unused-variable)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/hw/arm SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/hw/core SRCS)
list(LENGTH SRCS SRCS_count)
message(STATUS "[INFO] Found ${SRCS_count} source files.")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/build)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/build/arm/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/tcg/arm/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/build/arm-softmmu/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/build/hw/core)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/target/arm)
include_directories(${header_dir_list})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/hw)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/hw/arm/)

include_directories(/usr/include/glib-2.0)
include_directories(/usr/lib/x86_64-linux-gnu/glib-2.0/include)
include_directories(/usr/include/pixman-1)

add_definitions(-DNEED_CPU_H)
add_definitions(-DCONFIG_TARGET=\"arm-softmmu-config-target.h\")
add_definitions(-DCONFIG_DEVICES=\"arm-softmmu-config-devices.h\")

add_library(${PROJECT_NAME}
        ${CMAKE_CURRENT_SOURCE_DIR}/hw/dma/nrf52_edma.c
        ${CMAKE_CURRENT_SOURCE_DIR}/hw/timer/nrf_rtc.c
        ${CMAKE_CURRENT_SOURCE_DIR}/hw/gpio/nrf52_gpiote.c
        ${CMAKE_CURRENT_SOURCE_DIR}/hw/sensor/lsm6dsox.c
        ${SRCS})

add_custom_target(qemu_arm COMMAND make -C ${CMAKE_CURRENT_SOURCE_DIR}/build)

#install(TARGETS qemu_arm
#        #CONFIGURATIONS Release
#        DESTINATION ${CMAKE_INSTALL_BINDIR}
#        )
