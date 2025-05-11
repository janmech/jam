# Copyright 2018 The Min-DevKit Authors. All rights reserved.
# Use of this source code is governed by the MIT License found in the License.md file.

cmake_minimum_required(VERSION 3.0)

#############################################################
# Build jam.ilda.manager as extension (not as external)
#############################################################
set(C74_BUILD_MAX_EXTENSION 1)

set(C74_MIN_API_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../min-api)
include(${C74_MIN_API_DIR}/script/min-pretarget.cmake)


#############################################################
# MAX EXTERNAL
#############################################################


include_directories( 
	"${C74_INCLUDES}"
)


set( SOURCE_FILES
	${PROJECT_NAME}.cpp
    ../jam.helios.connector/libusb/sync.c
    ../jam.helios.connector/libusb/io.c
    ../jam.helios.connector/libusb/strerror.c
    ../jam.helios.connector/libusb/descriptor.c
    ../jam.helios.connector/libusb/os/darwin_usb.c
    ../jam.helios.connector/libusb/hotplug.c
    ../jam.helios.connector/helios-sdk/cpp/idn/idn.cpp
    ../jam.helios.connector/helios-sdk/cpp/HeliosDac.cpp
    ../jam.helios.connector/helios-sdk/cpp/shared_library/HeliosDacAPI.cpp
    ../jam.helios.connector/libusb/os/threads_posix.c
    ../jam.helios.connector/libusb/os/events_posix.c
    ../jam.helios.connector/libusb/core.c
    ../jam.helios.connector/helios-sdk/cpp/idn/idnServerList.cpp
    ../jam.helios.connector/helios-sdk/cpp/idn/plt-posix.cpp
    ../jam.helios.connector/jam.helios.connector.cpp
)


add_library( 
	${PROJECT_NAME} 
	MODULE
	${SOURCE_FILES}
)


include(${C74_MIN_API_DIR}/script/min-posttarget.cmake)


#############################################################
# UNIT TEST
#############################################################

include(${C74_MIN_API_DIR}/test/min-object-unittest.cmake)
