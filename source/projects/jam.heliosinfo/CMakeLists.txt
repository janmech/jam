# Copyright 2018 The Min-DevKit Authors. All rights reserved.
# Use of this source code is governed by the MIT License found in the License.md file.

cmake_minimum_required(VERSION 3.0)

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
    ../jam.helper/attribute_args_helper.cpp
    ../jam.helios.connector/jam.helios.connector.cpp
    ../jam.dmxusbpro.connector/helios-sdk/cpp/HeliosDac.cpp
    ../jam.dmxusbpro.connector/helios-sdk/cpp/shared_library/HeliosDacAPI.cpp
    ../jam.dmxusbpro.connector/libusb/sync.c
    ../jam.dmxusbpro.connector/helios-sdk/cpp/idn/idn.cpp
    ../jam.dmxusbpro.connector/libusb/io.c
    ../jam.dmxusbpro.connector/libusb/strerror.c
    ../jam.dmxusbpro.connector/libusb/descriptor.c
    ../jam.dmxusbpro.connector/libusb/os/darwin_usb.c
    ../jam.dmxusbpro.connector/libusb/hotplug.c
    ../jam.dmxusbpro.connector/helios-sdk/cpp/idn/idnServerList.cpp
    ../jam.dmxusbpro.connector/helios-sdk/cpp/idn/plt-posix.cpp
    ../jam.dmxusbpro.connector/libusb/os/threads_posix.c
    ../jam.dmxusbpro.connector/libusb/os/events_posix.c
    ../jam.dmxusbpro.connector/libusb/core.c
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
