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
    ./jam.helios.device_manager.cpp
    ./helios-sdk/cpp/HeliosDac.cpp
    ./helios-sdk/cpp/shared_library/HeliosDacAPI.cpp
    ./libusb/sync.c
    ./helios-sdk/cpp/idn/idn.cpp
    ./libusb/io.c
    ./libusb/strerror.c
    ./libusb/descriptor.c
    ./libusb/os/darwin_usb.c
    ./libusb/hotplug.c
    ./helios-sdk/cpp/idn/idnServerList.cpp
    ./helios-sdk/cpp/idn/plt-posix.cpp
    ./libusb/os/threads_posix.c
    ./libusb/os/events_posix.c
    ./libusb/core.c
    
    
    
        
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
