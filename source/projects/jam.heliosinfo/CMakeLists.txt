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
	../jam.helios.connector/helios-sdk/cpp/HeliosDac.cpp
	../jam.helios.connector/helios-sdk/cpp/idn/idn.cpp
    ../jam.helios.connector/helios-sdk/cpp/shared_library/HeliosDacAPI.cpp
    ../jam.helios.connector/helios-sdk/cpp/idn/idnServerList.cpp
    ../jam.helios.connector/helios-sdk/cpp/idn/plt-posix.c
    ../jam.helper/attribute_args_helper.cpp
    ../jam.helios.connector/jam.helios.connector.cpp
)


add_library( 
	${PROJECT_NAME} 
	MODULE
	${SOURCE_FILES}
	
)

# 1. Locate the dylib using the correct relative path
find_library(LIBUSB_BIN 
    NAMES libusb-1.0.0.dylib 
    PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../jam.helios.connector/helios-sdk/cpp/libusb_bin/macOS"
    NO_DEFAULT_PATH
)

# 2. Check if it was actually found to prevent the "NOTFOUND" error
if(NOT LIBUSB_BIN)
    message(FATAL_ERROR "Could not find libusb-1.0.0.dylib! Checked in: ${CMAKE_CURRENT_SOURCE_DIR}/../jam.helios.connector/helios-sdk/cpp/libusb_bin/macOS")
endif()

# 3. Link it
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBUSB_BIN})

# 4. Bundle it
set_source_files_properties(${LIBUSB_BIN} PROPERTIES 
    MACOSX_PACKAGE_LOCATION Frameworks
)
target_sources(${PROJECT_NAME} PRIVATE ${LIBUSB_BIN})

# 5. Set RPATH
set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "@loader_path/../Frameworks"
)


include(${C74_MIN_API_DIR}/script/min-posttarget.cmake)


#############################################################
# UNIT TEST
#############################################################

include(${C74_MIN_API_DIR}/test/min-object-unittest.cmake)
