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
    ./jam.shape.cpp
    ./jam_compose_data.cpp
    ../jam.helper/attribute_args_helper.cpp
    ../jam.ilda_common/stb_truetype.h
    ../jam.ilda_common/nanosvg.h
    ../jam.ilda_common/ilda_data_record.cpp
    ../jam.ilda_common/ilda_file_processor.cpp
    ../jam.ilda_common/ilda_frame.cpp
    ../jam.ilda_common/ilda_header.cpp
    ../jam.ilda_common/ilda_colors.cpp
    ../jam.ilda_common/ttf_file_processor.cpp
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
