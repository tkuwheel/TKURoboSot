#
# This file is part of libfreespace.
# Copyright (c) 2009-2013 Hillcrest Laboratories, Inc.
# libfreespace is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
#
#

cmake_minimum_required (VERSION 2.6)
project(libfreespace CXX C)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${PROJECT_SOURCE_DIR}/CMake")

set(PROJECT_VERSION_MAJOR 0)
set(PROJECT_VERSION_MINOR 7)
set(PROJECT_VERSION_PATCH 0)

set(PROJECT_VERSION_STRING "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}")

include(FindPythonInterp)
if (NOT DEFINED PYTHONINTERP_FOUND)
	message(FATAL_ERROR "Could not locate python interpreter, python is required to generate freespace messages)")
endif()

if (WIN32)
    set(LIBFREESPACE_LIB_TYPE_DEFAULT "STATIC")
else()
    set(LIBFREESPACE_LIB_TYPE_DEFAULT "SHARED")
endif()

### Project Configuration Options
set(LIBFREESPACE_ADDITIONAL_MESSAGE_FILE "" CACHE FILEPATH "An additional HID message definition file")
set(LIBFREESPACE_BACKEND "" CACHE STRING "Specify an alternate backend on some paltforms. On Linux, valid values are 'hidraw' and 'libusb'")
set(LIBFREESPACE_CODECS_ONLY OFF CACHE BOOL "Build only the libfreespace codecs")
set(LIBFREESPACE_CUSTOM_INSTALL_RULES "" CACHE FILEPATH "CMake file to customize install rules when libfreespace is built as part of a larger project")
set(LIBFREESPACE_HIDRAW_THREADED_WRITES OFF CACHE BOOL "Enable writes in a backend thread when using hidraw")
set(LIBFREESPACE_LIB_TYPE "${LIBFREESPACE_LIB_TYPE_DEFAULT}" CACHE STRING "The type of library to create, set to SHARED or STATIC")

set(LIBFREESPACE_CODEC_SRCS
    "${PROJECT_BINARY_DIR}/gen_src/freespace_codecs.c"
    "${PROJECT_BINARY_DIR}/gen_src/freespace_printers.c"
)

set(LIBFREESPACE_CODEC_HDRS
    "${PROJECT_BINARY_DIR}/include/freespace/freespace_codecs.h"
    "${PROJECT_BINARY_DIR}/include/freespace/freespace_printers.h"
)

### Message Code Generator #######################

# Build rule to generate the HCOMM messages as needed.
add_custom_command(
    OUTPUT ${LIBFREESPACE_CODEC_SRCS} ${LIBFREESPACE_CODEC_HDRS}
    COMMAND
        ${PYTHON_EXECUTABLE}
        "${PROJECT_SOURCE_DIR}/common/messageCodeGenerator.py"
        "-I" "${PROJECT_BINARY_DIR}/include/"
        "-s" "${PROJECT_BINARY_DIR}/gen_src/"
        "${PROJECT_SOURCE_DIR}/common/setupMessages.py"
        "${LIBFREESPACE_ADDITIONAL_MESSAGE_FILE}"
    ${buildMessageCommand}
    DEPENDS ${PROJECT_BINARY_DIR}/include
            ${PROJECT_SOURCE_DIR}/common/messageCodeGenerator.py
            ${PROJECT_SOURCE_DIR}/common/setupMessages.py
            ${LIBFREESPACE_ADDITIONAL_MESSAGE_FILE}
    COMMENT "Generating libfreespace message code"
)

# Determine the target endianness and set the libfreespace flag.
if(ANDROID)
    #TEST_BIG_ENDIAN doesn't work with Android NDK
    add_definitions( -DFREESPACE_LITTLE_ENDIAN )
    SET( CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -lm" )
else()
    include(TestBigEndian NO_POLICY_SCOPE)
    TEST_BIG_ENDIAN(isBigEndian)
    if (isBigEndian)
        add_definitions( -DFREESPACE_BIG_ENDIAN )
    else()
        add_definitions( -DFREESPACE_LITTLE_ENDIAN )
    endif(isBigEndian)
endif()

# List the common source files
set (LIBFREESPACE_COMMON_SRCS
    "common/freespace_deviceTable.c"
    "common/freespace_util.c"
    "${LIBFREESPACE_CODEC_SRCS}"
)

#message(STATUS "LIBFREESPACE_ADDITIONAL_MESSAGE_FILE = ${LIBFREESPACE_ADDITIONAL_MESSAGE_FILE}")
#message(STATUS "LIBFREESPACE_CODECS_ONLY             = ${LIBFREESPACE_CODECS_ONLY}")
#message(STATUS "LIBFREESPACE_LIB_TYPE                = ${LIBFREESPACE_LIB_TYPE}")
#message(STATUS "LIBFREESPACE_BACKEND                 = ${LIBFREESPACE_BACKEND}")
#message(STATUS "LIBFREESPACE_HIDRAW_THREADED_WRITES  = ${LIBFREESPACE_HIDRAW_THREADED_WRITES}")
#message(STATUS "LIBFREESPACE_CUSTOM_INSTALL_RULES    = ${LIBFREESPACE_CUSTOM_INSTALL_RULES}")

configure_file(${PROJECT_SOURCE_DIR}/freespace_config.h.in ${PROJECT_BINARY_DIR}/include/freespace_config.h)

set(_LIBFREESPACE_LIBRARIES "")

if (LIBFREESPACE_CODECS_ONLY)
    list(APPEND _LIBFREESPACE_LIBRARIES freespace-codecs)
    # BUILD WITHOUT HID DEPENDENCIES
    message(STATUS "Building libfreespace without HID support")
    add_library(freespace-codecs
        ${LIBFREESPACE_LIB_TYPE}
        ${LIBFREESPACE_CODEC_SRCS}
    )
else()
    list(APPEND _LIBFREESPACE_LIBRARIES freespace)
    if (WIN32)
        # WINDOWS BUILD CONFIGURATION

        find_package(WinDDK REQUIRED)

        # If using Visual Studio 9 (2008) then we need a couple special includes to make things work
        IF(MSVC90)
        	find_package(VC9Include REQUIRED)

        	include_directories(${VC9_INCLUDE})
        	include_directories(${WINDDK_INCLUDE_DIR}/api)
        ENDIF()

        # If using Visual Studio 8 (2005) then we need a couple special includes to make things work
        IF(MSVC80)
            find_package(VC8Include REQUIRED)

            include_directories(${VC8_INCLUDE})
            include_directories(${WINDDK_INCLUDE_DIR}/api)
        ENDIF()

        include_directories(${WINDDK_INCLUDE_DIR})

        add_definitions(-DUNICODE -D_UNICODE)

        add_library(freespace ${LIBFREESPACE_LIB_TYPE}
            ${LIBFREESPACE_COMMON_SRCS}
            "win32/freespace_device.c"
            "win32/freespace_deviceMgr.c"
            "win32/freespace_discovery.c"
            "win32/freespace_discoveryDetail.c"
        )

        set_target_properties(freespace PROPERTIES PREFIX "lib")


	    IF (CMAKE_SIZEOF_VOID_P EQUAL 8)   # Means we are using 64-bit
	        set(_hid ${WINDDK_WIN7_X64_HID_LIB})
            set(_setupapi ${WINDDK_WIN7_X64_SETUPAPI_LIB})
	    ELSE()
	        IF (${CMAKE_SYSTEM_VERSION} EQUAL 5.1)  # Means we are running XP
	            set(_hid ${WINDDK_WINXP_X86_HID_LIB})
                set(_setupapi ${WINDDK_WINXP_X86SETUPAPID_LIB})
	        ELSE()
	            set(_hid ${WINDDK_WIN7_X86_HID_LIB})
                set(_setupapi ${WINDDK_WIN7_X86_SETUPAPI_LIB})
	        ENDIF()
	    ENDIF()
        
        if (NOT _hid)
            message(FATAL_ERROR "Could not locate hid.lib")
        endif()
        
        if (NOT _setupapi)
            # Not found in WinDDK, assume it's in default search path
            set(_setupapi "setupapi.lib")
        endif()

        target_link_libraries(freespace ${_hid})
        target_link_libraries(freespace ${_setupapi})

    elseif(UNIX)
        # Additional headers
        include(CheckIncludeFiles)
        check_include_files(sys/time.h HAVE_SYS_TIME_H)
        if (NOT HAVE_SYS_TIME_H)
            message(FATAL_ERROR "Could not find include file <sys/time.h>")
        endif()
        if (LIBFREESPACE_BACKEND STREQUAL "hidraw")
            check_include_files(linux/hidraw.h HAVE_LINUX_HIDRAW_H)
            if (NOT HAVE_LINUX_HIDRAW_H)
                message(FATAL_ERROR "Could not find include file <linux/hidraw.h>")
            endif()

            if (LIBFREESPACE_HIDRAW_THREADED_WRITES)
                add_definitions(-DLIBFRESPACE_THREADED_WRITES -pthread)
                list(APPEND CMAKE_EXE_LINKER_FLAGS -pthread)
            endif()
            add_library(freespace ${LIBFREESPACE_LIB_TYPE}
                ${LIBFREESPACE_COMMON_SRCS}
                "linux/freespace_hidraw.c"
                "linux/linux_hotplug.c"
             )

        elseif (LIBFREESPACE_BACKEND STREQUAL "libusb" OR LIBFREESPACE_BACKEND STREQUAL "")
            #set(libusb_1_FIND_QUIETLY ON)
            set(LIBUSB1_FIND_REQUIRED ON)
            find_package(libusb-1.0)

            list(APPEND CMAKE_EXE_LINKER_FLAGS -pthread)
            add_definitions(-pthread)

            include_directories(${LIBUSB_1_INCLUDE_DIRS})

            add_library(freespace ${LIBFREESPACE_LIB_TYPE}
                ${LIBFREESPACE_COMMON_SRCS}
                "linux/freespace.c"
                "linux/linux_hotplug.c"
             )

            target_link_libraries(freespace ${LIBUSB_1_LIBRARIES})
        else()
            message(FATAL_ERROR "Unsupported backened -- ${LIBFREESPACE_BACKEND}")
        endif()
    elseif(APPLE)
        # Mac OSX / Darwing build configuration
        add_library(freespace ${LIBFREESPACE_LIB_TYPE}
            ${LIBFREESPACE_COMMON}
            "linux/freespace.c"
            "linux/darwin_hotplug.c"
        )
    else()
        message(FATAL_ERROR "Unsupported platform")
    endif()
endif()

## These includes are down here because the platform-specific includes must be added first.
include_directories("include")
include_directories("${PROJECT_BINARY_DIR}/include")

### Docs
add_subdirectory(doc)

### Install rules
if (NOT LIBFREESPACE_CUSTOM_INSTALL_RULES)
    if (NOT LIBFREESPACE_CODECS_ONLY)
        install(TARGETS freespace LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
        set_target_properties(freespace PROPERTIES
            VERSION ${PROJECT_VERSION_STRING}
            SOVERSION ${PROJECT_VERSION_MAJOR} )
    endif()
    install(DIRECTORY include/freespace DESTINATION include)
    install(FILES ${LIBFREESPACE_CODEC_HDRS} DESTINATION include/freespace)
else()
    include(${LIBFREESPACE_CUSTOM_INSTALL_RULES})
endif()

if (WIN32)
    SET(CPACK_GENERATOR ZIP)
else()
    SET(CPACK_GENERATOR TGZ)
endif()

### Package rules
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "CMake is a build tool")
SET(CPACK_PACKAGE_INSTALL_DIRECTORY "libfreespace")
SET(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "libfreespace")
SET(CPACK_PACKAGE_NAME "libfreespace")
SET(CPACK_PACKAGE_VENDOR "Hillcrest Labs, Inc.")
SET(CPACK_PACKAGE_VERSION "${PROJECT_VERSION_STRING}")
SET(CPACK_PACKAGE_VERSION_MAJOR "${PROJECT_VERSION_MAJOR}")
SET(CPACK_PACKAGE_VERSION_MINOR "${PROJECT_VERSION_MINOR}")
SET(CPACK_PACKAGE_VERSION_PATCH "${PROJECT_VERSION_PATCH}")
SET(CPACK_SOURCE_GENERATOR ZIP)
#SET(CPACK_SOURCE_STRIP_FILES "")
SET(CPACK_STRIP_FILES "libfreespace")

#SET(CPACK_NSIS_DISPLAY_NAME "libfreespace ${PROJECT_VERSIONSTR}")
#SET(CPACK_INSTALL_CMAKE_PROJECTS "/home/andy/vtk/CMake-bin;CMake;ALL;/")
#SET(CPACK_OUTPUT_CONFIG_FILE "/home/andy/vtk/CMake-bin/CPackConfig.cmake")
#SET(CPACK_PACKAGE_DESCRIPTION_FILE "README")

#SET(CPACK_RESOURCE_FILE_LICENSE "/home/andy/vtk/CMake/Copyright.txt")
#SET(CPACK_RESOURCE_FILE_README "/home/andy/vtk/CMake/Templates/CPack.GenericDescription.txt")
#SET(CPACK_RESOURCE_FILE_WELCOME "/home/andy/vtk/CMake/Templates/CPack.GenericWelcome.txt")

#SET(CPACK_SOURCE_OUTPUT_CONFIG_FILE "/home/andy/vtk/CMake-bin/CPackSourceConfig.cmake")

#SET(CPACK_SYSTEM_NAME "Linux-i686")
#SET(CPACK_TOPLEVEL_TAG "Linux-i686")

include(CPack)

#
# Exports
#
set(LIBFREESPACE_LIBRARIES ${_LIBFREESPACE_LIBRARIES}
    CACHE "STRING" "List of libraries to include for libfreespace support" FORCE)
set(LIBFREESPACE_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include ${PROJECT_BINARY_DIR}/include
    CACHE "STRING" "List of include directories for libfreespace support" FORCE
)
mark_as_advanced(LIBFREESPACE_LIBRARIES LIBFREESPACE_INCLUDE_DIRS)
