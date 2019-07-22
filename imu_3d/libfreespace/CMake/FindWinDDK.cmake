# Find WDK
#
# This module defines
#  WINDDK_FOUND
#  WINDDK_DIR
#  WINDDK_INCLUDE_DIR
#  WINDDK_WINXP_X86_LIBS
#  WINDDK_WIN7_X86_LIBS
#  WINDDK_WIN7_X64_LIBS
#

IF (NOT WINDDK_FOUND OR NOT WINDDK_DIR)
	# If WINDDK_DIR was defined in the environment, use it.
	IF (NOT DEFINED WINDDK_DIR AND NOT "$ENV{WINDDK_DIR}" STREQUAL "")
		SET(WINDDK_DIR $ENV{WINDDK_DIR})
	ENDIF()

	# If the WinDDK was not defined in the environment variables, take a couple guesses about where it could be
	# It only looks for the version 7, i.e. ~/WinDDK/7*
	IF(NOT DEFINED WINDDK_DIR AND EXISTS "C:/WinDDK" AND IS_DIRECTORY "C:/WinDDK")
		FILE(GLOB WINDDK_DIR "C:/WinDDK/7*")
	ENDIF()
	IF(NOT DEFINED WINDDK_DIR AND EXISTS "C:/Program Files/WinDDK" AND IS_DIRECTORY "C:/Program Files/WinDDK")
		FILE(GLOB WINDDK_DIR "C:/Program Files/WinDDK/7*")
	ENDIF()
	IF(NOT DEFINED WINDDK_DIR AND EXISTS "C:/Program Files (x86)/WinDDK" AND IS_DIRECTORY "C:/Program Files (x86)/WinDDK")
		FILE(GLOB WINDDK_DIR "C:/Program Files (x86)/WinDDK/7*")
	ENDIF()

	IF(DEFINED WINDDK_DIR)
		SET(WINDDK_FOUND TRUE CACHE INTERNAL "" FORCE)
	ENDIF()

	SET(WINDDK_DIR ${WINDDK_DIR} CACHE PATH "The WINDDK path for Windows 32-bit builds" FORCE)

	# log find result
	IF(WINDDK_FOUND)
		IF(NOT FIND_WINDDK_QUIETLY)
			MESSAGE(STATUS "WinDDK in ${WINDDK_DIR}")
		ENDIF()
	ELSE()
		UNSET(WINDDK_FOUND CACHE)
		MESSAGE(FATAL_ERROR "Could not locate WinDDK - Specify path using the WINDDK_DIR variable")
	ENDIF()

	# Search for header files
	SET(WINDDK_INCLUDE_SEARCH_DIRS ${WINDDK_DIR}/inc )
	FIND_PATH(WINDDK_INCLUDE_DIR api/hidpi.h
		PATHS ${WINDDK_INCLUDE_SEARCH_DIRS} )
	SET(WINDDK_INCLUDE_DIR ${WINDDK_INCLUDE_DIR} 
		CACHE STRING "Directory containing Ms WinDDK header files")

	# Search for Win7-x86 libs
	FIND_LIBRARY(WINDDK_WIN7_X86_HID_LIB hid 
		PATHS "${WINDDK_DIR}/lib/win7/i386"
		DOC "MS WinDDK Win7-x86 hid library"
	)	
	FIND_LIBRARY(WINDDK_WIN7_X86_SETUPAPI_LIB setupapi 
		PATHS "${WINDDK_DIR}/lib/win7/i386"
		DOC "MS WinDDK Win7-x86 setupapi library"
	)

	# Search for Win7-x64 libs
	FIND_LIBRARY(WINDDK_WIN7_X64_HID_LIB hid 
		PATHS "${WINDDK_DIR}/lib/win7/amd64"
		DOC "MS WinDDK Win7-x64 hid library"
	)
	FIND_LIBRARY(WINDDK_WIN7_X64_SETUPAPI_LIB setupapi 
		PATHS "${WINDDK_DIR}/lib/win7/amd64"
		DOC "MS WinDDK Win7-x64 setupapi library"
	)

	# Search for WinXP-x86 libs
	FIND_LIBRARY(WINDDK_WINXP_X64_HID_LIB hid 
		PATHS "${WINDDK_DIR}/lib/wxp/i386"
		DOC "MS WinDDK WinXP-x86 hid library"
	)
	FIND_LIBRARY(WINDDK_WINXP_X64_SETUPAPI_LIB setupapi 
		PATHS "${WINDDK_DIR}/lib/wxp/i386"
		DOC "MS WinDDK WinXP-x86 setupapi library"
	)

	# Set most variables as advanced
	MARK_AS_ADVANCED(
		WINDDK_INCLUDE_DIR 
		WINDDK_WIN7_X86_HID_LIB 
		WINDDK_WIN7_X86_SETUPAPI_LIB		 
		WINDDK_WIN7_X64_HID_LIB	 
		WINDDK_WIN7_X64_SETUPAPI_LIB	 
		WINDDK_WINXP_X64_HID_LIB	 
		WINDDK_WINXP_X64_SETUPAPI_LIB
	)
endif()
