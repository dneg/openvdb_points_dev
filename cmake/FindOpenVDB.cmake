# -*- cmake -*-
# - Find OpenVDB
#
# Author : James Bird (jsb@dneg.com)
#
# OpenVDB_FOUND            set if OpenVDB is found.
# OpenVDB_LOCATION         OpenVDB's installation prefix, as detected by this module
# OpenVDB_INCLUDE_DIR      OpenVDB's include directory
# OpenVDB_LIBRARY_DIR      OpenVDB's library directory
# OpenVDB_LIBRARY          OpenVDB's library name

FIND_PACKAGE ( PackageHandleStandardArgs )

FIND_PATH( OpenVDB_LOCATION include/openvdb/openvdb.h
  "$ENV{OPENVDB_ROOT}"
  NO_DEFAULT_PATH
  NO_SYSTEM_ENVIRONMENT_PATH
  )

FIND_PACKAGE_HANDLE_STANDARD_ARGS ( OpenVDB
  FOUND_VAR OpenVDB_FOUND
  REQUIRED_VARS OpenVDB_LOCATION
  )

IF ( OpenVDB_FOUND )

  # Static library setup
  IF(OpenVDB_USE_STATIC_LIBS)
    SET(CMAKE_FIND_LIBRARY_SUFFIXES_BACKUP ${CMAKE_FIND_LIBRARY_SUFFIXES})
    SET(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
  ENDIF()

  # Find openvdb library.
  FIND_LIBRARY ( OpenVDB_LIBRARY openvdb
    PATHS ${OpenVDB_LOCATION}/lib
    NO_DEFAULT_PATH
    NO_SYSTEM_ENVIRONMENT_PATH
    )

  # Static library teardown.
  IF(OpenVDB_USE_STATIC_LIBS)
    SET( CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_FIND_LIBRARY_SUFFIXES_BACKUP} )
  ENDIF()

  # Search paths
  SET( OpenVDB_INCLUDE_DIR "${OpenVDB_LOCATION}/include" CACHE STRING "OpenVDB include directory" )
  SET( OpenVDB_LIBRARY_DIR "${OpenVDB_LOCATION}/lib" CACHE STRING "OpenVDB library directory" )

ELSE( OpenVDB_FOUND )

  SET( OpenVDB_INCLUDE_DIR "NOT_FOUND" CACHE STRING "OpenVDB include directory" )
  SET( OpenVDB_LIBRARY_DIR "NOT_FOUND" CACHE STRING "OpenVDB library directory" )
  SET( OpenVDB_LIBRARY "NOT_FOUND" CACHE STRING "OpenVDB library name" )

ENDIF ( OpenVDB_FOUND )
