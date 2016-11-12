# -*- cmake -*-
# - Find cppunit
#
# Author : James Bird (jsb@dneg.com)
#
# CPPUNIT_FOUND            set if cppunit is found.
# CPPUNIT_INCLUDE_DIR      cppunit's include directory
# CPPUNIT_LIBRARY_DIR      cppunit's library directory
# CPPUNIT_LIBRARY          cppunit library name

FIND_PACKAGE ( PackageHandleStandardArgs )

FIND_PATH( CPPUNIT_LOCATION include/cppunit/config-auto.h
  "$ENV{CPPUNIT_ROOT}"
  NO_DEFAULT_PATH
  NO_SYSTEM_ENVIRONMENT_PATH
  )

FIND_PACKAGE_HANDLE_STANDARD_ARGS ( cppunit
  FOUND_VAR CPPUNIT_FOUND
  REQUIRED_VARS CPPUNIT_LOCATION
  )

IF ( CPPUNIT_FOUND )
  FIND_LIBRARY ( CPPUNIT_LIBRARY cppunit
    PATHS ${CPPUNIT_LOCATION}/lib
    NO_DEFAULT_PATH
    NO_SYSTEM_ENVIRONMENT_PATH
    )
  SET( CPPUNIT_INCLUDE_DIR "${CPPUNIT_LOCATION}/include" CACHE STRING "CPPUNIT include directory" )
ENDIF ( CPPUNIT_FOUND )
