cmake_minimum_required( VERSION 3.20 )
project ( METIS )

#set ( GKLIB_PATH "GKlib" CACHE PATH "path to GKlib")
set ( GKLIB_PATH "GKlib" )
set ( SHARED FALSE CACHE BOOL "build a shared library" )

set ( METIS_INSTALL TRUE )

option ( METIS_ENABLE_64BIT "Enable or disable 64-bit code building" "ON" )

if ( METIS_ENABLE_64BIT )
    set( BUILD64BIT 1 )
else()
    set( BUILD64BIT 0 )
endif()

# Configure libmetis library.
if ( SHARED )
    set ( METIS_LIBRARY_TYPE SHARED )
else()
    set ( METIS_LIBRARY_TYPE STATIC )
endif()

include( ${GKLIB_PATH}/GKlibSystem.cmake )
# Add include directories.
include_directories( ${GKLIB_PATH} )
include_directories( include )
# Recursively look for CMakeLists.txt in subdirs.
add_subdirectory ( "include"  )
add_subdirectory ( "libmetis" )
add_subdirectory ( "programs" )
