# Add this directory for internal users.
include_directories(.)
# Find sources.
file( GLOB metis_sources *.c )
# Build libmetis.
add_library( metis ${METIS_LIBRARY_TYPE} ${GKlib_sources} ${metis_sources} )

if ( SHARED )
    target_compile_definitions( metis
        PRIVATE
            BUILD_METIS_DLL
)
endif()

if ( UNIX )
    target_link_libraries( metis m )
endif()

if ( METIS_INSTALL )
    install ( TARGETS metis
        RUNTIME DESTINATION bin 
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib  
    )
endif()
