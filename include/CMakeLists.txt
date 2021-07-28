configure_file( 
    ${CMAKE_CURRENT_SOURCE_DIR}/metis.h.in
    ${CMAKE_CURRENT_SOURCE_DIR}/metis.h
)

if(METIS_INSTALL)
  install(FILES metis.h DESTINATION include)
endif()
