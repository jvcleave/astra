set (_projname "orbbec_xs")

set(${_projname}_SOURCES
  XSPlugin.h
  XSPlugin.cpp
  PointProcessor.h
  PointProcessor.cpp
  PointStream.h
 )

add_library(${_projname} SHARED ${${_projname}_SOURCES})

set_target_properties(${_projname} PROPERTIES FOLDER "plugins")

target_link_libraries(${_projname} AstraAPI AstraUL)

include_directories(${_projname})

install_lib(${_projname} "Plugins")