set (_projname "AstraUL")

set(${_projname}_SOURCES
  ../../include/AstraUL/AstraUL.h
  ../../include/AstraUL/skul_capi.h
  ../../include/AstraUL/Vector.h
  ../../include/AstraUL/Vector2f.h
  ../../include/AstraUL/Vector2i.h
  ../../include/AstraUL/Vector3f.h
  ../../include/AstraUL/Vector3i.h
  ../../include/AstraUL/astraul_ctypes.h
  ../../include/AstraUL/streams/Image.h
  ../../include/AstraUL/streams/image_capi.h
  ../../include/AstraUL/streams/image_types.h
  ../../include/AstraUL/streams/image_parameters.h
  ../../include/AstraUL/streams/Depth.h
  ../../include/AstraUL/streams/depth_capi.h
  ../../include/AstraUL/streams/depth_types.h
  ../../include/AstraUL/streams/depth_parameters.h
  ../../include/AstraUL/streams/Color.h
  ../../include/AstraUL/streams/color_capi.h
  ../../include/AstraUL/streams/color_types.h
  ../../include/AstraUL/streams/Infrared.h
  ../../include/AstraUL/streams/infrared_capi.h
  ../../include/AstraUL/streams/infrared_types.h
  ../../include/AstraUL/streams/Hand.h
  ../../include/AstraUL/streams/DebugHand.h
  ../../include/AstraUL/streams/Skeleton.h
  ../../include/AstraUL/streams/hand_capi.h
  ../../include/AstraUL/streams/hand_types.h
  ../../include/AstraUL/streams/hand_parameters.h
  ../../include/AstraUL/streams/skeleton_capi.h
  ../../include/AstraUL/streams/skeleton_types.h
  ../../include/AstraUL/Plugins/stream_types.h
  ../../include/AstraUL/streams/Point.h
  ../../include/AstraUL/streams/point_capi.h
  ../../include/AstraUL/streams/point_types.h
  depth_capi.cpp
  color_capi.cpp
  infrared_capi.cpp
  image_capi.cpp
  hand_capi.cpp
  point_capi.cpp
  skeleton_capi.cpp
  AstraUL.cpp
  generic_stream_api.h
  )

add_library(${_projname} SHARED ${${_projname}_SOURCES})

add_definitions(-DASTRA_BUILD_EX)

target_link_libraries(${_projname} AstraAPI)

include_directories(${_projname})

install_lib(${_projname})