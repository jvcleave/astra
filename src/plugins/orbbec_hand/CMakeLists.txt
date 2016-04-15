set (_projname "orbbec_hand")

set(${_projname}_SOURCES
  HandTracker.cpp
  HandTracker.h
  HandStream.h
  HandStream.cpp
  TrajectoryAnalyzer.h
  TrajectoryAnalyzer.cpp
  DebugHandStream.h
  DebugHandStream.cpp
  DebugVisualizer.h
  DepthUtility.h
  DepthUtility.cpp
  Segmentation.h
  Segmentation.cpp
  TrackedPoint.h
  PointProcessor.h
  PointProcessor.cpp
  TrackingData.h
  ScalingCoordinateMapper.h
  ScalingCoordinateMapper.cpp
  HandPlugin.cpp
  HandPlugin.h
  HandSettings.h
  SettingsParser.cpp
  constants.h
  ../../Astra/vendor/cpptoml.h
  orbbec_hand.toml
  )

if (ANDROID)
  if (NOT OpenCV_ANDROID_SDK_ROOT)
    message(FATAL_ERROR "OpenCV_ANDROID_SDK_ROOT must be set if building for Android.")
  endif()
  set(OpenCV_DIR "${OpenCV_ANDROID_SDK_ROOT}/sdk/native/jni")
  find_package(OpenCV REQUIRED core imgproc NO_DEFAULT_PATH)
else()
  find_package(OpenCV REQUIRED core imgproc)
endif()

#maps relwithbedinfo to release, so don't include debug version of opencv
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_MINSIZEREL RELEASE)

add_library(${_projname} SHARED ${${_projname}_SOURCES})

set_target_properties(${_projname} PROPERTIES FOLDER "plugins")

target_link_libraries(${_projname} AstraAPI AstraUL ${OpenCV_LIBS})

include_directories(${_projname} ${OpenCV_INCLUDE_DIRS})

include_directories(${_projname})

add_custom_target(copytoml_hand ALL
  #orbbec_hand.toml
  COMMAND ${CMAKE_COMMAND} -E copy
  "${PROJECT_SOURCE_DIR}/src/plugins/orbbec_hand/orbbec_hand.toml"
  "$<TARGET_FILE_DIR:${_projname}>")
set_target_properties(copytoml_hand PROPERTIES FOLDER CMakeCopyTargets)

install_lib(${_projname} "Plugins/")
install_file("${PROJECT_SOURCE_DIR}/src/plugins/orbbec_hand/orbbec_hand.toml" lib "Plugins/")
