cmake_minimum_required (VERSION 3.2)

project (Astra)

if (ANDROID)
  set(ASTRA_ANDROID TRUE)
elseif (WIN32)
  set(ASTRA_WINDOWS TRUE)
elseif (APPLE)
  set(ASTRA_OSX TRUE)
elseif (UNIX) #not android
  set(ASTRA_UNIX TRUE)
endif()

# Version
string (TIMESTAMP BUILD_DATE "%Y%m%dT%H%M%SZ" UTC)
set (Astra_VERSION_MAJOR 0)
set (Astra_VERSION_MINOR 4)
set (Astra_VERSION_PATCH 0)
set (Astra_VERSION_BUILD 0)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)

if("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
  set(ASTRA_CLANG ON)
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set(ASTRA_GCC ON)
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
  set(ASTRA_INTELC ON)
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  set(ASTRA_MSVC ON)
endif()

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(ASTRA_64 ON)
else()
  set(ASTRA_32 ON)
endif()

if (ASTRA_MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /MP")
  MESSAGE("CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}")
  MESSAGE("CMAKE_C_FLAGS ${CMAKE_C_FLAGS}")
endif()

set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(ASTRA_GCC OR ASTRA_CLANG)

  set(ASTRA_CXX_FLAGS "-Wall -fPIC")
  if (COMPILER_SUPPORTS_CXX14)
    set(ASTRA_CXX_FLAGS "${ASTRA_CXX_FLAGS} -std=c++14")
  elseif(COMPILER_SUPPORTS_CXX11)
    set(ASTRA_CXX_FLAGS "${ASTRA_CXX_FLAGS} -std=c++11")
  endif()

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ASTRA_CXX_FLAGS}")

  message("flags ${CMAKE_CXX_FLAGS}")

  if(ASTRA_ANDROID)
    set(ANDROID_CXX_FLAGS "${ANDROID_CXX_FLAGS} ${ASTRA_CXX_FLAGS}")
  endif()

endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(ASTRA_DOCS FALSE CACHE BOOL "Build documentation")
set(ASTRA_XS TRUE CACHE BOOL "Build extra stream support plugin")
set(ASTRA_HAND TRUE CACHE BOOL "Build hand tracking plugin")
set(ASTRA_STREAMPLAYER FALSE CACHE BOOL "Build experimental stream playback plugin")
set(ASTRA_MOCK_DEVICE FALSE CACHE BOOL "Build mock test device plugin")

if (ASTRA_DOCS)
  add_subdirectory(docs)
endif()

if(ASTRA_ANDROID)
  find_host_package(CLISP REQUIRED)
else()
  find_package(CLISP REQUIRED)
endif()

#Override FindAstra equivalents
set(ASTRA_SDK_BUILD TRUE)
set(ASTRA_LIBRARIES
  Astra
  AstraAPI
  AstraUL
  )
set(ASTRA_INCLUDE_DIR
  ${PROJECT_SOURCE_DIR}/include
  )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

set(SAMPLE_DIR_FOLDER "samples/")

set (SDK_DEPENDENT_TARGET "SDK_dependent")
add_custom_target(${SDK_DEPENDENT_TARGET})

#set(SHINY_INCLUDE "${PROJECT_SOURCE_DIR}/vendor/shinyprofiler/include")
set(CATCH_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/vendor/catch/include" CACHE INTERNAL "Path to include folder for Catch")

if (ASTRA_64)
  set(SAMPLE_DEPENDENCY_DIR "copy_to_bin64_dir")
else()
  set(SAMPLE_DEPENDENCY_DIR "copy_to_bin32_dir")
endif()

function(install_lib _targetname)
  set(extrapath ${ARGV1})
  install(TARGETS ${_targetname}
    RUNTIME DESTINATION bin/${extrapath}
    LIBRARY DESTINATION lib/${extrapath}
    ARCHIVE DESTINATION lib/${extrapath}
    )
  if (MSVC)
    install_pdb(${_targetname} bin/)
    set(INSTALL_PATHS
      samples/vs2013/${SAMPLE_DEPENDENCY_DIR}/${extrapath}
      samples/vs2013/bin/Debug/${extrapath}
      samples/vs2013/bin/Release/${extrapath}
    )
    foreach(INSTALL_PATH ${INSTALL_PATHS})
      install(TARGETS ${_targetname}
        RUNTIME DESTINATION ${INSTALL_PATH}
        )
      install_pdb(${_targetname} ${INSTALL_PATH})
    endforeach()
  endif()
endfunction()

function(install_file _targetfile basepath)
  set(extrapath ${ARGV2})
  if (MSVC)
    set(INSTALL_PATHS
      bin/${extrapath}
      samples/vs2013/${SAMPLE_DEPENDENCY_DIR}/${extrapath}
      samples/vs2013/bin/Debug/${extrapath}
      samples/vs2013/bin/Release/${extrapath}
    )
    foreach(INSTALL_PATH ${INSTALL_PATHS})
      install(FILES ${_targetfile}
        DESTINATION ${INSTALL_PATH}
        )
    endforeach()
  else()
    install(FILES ${_targetfile}
      DESTINATION ${basepath}/${extrapath}
    )
  endif()
endfunction()

function(install_dir _targetdir basepath)
  set(extrapath ${ARGV2})
  if (MSVC)
    set(INSTALL_PATHS
      bin/${extrapath}
      samples/vs2013/${SAMPLE_DEPENDENCY_DIR}/${extrapath}
      samples/vs2013/bin/Debug/${extrapath}
      samples/vs2013/bin/Release/${extrapath}
    )
    foreach(INSTALL_PATH ${INSTALL_PATHS})
      install(DIRECTORY ${_targetdir}
        DESTINATION ${INSTALL_PATH}
        )
    endforeach()
  else()
    install(DIRECTORY ${_targetdir}
      DESTINATION ${basepath}/${extrapath}
    )
  endif()
endfunction()

set(ASTRA_SDK_BINARY_DIR ${PROJECT_BINARY_DIR})
set(ASTRA_SDK_SOURCE_DIR ${PROJECT_SOURCE_DIR})

add_subdirectory(vendor)
add_subdirectory(src)
add_subdirectory(samples)
add_subdirectory(tests)

set(CMAKE_INSTALL_PREFIX "${PROJECT_BINARY_DIR}/sdk")
MESSAGE("CMAKE_INSTALL_PREFIX : ${CMAKE_INSTALL_PREFIX}")

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include
  DESTINATION .
  PATTERN "*.lpp" EXCLUDE
  PATTERN "include/AstraUL/Plugins" EXCLUDE
  )

install(DIRECTORY ${PROJECT_SOURCE_DIR}/samples
  DESTINATION .
  PATTERN "build" EXCLUDE
  PATTERN ".gitignore" EXCLUDE
  PATTERN "samples/plugins/orbbec_skeleton" EXCLUDE
  PATTERN "samples/sfml/SimpleSkeletonViewer-SFML" EXCLUDE
  PATTERN "samples/tools/StreamRecorder" EXCLUDE
  )

if (ASTRA_UNIX)
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/install
    DESTINATION .
    )
endif()

install(DIRECTORY ${PROJECT_SOURCE_DIR}/cmake
  DESTINATION samples
  PATTERN "build" EXCLUDE
  PATTERN ".gitignore" EXCLUDE
  )

if (MSVC)
  #copy third party build files to install vs2013 directory
  #Lack of a trailing / is important here, thirdparty ends up in samples/vs2013/
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/samples-aux/vs2013/thirdparty
    DESTINATION samples/vs2013/
    PATTERN "build" EXCLUDE
    PATTERN ".gitignore" EXCLUDE
    PATTERN ".git" EXCLUDE
    )

  #copy samples runtime files to all the places vs2013 would want them
  set(INSTALL_PATHS
    bin/
    samples/vs2013/${SAMPLE_DEPENDENCY_DIR}/
    samples/vs2013/bin/Debug/
    samples/vs2013/bin/Release/
  )
  foreach(INSTALL_PATH ${INSTALL_PATHS})
   install(DIRECTORY ${PROJECT_SOURCE_DIR}/samples-aux/vs2013/${SAMPLE_DEPENDENCY_DIR}/
     DESTINATION ${INSTALL_PATH}
     PATTERN "opencv_core2410d.dll" EXCLUDE
     PATTERN "opencv_imgproc2410d.dll" EXCLUDE
     )
  endforeach()

  #prepare to generate vs2013 solution for installation
  set(SAMPLES_SOURCE_DIR "${PROJECT_BINARY_DIR}/sdk/samples")
  set(SAMPLES_BUILD_TEMP_DIR "${PROJECT_BINARY_DIR}/temp/vs2013/")

  #This doesn't replace anything except line endings for Windows CRLF style
  configure_file("${PROJECT_SOURCE_DIR}/samples-aux/README.txt"
    "${PROJECT_BINARY_DIR}/temp/README.txt"
    @ONLY
    NEWLINE_STYLE CRLF)

  #install the CRLF line ending readme
  install(FILES ${PROJECT_BINARY_DIR}/temp/README.txt
    DESTINATION .
    )

  #Configure VS2013 project generation & cleanup script
  configure_file("${PROJECT_SOURCE_DIR}/samples-aux/InstallVS.cmake.in"
    "${PROJECT_BINARY_DIR}/InstallVS.cmake"
    @ONLY)
  #Run the script at install time to generate VS2013 projects for the samples
  #also de-cmake-ifies the projects and makes everything a relative directory
  install(SCRIPT "${PROJECT_BINARY_DIR}/InstallVS.cmake")

  #install the sample VS2013 projects
  install(DIRECTORY ${SAMPLES_BUILD_TEMP_DIR}
    DESTINATION samples/vs2013
    PATTERN "build" EXCLUDE
    PATTERN ".gitignore" EXCLUDE
    )
  #remove temporary build dir
  install(CODE "file(REMOVE_RECURSE ${SAMPLES_BUILD_TEMP_DIR})")
else()
  #for non-MSVC platforms, just install the readme directly
  install(FILES ${PROJECT_SOURCE_DIR}/samples-aux/README.txt
    DESTINATION .
    )
endif()

set(CPACK_PACKAGE_NAME "AstraSDK")
set(CPACK_PACKAGE_VENDOR "Orbbec")
set(CPACK_PACKAGE_DIRECTORY "${PROJECT_BINARY_DIR}/sdk_package")
set(CPACK_PACKAGE_VERSION_MAJOR ${Astra_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${Astra_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH "${Astra_VERSION_PATCH}-${BUILD_DATE}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Orbbec Astra SDK")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "Orbbec/Astra")

message("CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")
string(TOLOWER "${CMAKE_SYSTEM_NAME}" system_name_lower)
if (system_name_lower STREQUAL "darwin")
  if (ASTRA_64)
    set(CMAKE_SYSTEM_NAME "osx-x64")
  else()
    error("32-bit OS X not supported")
  endif()
  message("Setting CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")
endif()

if (WIN32)
  set(CPACK_GENERATOR "ZIP")
else()
  set(CPACK_GENERATOR "TGZ")
endif()

INCLUDE(CPack)
