project(Echll)
cmake_minimum_required(VERSION 2.8)
set(Echll_MAJOR 0)
set(Echll_MINOR 0)
set(Echll_PATCH 99)
set(Echll_VERSION "${Echll_MAJOR}.${Echll_MINOR}.${Echll_PATCH}")
set(Echll_STABLEVERSION "${Echll_MAJOR}.${Echll_MINOR}")

if (CMAKE_BUILD_TYPE STREQUAL "")
  set (CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build" FORCE)
endif ()

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++11")

  if ("${CMAKE_BUILD_TYPE}" EQUAL "Debug" OR "${CMAKE_BUILD_TYPE}" EQUAL "RelWithDebInfo")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ggdb3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb3")
  endif ()
endif ()

include_directories(. ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_BINARY_DIR})
find_package(PkgConfig)

macro (pkg_config_library LIBNAME pcfile)
  pkg_check_modules(${LIBNAME} REQUIRED ${pcfile})
  include_directories(${${LIBNAME}_INCLUDE_DIRS})
  link_directories(${${LIBNAME}_LIBRARY_DIRS})
  add_definitions(${${LIBNAME}_CFLAGS_OTHER})
  set(Echll_LINK_LIBRARIES ${Echll_LINK_LIBRARIES} ${${LIBNAME}_LIBRARIES})
  set(Echll_STATIC_LINK_LIBRARIES ${Echll_STATIC_LINK_LIBRARIES} ${${LIBNAME}_STATIC_LIBRARIES})
endmacro ()

#pkg_config_library(LIBMPI ompi-cxx)

message(STATUS "checking for a C++ compiler with MPI support")
find_package(MPI REQUIRED)
if (NOT MPI_CXX_FOUND)
  message(ERROR " failed to found a C++ compiler with MPI support.
  Please assign MPI_CXX_COMPILER to the MPI wrapper (mpicc, etc.) and
  rerun CMake. If this fails, set both MPI_CXX_LIBRARIES and
  MPI_CXX_INCLUDE_PATH and rerun CMake.")
else()
  message(STATUS " MPI support enabled")
endif ()


### # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
## Echll-x.y.pc
#

if (WIN32)
  set(Echll_PKGCONFIG_PREFIXPATH "/target")
  set(Echll_PKGCONFIG_LIBDIR "bin")
else()
  set(Echll_PKGCONFIG_PREFIXPATH "${CMAKE_INSTALL_PREFIX}")
  set(Echll_PKGCONFIG_LIBDIR "lib")
endif()

configure_file(Echll.pc.in ${PROJECT_BINARY_DIR}/Echll-${Echll_STABLEVERSION}.pc @ONLY)
install(FILES ${PROJECT_BINARY_DIR}/Echll-${Echll_STABLEVERSION}.pc
  DESTINATION lib/pkgconfig)

### # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
## Core library
#

set(Echll_CORE_HDRS vle/common.hpp vle/dbg.hpp vle/devs.hpp
  vle/dsde.hpp vle/dtss.hpp vle/environment.hpp vle/export.hpp
  vle/heap.hpp vle/i18n.hpp vle/init.hpp vle/path.hpp vle/port.hpp
  vle/time.hpp vle/utils.hpp vle/vle.hpp)

set(Echll_CORE_SRCS vle/common.hpp vle/dbg.hpp vle/devs.hpp
  vle/dsde.hpp vle/dtss.hpp vle/environment.cpp vle/environment.hpp
  vle/export.hpp vle/heap.hpp vle/i18n.hpp vle/init.hpp vle/path.cpp
  vle/path.hpp vle/port.hpp vle/time.hpp vle/utils.cpp vle/utils.hpp
  vle/vle.hpp)

add_library(Echll_corelib SHARED ${Echll_CORE_SRCS})
add_library(Echll_static_corelib STATIC ${Echll_CORE_SRCS})

set_target_properties(Echll_corelib PROPERTIES
  VERSION 0
  OUTPUT_NAME "echll-${Echll_STABLEVERSION}"
  DEFINE_SYMBOL "Echll_EXPORTS"
  COMPILE_DEFINITIONS "Echll_DLL"
  COMPILE_FLAGS "-fvisibility=hidden -fvisibility-inlines-hidden")

if (WIN32)
  target_link_libraries(Echll_corelib ${Echll_STATIC_LINK_LIBRARIES})
else ()
  target_link_libraries(Echll_corelib ${Echll_LINK_LIBRARIES})
endif ()

set_target_properties(Echll_static_corelib PROPERTIES
  VERSION 0
  OUTPUT_NAME "echll-${Echll_STABLEVERSION}"
  DEFINE_SYMBOL "Echll_EXPORTS"
  COMPILE_FLAGS "-fvisibility=hidden -Fvisibility-inlines-hidden")

install(FILES ${Echll_CORE_HDRS}
  DESTINATION include/echll-${Echll_STABLEVERSION})

install(TARGETS Echll_corelib
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(TARGETS Echll_static_corelib
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

### # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
## Executable
#

set(Echll_MAIN_SRCS vle/main.cpp)

add_executable(echll ${Echll_MAIN_SRCS})

if (MPI_CXX_FOUND)
  include_directories(${MPI_CXX_INCLUDE_PATH})
  target_link_libraries(echll ${MPI_CXX_LIBRARIES})

  if (MPI_CXX_COMPILE_FLAGS)
    set_target_properties(echll PROPERTIES COMPILE_FLAGS "${MPI_CXX_COMPILE_FLAGS}")
  endif ()

  if (MPI_CXX_LINK_FLAGS)
    set_target_properties(echll PROPERTIES LINK_FLAGS "${MPI_CXX_LINK_FLAGS}")
  endif ()
endif ()

if (WIN32)
  target_link_libraries(echll Echll_static_corelib ${Echll_STATIC_LINK_LIBRARIES})
  set_target_properties(echll PROPERTIES
    COMPILE_FLAGS "-fvisibility=hidden -fvisibility-inlines-hidden")
else ()
  target_link_libraries(echll Echll_corelib ${Echll_LINK_LIBRARIES})
  set_target_properties(echll PROPERTIES
    VERSION ${Echll_STABLEVERSION}
    COMPILE_FLAGS "-fvisibility=hidden -fvisibility-inlines-hidden")
endif ()

install(TARGETS echll DESTINATION bin)

### # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
## Testing
#

enable_testing()

message(STATUS "checking for 'catch.hpp'")
find_path(CATCH_INCLUDE_DIR catch.hpp PATHS /usr/include /usr/local/include ENV CATCH_INCLUDE_DIR)

if (CATCH_INCLUDE_DIR)
  message(STATUS " found 'catch.hpp' in ${CATCH_INCLUDE_DIR}")
  include_directories(${CATCH_INCLUDE_DIR})

  add_executable(test_utils tests/utils.cpp)
  target_link_libraries(test_utils Echll_corelib ${Echll_LINK_LIBRARIES})
  set_target_properties(test_utils PROPERTIES
    COMPILE_DEFINITIONS EXAMPLES_DIR="${CMAKE_SOURCE_DIR}/tests")
  add_test(NAME test_utils COMMAND test_utils)

  add_executable(test_devs-engine tests/devs-engine.cpp)
  target_link_libraries(test_devs-engine Echll_corelib ${Echll_LINK_LIBRARIES})
  set_target_properties(test_devs-engine PROPERTIES
    COMPILE_DEFINITIONS EXAMPLES_DIR="${CMAKE_SOURCE_DIR}/tests")
  add_test(NAME test_devs-engine COMMAND test_devs-engine)

  add_executable(test_dsde-engine tests/dsde-engine.cpp)
  target_link_libraries(test_dsde-engine Echll_corelib pthread ${Echll_LINK_LIBRARIES})
  set_target_properties(test_dsde-engine PROPERTIES
    COMPILE_DEFINITIONS EXAMPLES_DIR="${CMAKE_SOURCE_DIR}/tests")
  add_test(NAME test_dsde-engine COMMAND test_dsde-engine)

  add_executable(test_dtss-engine tests/dtss-engine.cpp)
  target_link_libraries(test_dtss-engine Echll_corelib ${Echll_LINK_LIBRARIES})
  set_target_properties(test_dtss-engine PROPERTIES
    COMPILE_DEFINITIONS EXAMPLES_DIR="${CMAKE_SOURCE_DIR}/tests")
  add_test(NAME test_dtss-engine COMMAND test_dtss-engine)
else ()
  message(STATUS " not found catch.hpp. Unit test disabled")
endif ()


### # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
## CPack
#

include(InstallRequiredSystemLibraries)

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Echll - A new kernel for VLE")
set(CPACK_PACKAGE_VENDOR "INRA")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${PROJECT_SOURCE_DIR}/README")
set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/COPYING")
set(CPACK_PACKAGE_NAME "Echll")
set(CPACK_PACKAGE_VERSION_MAJOR "${Echll_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${Echll_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${Echll_PATCH}")
SET(CPACK_PACKAGE_INSTALL_DIRECTORY "Echll ${Echll_MAJOR}.${Echll_MINOR}")
set(CPACK_STRIP_FILES "1")
set(CPACK_PACKAGE_CONTACT "Gauthier Quesnel <gauthier.quesnel@toulouse.inra.fr>")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY "1")

if (WIN32)
  set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY "0")
  set (CPACK_GENERATOR "ZIP;NSIS")
  set (CPACK_SOURCE_GENERATOR "ZIP")
else ()
  set (CPACK_GENERATOR "TGZ;DEB;RPM")
  set (CPACK_SOURCE_GENERATOR "TGZ;TBZ2")
endif ()

set(CPACK_SOURCE_PACKAGE_FILE_NAME "echll-${Echll_VERSION}")
set(CPACK_SOURCE_IGNORE_FILES
  "/\\\\.git/"
  "\\\\.#"
  "/#"
  "/\\\\.gitignore$"
  "/\\\\.gitattributes$"
  "/build-debug-clang/"
  "/build-debug-gcc/"
  "/build-release-clang/"
  "/build-release-gcc/"
  "/build-cross/"
  "/coverage/"
  )

if (WIN32 AND NOT UNIX)
  set(CPACK_NSIS_MUI_ICON "${PROJECT_SOURCE_DIR}\\\\echll.ico")
  set(CPACK_NSIS_MUI_UNIICON "${PROJECT_SOURCE_DIR}\\\\echll.ico")
  set(CPACK_PACKAGE_ICON "${PROJECT_SOURCE_DIR}\\\\echll.bmp")
  set(CPACK_NSIS_INSTALLED_ICON_NAME "bin\\\\echll.exe")
  set(CPACK_NSIS_DISPLAY_NAME "${CPACK_PACKAGE_INSTALL_DIRECTORY} Echll ${Echll_VERSION}")
  set(CPACK_NSIS_HELP_LINK "http:\\\\\\\\https://github.com/quesnel/Echll")
  set(CPACK_NSIS_URL_INFO_ABOUT "http:\\\\\\\\https://github.com/quesnel/Echll")
  set(CPACK_NSIS_CONTACT "gauthier.quesnel@toulouse.inra.fr")
  set(CPACK_NSIS_MODIFY_PATH ON)
else ()
  set(CPACK_STRIP_FILES "bin/echll")
endif ()

INCLUDE(CPack)