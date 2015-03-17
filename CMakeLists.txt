project(Echll)
cmake_minimum_required(VERSION 2.8.7)
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
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++1y")

  if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    if (${CMAKE_CXX_COMPILER_VERSION} VERSION_EQUAL "3.5" OR ${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER "3.5")
      message(STATUS "Clang ${CMAKE_CXX_COMPILER_VERSION} so, remarks system enabled")
      set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Weverything")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Weverything")
    endif ()
  endif ()

  if ("${CMAKE_BUILD_TYPE}" EQUAL "Debug" OR "${CMAKE_BUILD_TYPE}" EQUAL "RelWithDebInfo")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ggdb3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb3")
  endif ()
endif ()

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_BINARY_DIR})

message(STATUS "checking for a (p)thread library")
find_package(Threads REQUIRED)
if (NOT Threads_FOUND)
  message(ERROR " failed to found a (p)thread support.")
else ()
  message(STATUS " threads support enabled (${CMAKE_THREAD_LIBS_INIT})")
  set(Echll_LINK_LIBRARIES ${Echll_LINK_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
endif ()

message(STATUS "checking for a C++ compiler with MPI support")
find_package(MPI REQUIRED)
if (NOT MPI_CXX_FOUND)
  message(ERROR " failed to found a C++ compiler with MPI support.
  Please assign MPI_CXX_COMPILER to the MPI wrapper (mpicc, etc.) and
  rerun CMake. If this fails, set both MPI_CXX_LIBRARIES and
  MPI_CXX_INCLUDE_PATH and rerun CMake.")
else()
  message(STATUS " MPI support enabled")
  include_directories(${MPI_CXX_INCLUDE_PATH})
  set(Echll_LINK_LIBRARIES ${Echll_LINK_LIBRARIES} ${MPI_CXX_LIBRARIES})
endif ()

message(STATUS "checking for Boost headers and libraries")
find_package(Boost 1.55 REQUIRED COMPONENTS mpi serialization)
if (NOT Boost_FOUND)
  message(ERROR " failed to found Boost headers or Boost MPI library.
  Please install them or assign BOOST_ROOT, BOOST_INCLUDREDIR,
  BOOST_LIBRARYDIR and rerun CMake. Read the installed FindBoost in
  CMAke's directory too.")
else ()
  message(STATUS " Boost found")
  include_directories(${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS})
  set(Echll_LINK_LIBRARIES ${Echll_LINK_LIBRARIES} ${Boost_LIBRARIES})
endif ()

### # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
## Echll-x.y.pc
#

set(Echll_PKGCONFIG_PREFIX_DIR "${CMAKE_INSTALL_PREFIX}")
set(Echll_PKGCONFIG_BOOSTINCLUDE_DIRS "${Boost_INCLUDE_DIRS}")

configure_file(Echll.pc.in ${PROJECT_BINARY_DIR}/Echll-${Echll_STABLEVERSION}.pc @ONLY)
install(FILES ${PROJECT_BINARY_DIR}/Echll-${Echll_STABLEVERSION}.pc
  DESTINATION lib/pkgconfig)

### # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
## Core library
#

install(DIRECTORY vle
  DESTINATION include/Echll-${Echll_STABLEVERSION}/vle)

### # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
## Testing
#

option(WITH_LOGGING "Logging system in echll [default=ON]" ON)
option(WITH_DEBUG "Debugging system in echll [default=ON]" ON)

if (WITH_LOGGING)
    set(echll_compile_flags "-DENABLE_LOGGING")
endif ()

if (WITH_DEBUG)
    set(echll_compile_flags "-DENABLE_DEBUG ${echll_compile_flags}")
endif ()

enable_testing()
message(STATUS "checking for 'catch.hpp'")
find_path(CATCH_INCLUDE_DIR catch.hpp PATHS /usr/include /usr/local/include ENV CATCH_INCLUDE_DIR)

if (CATCH_INCLUDE_DIR)
  message(STATUS " found 'catch.hpp' in ${CATCH_INCLUDE_DIR}")
  include_directories(${CATCH_INCLUDE_DIR})

  add_executable(test_utils tests/utils.cpp)
  target_link_libraries(test_utils ${Echll_LINK_LIBRARIES})
  set_target_properties(test_utils PROPERTIES
    COMPILE_DEFINITIONS EXAMPLES_DIR="${CMAKE_SOURCE_DIR}/tests"
    COMPILE_FLAGS "${echll_compile_flags}")
  add_test(NAME test_utils COMMAND test_utils)

  add_executable(test_devs-engine tests/devs-engine.cpp)
  target_link_libraries(test_devs-engine ${Echll_LINK_LIBRARIES})
  set_target_properties(test_devs-engine PROPERTIES
    COMPILE_DEFINITIONS EXAMPLES_DIR="${CMAKE_SOURCE_DIR}/tests"
    COMPILE_FLAGS "${echll_compile_flags}")
  add_test(NAME test_devs-engine COMMAND test_devs-engine)

  add_executable(test_dsde-engine tests/dsde-engine.cpp)
  target_link_libraries(test_dsde-engine ${Echll_LINK_LIBRARIES})
  set_target_properties(test_dsde-engine PROPERTIES
    COMPILE_DEFINITIONS EXAMPLES_DIR="${CMAKE_SOURCE_DIR}/tests"
    COMPILE_FLAGS "${echll_compile_flags}")
  add_test(NAME test_dsde-engine COMMAND test_dsde-engine)

  add_executable(test_dsde-aqss-engine tests/dsde-aqss.cpp)
  target_link_libraries(test_dsde-aqss-engine ${Echll_LINK_LIBRARIES})
  set_target_properties(test_dsde-aqss-engine PROPERTIES
    COMPILE_DEFINITIONS EXAMPLES_DIR="${CMAKE_SOURCE_DIR}/tests"
    COMPILE_FLAGS "${echll_compile_flags}")
  add_test(NAME test_dsde-aqss-engine COMMAND test_dsde-aqss-engine)

  add_executable(test_dtss-engine tests/dtss-engine.cpp)
  target_link_libraries(test_dtss-engine ${Echll_LINK_LIBRARIES})
  set_target_properties(test_dtss-engine PROPERTIES
    COMPILE_DEFINITIONS EXAMPLES_DIR="${CMAKE_SOURCE_DIR}/tests"
    COMPILE_FLAGS "${echll_compile_flags}")
  add_test(NAME test_dtss-engine COMMAND test_dtss-engine)

  add_executable(test_mpi-synchronous tests/mpi-synchronous.cpp)
  target_link_libraries(test_mpi-synchronous ${Echll_LINK_LIBRARIES})
  set_target_properties(test_mpi-synchronous PROPERTIES
    COMPILE_DEFINITIONS EXAMPLES_DIR="${CMAKE_SOURCE_DIR}/tests"
    COMPILE_FLAGS "${echll_compile_flags}")
  add_test(NAME test_mpi-synchronous COMMAND mpirun -np 4
    test_mpi-synchronous)

  add_executable(test_tree-mpi-synchronous tests/tree.cpp)
  target_link_libraries(test_tree-mpi-synchronous ${Echll_LINK_LIBRARIES})
  set_target_properties(test_tree-mpi-synchronous PROPERTIES
    COMPILE_DEFINITIONS EXAMPLES_DIR="${CMAKE_SOURCE_DIR}/tests"
    COMPILE_FLAGS "${echll_compile_flags}")
  add_test(NAME test_tree-mpi-synchronous COMMAND mpirun -np 4
    test_tree-mpi-synchronous)

  add_executable(test_mpi-timewarp tests/mpi-timewarp.cpp)
  target_link_libraries(test_mpi-timewarp ${Echll_LINK_LIBRARIES})
  set_target_properties(test_mpi-timewarp PROPERTIES
    COMPILE_DEFINITIONS EXAMPLES_DIR="${CMAKE_SOURCE_DIR}/tests"
    COMPILE_FLAGS "${echll_compile_flags}")
  add_test(NAME test_mpi-timewarp COMMAND mpirun -np 4
    test_mpi-timewarp)
else ()
  message(STATUS " not found catch.hpp. Unit test disabled")
endif ()


### # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
## CPack
#

include(InstallRequiredSystemLibraries)

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Echll - A new kernel for VLE")
set(CPACK_PACKAGE_VENDOR "INRA")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${PROJECT_SOURCE_DIR}/README.md")
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
