cmake_minimum_required(VERSION 3.3)
project(vdb2htg LANGUAGES CXX)

#########################################################################
# Monitoring <PackageName>_ROOT variables
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
include(GNUInstallDirs)

#########################################################################

message(STATUS "----------------------------------------------------")
message(STATUS "----------- Configuring vdb2htg --------------------")
message(STATUS "----------------------------------------------------")

##########################################################################

# Collect lib dependencies shared by all binaries

if(NOT OPENVDB_BUILD_CORE)
  # @note  Could also use the openvdb_je target here, but we just opt to
  # handle the value of CONCURRENT_MALLOC outside of this branching for
  # both cases
  set(OPENVDB_LIB OpenVDB::openvdb)
else()
  set(OPENVDB_LIB openvdb)
endif()

set(OPENVDB_BINARIES_DEPENDENT_LIBS
  ${OPENVDB_LIB}
)

if(CONCURRENT_MALLOC STREQUAL "Jemalloc")
  find_package(Jemalloc REQUIRED)
  list(APPEND OPENVDB_BINARIES_DEPENDENT_LIBS Jemalloc::jemalloc)
elseif(CONCURRENT_MALLOC STREQUAL "Tbbmalloc")
  find_package(TBB ${MINIMUM_TBB_VERSION} REQUIRED COMPONENTS tbbmalloc)
  list(APPEND OPENVDB_BINARIES_DEPENDENT_LIBS TBB::tbbmalloc)
endif()

##########################################################################

# rpath handling

set(RPATHS "")
if(OPENVDB_ENABLE_RPATH)
  # @todo There is probably a better way to do this for imported targets
  list(APPEND RPATHS
    ${Boost_LIBRARY_DIRS}
    ${IlmBase_LIBRARY_DIRS}
    ${Log4cplus_LIBRARY_DIRS}
    ${Blosc_LIBRARY_DIRS}
    ${Tbb_LIBRARY_DIRS}
  )
  if(OPENVDB_BUILD_CORE)
    list(APPEND RPATHS ${CMAKE_INSTALL_PREFIX}/lib)
  else()
    list(APPEND RPATHS ${OpenVDB_LIBRARY_DIRS})
  endif()

  list(REMOVE_DUPLICATES RPATHS)
endif()

##########################################################################

message(STATUS "OPENVDB_BINARIES_DEPENDENT_LIBS "${OPENVDB_BINARIES_DEPENDENT_LIBS})

include_directories(${CMAKE_CURRENT_LIST_DIR})

add_executable(vdb2htg main.cpp
  rkcommon/common.cpp)

target_link_libraries(vdb2htg ${OPENVDB_BINARIES_DEPENDENT_LIBS})

if(OPENVDB_ENABLE_RPATH)
  set_target_properties(vdb2htg
    PROPERTIES INSTALL_RPATH "${RPATHS}"
  )
endif()

install(TARGETS vdb2htg RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
