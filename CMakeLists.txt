
cmake_minimum_required(VERSION 2.8)

project(Correspondance)


#Set variables 
set(CD ${CMAKE_CURRENT_SOURCE_DIR})
set(LIBDIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(BUILD ${CMAKE_CURRENT_SOURCE_DIR}/build)
set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(LOCAL_INCLUDE  /usr/local/include)
message("Variabili dei percorsi impostate")


find_package(PCL 1.8 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS} ${CD}/include ${LOCAL_INCLUDE}/eigen3/)
link_directories(${PCL_LIBRARY_DIRS} ${LIB} ${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(${PCL_DEFINITIONS})
message("Libreria PCL trovata")


#Required for cgal library
set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true )
 
if ( COMMAND cmake_policy )

  cmake_policy( SET CMP0003 NEW )  

endif()

# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )

if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()

# include helper file
include( ${CGAL_USE_FILE} )

message("CGAL Trovata")
# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()
message("Boost trovata")
# include for local directory

# include for local package
include_directories( BEFORE ../../include )
link_directories(${LIBDIR})
SET(CMAKE_CXX_FLAGS "-std=c++0x")

add_library(SEGMENTA ${SRC}/SEGMENTA.cpp)
add_library(SGURF_module ${SRC}/SGURF.cpp)
add_library(LettorePLY_module ${SRC}/lettorePLY.cpp)
add_library(FPFH ${SRC}/FPFH.cpp)
add_library(EliminaPiano ${SRC}/EliminaPiano.cpp)
add_library(DescrizioneCloud ${SRC}/DescrizioneCloud.cpp)
add_library(Correspondance ${SRC}/Correspondance.cpp)
add_executable(main ${SRC}/main.cpp)


message( "ehi" ${LIBDIR})
target_link_libraries (main ${PCL_LIBRARIES}  ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} 
Correspondance
DescrizioneCloud
 boost_filesystem
 SEGMENTA
 SGURF_module LettorePLY_module
FPFH 
VisualizzatorePatch
EliminaPiano)

