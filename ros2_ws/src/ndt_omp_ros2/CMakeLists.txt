cmake_minimum_required(VERSION 3.10)
project(ndt_omp_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common registration filters)
find_package(OpenMP REQUIRED)

add_library(pclomp SHARED
  src/pclomp/ndt_omp.cpp
  src/pclomp/gicp_omp.cpp
  src/pclomp/voxel_grid_covariance_omp.cpp
)
target_include_directories(pclomp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(pclomp ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)
target_compile_options(pclomp PRIVATE ${OpenMP_CXX_FLAGS})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS pclomp
  EXPORT export_pclomp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(pclomp)
ament_package()
