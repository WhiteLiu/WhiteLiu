cmake_minimum_required( VERSION 2.8 )
project ( cpp_note )

set(G2O_DIR "/home/jinhuazhe/Downloads/liu/slambook/3rdparty/g2o/build")

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块以使用ceres库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( PCL REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
find_package( Cholmod )
# 寻找G2O
find_package( G2O REQUIRED )
include_directories(
    ${G2O_INCLUDE_DIRS}
    "/usr/include/eigen3"
${CHOLMOD_INCLUDE_DIR}
${PCL_INCLUDE_DIRS}
)

set(OpenCV_DIR "/home/jinhuazhe/Downloads/liu/opencv-2.4.13/build")
find_package(OpenCV REQUIRED)

add_executable( main main.cpp )
target_link_libraries( main ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable( plusOneUntil c.cpp )
target_link_libraries( plusOneUntil )


add_executable( map_example map_example.cpp )
target_link_libraries( map_example  )

add_executable( vector_example vector_example.cpp )
target_link_libraries( vector_example  )


add_executable( cv_example cv_example.cpp )
target_link_libraries( cv_example
${OpenCV_LIBS}
  )


add_executable( list_example list_example.cpp )
target_link_libraries( list_example  )

add_executable( f f.cpp )
target_link_libraries( f )

add_library(img img.cpp)



#add_executable( rand_example rand_example.cpp )
#target_link_libraries( rand_example)


add_executable( ba_g2o ba_g2o.cpp
common.cpp
)
target_link_libraries( ba_g2o
${OpenCV_LIBS}
${PCL_LIBRARIES}
g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sba
${CHOLMOD_LIBRARIES}
  )
