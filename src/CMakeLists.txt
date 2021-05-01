set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-O3" )
#输出位置
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)  

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )
include_directories( /usr/include/eigen3 )

# 为使用 sophus，您需要使用find_package命令找到它
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

# opencv 
set(OpenCV_DIR /usr/local/include/opencv2)
find_package(OpenCV 3 REQUIRED) 
include_directories( ${OpenCV_INCLUDE_DIRS} )

# pcl 
find_package( PCL REQUIRED COMPONENT common io )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# 添加头文件的查找目录
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
# 添加源文件目录
aux_source_directory(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
add_executable(output ${DIR_SRCS})
#add_executable( output  main.cpp Camera.cpp Frame.cpp)
# 指定库文件输出路径(可以不生成库，直接注释)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
# 生成动态库
add_library(cam SHARED Camera.cpp)
add_library(frame SHARED Frame.cpp)
add_library(viewpoint SHARED Frame.cpp)
# 设置库输出名为
set_target_properties(cam PROPERTIES OUTPUT_NAME "Camera")
# 生成静态库
add_library(cama STATIC Camera.cpp)
add_library(framea STATIC Frame.cpp)
add_library(viewpointa SHARED Virtual_ViewPoint.cpp)
# 设置输库出名为
set_target_properties(cama PROPERTIES OUTPUT_NAME "Cameraa")

target_link_libraries( output   ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Sophus_LIBRARIES} cama framea viewpointa)
#target_link_libraries( output   ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Sophus_LIBRARIES})
