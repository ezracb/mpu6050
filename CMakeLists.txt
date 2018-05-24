cmake_minimum_required(VERSION 2.8.3)
project(mpu6050)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/mpu6050
)

set(SOURCE_FILES
	${PROJECT_SOURCE_DIR}/include/mpu6050/I2Cdev.h
	${PROJECT_SOURCE_DIR}/include/mpu6050/MPU6050.h
        ${PROJECT_SOURCE_DIR}/include/mpu6050/MPU6050_6Axis_MotionApps20.h
        ${PROJECT_SOURCE_DIR}/include/mpu6050/helper_3dmath.h
        ${PROJECT_SOURCE_DIR}/src/I2Cdev.cpp
	${PROJECT_SOURCE_DIR}/src/MPU6050.cpp
        ${PROJECT_SOURCE_DIR}/src/demo_dmp.cpp

)

add_executable(mpu6050_node ${SOURCE_FILES})

target_link_libraries(mpu6050_node ${catkin_LIBRARIES})
