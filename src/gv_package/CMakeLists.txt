cmake_minimum_required(VERSION 3.8)
project(gv_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-error=pedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(OpenSSL REQUIRED)
find_package(yaml-cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# SOEM 라이브러리 포함 및 빌드
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/include/soem)

#include directory
include_directories(
  include
  /usr/local/include
)
link_directories(/usr/local/lib)

# 실행 파일 생성
add_executable(MotorControl src/control.cpp src/GVdrive.cpp)

target_include_directories(MotorControl PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
target_compile_features(MotorControl PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17)

# ROS, Boost, OpenSSL 의존성
 ament_target_dependencies(MotorControl
   rclcpp
   std_msgs
   ament_index_cpp
   Boost
   OpenSSL
   yaml-cpp
 )

# SOEM 라이브러리 링크
target_link_libraries(MotorControl 
  soem
  OpenSSL::SSL
  OpenSSL::Crypto
  pthread
  Boost::system
  Boost::filesystem
  yaml-cpp
 )

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS MotorControl
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
