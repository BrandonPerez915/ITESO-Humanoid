# ================================================================
# cmake -S ./ -B ./build/
# cmake --build ./build/ --target all
# ================================================================

cmake_minimum_required(VERSION 3.10)
project(humanoid)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

if(DEFINED TORCH_PATH)
  find_package(Torch QUIET PATHS ${TORCH_PATH})
else()
  find_package(Torch QUIET)
endif()

if(Torch_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
endif()

include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG yaml-cpp-0.7.0
)
FetchContent_MakeAvailable(yaml-cpp)


# ================================ main application ================================

add_executable(main
    core/main.cpp
    core/udp.c
    core/serializer.c
    core/imu.cpp
    core/socketcan.cpp
    core/motor_controller.cpp
    core/real_humanoid.cpp
)

set_property(TARGET main PROPERTY CXX_STANDARD 17)

target_include_directories(main PRIVATE core core/lib)

target_link_libraries(main yaml-cpp::yaml-cpp)


# ================================ test applications ================================

# Torch (optional test)
if(Torch_FOUND)
  add_executable(test-torch tests/test-torch.cpp)
  target_link_libraries(test-torch "${TORCH_LIBRARIES}")
  set_property(TARGET test-torch PROPERTY CXX_STANDARD 17)
endif()


# IMU
add_executable(test-imu tests/test-imu.cpp core/imu.cpp)
target_include_directories(test-imu PRIVATE core core/lib)
set_property(TARGET test-imu PROPERTY CXX_STANDARD 17)


# UDP
add_executable(test-udp tests/test-udp.cpp core/udp.c)
target_include_directories(test-udp PRIVATE core core/lib)
set_property(TARGET test-udp PROPERTY CXX_STANDARD 17)


# YAML
add_executable(test-yaml tests/test-yaml.cpp)
target_include_directories(test-yaml PRIVATE core core/lib)
target_link_libraries(test-yaml yaml-cpp::yaml-cpp)
set_property(TARGET test-yaml PROPERTY CXX_STANDARD 17)
