# Copyright 2021 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# INTERPROCEDURAL_OPTIMIZATION is enforced when enabled.
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
# Default to GLVND if available.
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)

# This line has to appear before 'PROJECT' in order to be able to disable incremental linking
set(MSVC_INCREMENTAL_DEFAULT ON)

project(
  mujoco_simulate
  VERSION 2.2.2
  DESCRIPTION "MuJoCo simulate binaries"
  HOMEPAGE_URL "https://mujoco.org"
)

find_package(mujoco REQUIRED)
set(MUJOCO_INCLUDE_DIRS "/usr/local/include/mujoco")
set(MUJOCO_LIBRARIES "mujoco::mujoco")

enable_language(C)
enable_language(CXX)


list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../cmake")


# Simulate library
FILE(GLOB_RECURSE headers "*.h" "*.hpp")
file(GLOB_RECURSE sources *.cc)

add_library(libsimulate STATIC ${headers} ${sources})
add_library(mujoco::libsimulate ALIAS libsimulate)

target_include_directories(libsimulate PUBLIC ${MUJOCO_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(libsimulate PUBLIC glfw ${MUJOCO_LIBRARIES})

# set(SIMULATE_RESOURCE_FILES "")
# add_executable(simulate main.cc array_safety.h ${SIMULATE_RESOURCE_FILES})
# target_compile_options(simulate PUBLIC ${MUJOCO_SIMULATE_COMPILE_OPTIONS})
# if(WIN32)
#   add_custom_command(
#     TARGET simulate
#     PRE_BUILD
#     COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/../dist/mujoco.ico
#             ${CMAKE_CURRENT_SOURCE_DIR}
#     POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E rm ${CMAKE_CURRENT_SOURCE_DIR}/mujoco.ico
#   )
# endif()

# target_link_libraries(
#   simulate
#   libsimulate
#   mujoco::mujoco
#   glfw
#   Threads::Threads
#   lodepng
# )

# target_link_options(simulate PRIVATE ${MUJOCO_SIMULATE_LINK_OPTIONS})

