cmake_minimum_required(VERSION 3.24)
project(hpl_mjpc_bridge LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
find_package(Threads REQUIRED)
include(FetchContent)
set(ABSL_PROPAGATE_CXX_STD ON)
set(ABSL_BUILD_TESTING OFF)
FetchContent_Declare(abseil GIT_REPOSITORY https://github.com/abseil/abseil-cpp.git
  GIT_TAG fb3621f4f897824c0dbe0615fa94543df6192f30)
FetchContent_MakeAvailable(abseil)
if(NOT EXISTS "${MJPC_SOURCE}/mjpc/planners/sampling/planner.cc")
  message(FATAL_ERROR "MJPC_SOURCE must be the pinned external checkout")
endif()
# Reproducible noise only: no changes to upstream cost/planning/physics.
file(READ "${MJPC_SOURCE}/mjpc/planners/sampling/planner.cc" sampling)
string(FIND "${sampling}" "absl::BitGen gen_;" seed_at)
if(seed_at LESS 0)
  message(FATAL_ERROR "Upstream RNG changed; review the deterministic patch")
endif()
string(REPLACE "absl::BitGen gen_;"
  "std::seed_seq seed{20261116u, static_cast<unsigned>(i), static_cast<unsigned>(std::llround(start_time * 1000000))}; absl::BitGen gen_(seed);"
  sampling "${sampling}")
file(WRITE "${CMAKE_BINARY_DIR}/sampling.cc" "#include <random>\n${sampling}")
add_library(mujoco SHARED IMPORTED)
set_target_properties(mujoco PROPERTIES
  IMPORTED_IMPLIB "${MUJOCO_SDK}/lib/mujoco.lib"
  IMPORTED_LOCATION "${MUJOCO_SDK}/bin/mujoco.dll"
  INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_SDK}/include")
file(READ "${MJPC_SOURCE}/mjpc/utilities.cc" utilities)
string(FIND "${utilities}" "bodyexclude, &geomid);" ray_at)
if(ray_at LESS 0)
  message(FATAL_ERROR "Upstream ray ABI changed; review the compatibility patch")
endif()
string(REPLACE "bodyexclude, &geomid);" "bodyexclude, &geomid, nullptr);" utilities "${utilities}")
file(WRITE "${CMAKE_BINARY_DIR}/utilities.cc" "${utilities}")
add_library(hpl_mjpc SHARED bridge.cc "${CMAKE_BINARY_DIR}/sampling.cc"
  "${MJPC_SOURCE}/mjpc/planners/planner.cc"
  "${MJPC_SOURCE}/mjpc/planners/sampling/policy.cc"
  "${MJPC_SOURCE}/mjpc/spline/spline.cc"
  "${MJPC_SOURCE}/mjpc/states/state.cc"
  "${MJPC_SOURCE}/mjpc/trajectory.cc"
  "${MJPC_SOURCE}/mjpc/task.cc"
  "${MJPC_SOURCE}/mjpc/norm.cc"
  "${CMAKE_BINARY_DIR}/utilities.cc"
  "${MJPC_SOURCE}/mjpc/threadpool.cc")
target_include_directories(hpl_mjpc PRIVATE "${MJPC_SOURCE}")
target_compile_definitions(hpl_mjpc PRIVATE NOMINMAX _USE_MATH_DEFINES)
target_link_libraries(hpl_mjpc PRIVATE mujoco Threads::Threads absl::base
  absl::flat_hash_map absl::check absl::log absl::random_random absl::span absl::strings)
if(MSVC)
  target_compile_options(hpl_mjpc PRIVATE /O2 /EHsc)
endif()
