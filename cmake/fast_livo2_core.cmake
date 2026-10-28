find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)
add_library(fast_livo2_support STATIC
  ${FAST_LIVO2_ROOT}/third_party/sophus/so3.cpp
  ${FAST_LIVO2_ROOT}/third_party/sophus/se3.cpp
  ${FAST_LIVO2_ROOT}/third_party/vikit/src/atan_camera.cpp
  ${FAST_LIVO2_ROOT}/third_party/vikit/src/omni_camera.cpp
  ${FAST_LIVO2_ROOT}/third_party/vikit/src/pinhole_camera.cpp
  ${FAST_LIVO2_ROOT}/third_party/vikit/src/equidistant_camera.cpp
  ${FAST_LIVO2_ROOT}/third_party/vikit/src/polynomial_camera.cpp
  ${FAST_LIVO2_ROOT}/third_party/vikit/src/math_utils.cpp
  ${FAST_LIVO2_ROOT}/third_party/vikit/src/vision.cpp)
set_target_properties(fast_livo2_support PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(fast_livo2_support PUBLIC ${FAST_LIVO2_ROOT}/third_party ${FAST_LIVO2_ROOT}/third_party/vikit/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(fast_livo2_support PUBLIC Eigen3::Eigen ${OpenCV_LIBS})
add_library(fast_livo2_core SHARED
  ${FAST_LIVO2_ROOT}/src/core/mapper.cpp
  ${FAST_LIVO2_ROOT}/src/core/input_buffer.cpp
  ${FAST_LIVO2_ROOT}/src/core/synchronization.cpp
  ${FAST_LIVO2_ROOT}/src/core/registration.cpp
  ${FAST_LIVO2_ROOT}/src/core/output.cpp
  ${FAST_LIVO2_ROOT}/src/core/runtime_log.cpp
  ${FAST_LIVO2_ROOT}/src/IMU_Processing.cpp
  ${FAST_LIVO2_ROOT}/src/preprocess.cpp
  ${FAST_LIVO2_ROOT}/src/preprocess_features.cpp
  ${FAST_LIVO2_ROOT}/src/preprocess_planes.cpp
  ${FAST_LIVO2_ROOT}/src/voxel_map.cpp
  ${FAST_LIVO2_ROOT}/src/vio.cpp
  ${FAST_LIVO2_ROOT}/src/visual_projection.cpp
  ${FAST_LIVO2_ROOT}/src/visual_retrieval.cpp
  ${FAST_LIVO2_ROOT}/src/visual_map.cpp
  ${FAST_LIVO2_ROOT}/src/visual_alignment.cpp
  ${FAST_LIVO2_ROOT}/src/visual_output.cpp
  ${FAST_LIVO2_ROOT}/src/frame.cpp
  ${FAST_LIVO2_ROOT}/src/visual_point.cpp)
target_compile_features(fast_livo2_core PUBLIC cxx_std_17)
target_compile_features(fast_livo2_support PUBLIC cxx_std_17)
target_compile_definitions(fast_livo2_core PRIVATE ROOT_DIR="${FAST_LIVO2_ROOT}/" MP_EN)
target_include_directories(fast_livo2_core PUBLIC $<BUILD_INTERFACE:${FAST_LIVO2_ROOT}/include> $<INSTALL_INTERFACE:include> ${OpenCV_INCLUDE_DIRS})
target_link_libraries(fast_livo2_core PUBLIC Eigen3::Eigen pcl_common ${OpenCV_LIBS}
  PRIVATE fast_livo2_support ${PCL_LIBRARIES} OpenMP::OpenMP_CXX Threads::Threads)
set_target_properties(fast_livo2_core PROPERTIES CXX_EXTENSIONS OFF)
install(TARGETS fast_livo2_core EXPORT fast_livo2_coreTargets LIBRARY DESTINATION lib)
install(DIRECTORY ${FAST_LIVO2_ROOT}/include/fast_livo2 DESTINATION include)
target_link_options(fast_livo2_core PRIVATE "LINKER:--no-undefined")

include(CMakePackageConfigHelpers)
configure_package_config_file(${FAST_LIVO2_ROOT}/cmake/fast_livo2_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/fast_livo2_coreConfig.cmake
  INSTALL_DESTINATION lib/cmake/fast_livo2_core)
install(EXPORT fast_livo2_coreTargets NAMESPACE fast_livo2:: DESTINATION lib/cmake/fast_livo2_core)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/fast_livo2_coreConfig.cmake DESTINATION lib/cmake/fast_livo2_core)
