cmake_minimum_required(VERSION 3.0)
project(bspline_fitting CXX C)
set(APP_NAME bspline_fitting)

# 添加目录中所有源文件
# aux_source_directory(${PROJECT_SOURCE_DIR}/src   SRC_FILES)
file(GLOB_RECURSE SRC_FILES ${PROJECT_SOURCE_DIR}/*.cpp)
add_executable(${APP_NAME} ${SRC_FILES})
target_link_libraries(${APP_NAME} ${LINK_LIBS})
# message("link libs ${LINK_LIBS}")
install(TARGETS ${APP_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

# set_target_properties(${TARGET_NAME}  PROPERTIES LINK_FLAGS "-Wl,--disable-new-dtags,-rpath,${CMAKE_INSTALL_PREFIX}/lib:${FASTDEPLOY_ASCEND_DIR}/third_libs/install/opencv/lib")
set_target_properties(${APP_NAME}  PROPERTIES LINK_FLAGS "-Wl,--disable-new-dtags,-rpath,/home/ubuntu/workspaces/wxf/pcl/install/lib")
