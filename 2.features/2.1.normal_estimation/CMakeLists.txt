cmake_minimum_required(VERSION 3.0)
project(normal_estimation_using_integral_images CXX C)
set(APP_NAME normal_estimation_using_integral_images)

# 添加目录中所有源文件
# aux_source_directory(${PROJECT_SOURCE_DIR}/src   SRC_FILES)
file(GLOB_RECURSE SRC_FILES ${PROJECT_SOURCE_DIR}/*.cpp)
add_executable(${APP_NAME} ${SRC_FILES})
target_link_libraries(${APP_NAME} ${LINK_LIBS})
# message("link libs ${LINK_LIBS}")
install(TARGETS ${APP_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)