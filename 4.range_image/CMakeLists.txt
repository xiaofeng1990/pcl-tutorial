cmake_minimum_required(VERSION 3.10)
project(PCLTutorial CXX C)

# executable file output path
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin" CACHE PATH "output directory")
# library file output path
set(LIBRARY_OUTPUT_PATH    "${CMAKE_BINARY_DIR}/lib" CACHE PATH "library output directory")
# cmake install path
set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install")
# runtime 搜索路径
set(CMAKE_INSTALL_RPATH ".:lib:build/install/lib")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(XF_COMPILE_FLAGS "-g -O2 -ffunction-sections -fdata-sections -funwind-tables")
set(XF_COMPILE_FLAGS "${XF_COMPILE_FLAGS} -fstack-protector-strong")

set(CMAKE_C_FLAGS    "${XF_COMPILE_FLAGS}")
set(CMAKE_CXX_FLAGS  "${XF_COMPILE_FLAGS}")

find_package(PCL 1.3 REQUIRED)
if(PCL_FOUND)
    message(STATUS "PCL library status:")
    message(STATUS "    version: ${PCL_VERSION}")
    message(STATUS "    include path: ${PCL_INCLUDE_DIRS}")
    message(STATUS "    lib path: ${PCL_LIBRARY_DIRS}")
    message(STATUS "    lib : ${PCL_LIBRARIES}")
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    set(LINK_LIBS ${LINK_LIBS} ${PCL_LIBRARIES})
else()
    message(FATA_ERROR "Do not find PCL")
endif()


add_subdirectory(4.1.range_image_creation)
add_subdirectory(4.2.range_image_border_extraction)