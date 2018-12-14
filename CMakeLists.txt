project(multidof_walking)
cmake_minimum_required(VERSION 3.0.0)

add_compile_options("-std=c++11")

find_package(OpenMpC REQUIRED)

include_directories(include/ 
                    ${OpenMpC_INCLUDE_DIRS})
                    
add_library(mdof_wpg SHARED src/lipm.cpp)    

target_link_libraries(mdof_wpg ${OpenMpC_LIBRARIES})