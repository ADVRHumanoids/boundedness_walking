project(multidof_walking)
cmake_minimum_required(VERSION 3.0.0)

add_compile_options("-std=c++11")

find_package(OpenMpC REQUIRED)

include_directories(include/ 
                    ${OpenMpC_INCLUDE_DIRS})
                    
add_library(mdof_wpg SHARED 
                src/lipm.cpp 
                src/simple_walker.cpp
                src/foot_trajectory.cpp)    
target_link_libraries(mdof_wpg ${OpenMpC_LIBRARIES})

add_executable(test_1 src/test_1.cpp)
target_link_libraries(test_1 mdof_wpg)


add_subdirectory(xbotcore/)
include(src/gui/gui.cmake)

install(TARGETS mdof_wpg mdof_wpg
    EXPORT mdof_wpg
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/multidof_walking/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)
